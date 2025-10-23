#include "bsp_audio.h"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static esp_err_t _play(const char *source, audio_play_format_t format);
static esp_err_t _set_output_format(uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channel_count);
extern esp_err_t wav_play(audio_handle_t handle, const char *path);


static const char *TAG = "bsp_audio";

static audio_handle_t _audio_handle = NULL;
static TaskHandle_t   _output_task_handle = NULL;
static audio_queue_node_t*          _audio_queue_nodes[AUDIO_QUEUE_SIZE];


audio_handle_t bsp_audio_handle_get(void)
{
    return _audio_handle;
}

static esp_err_t _read(void *data, size_t size, size_t *bytes_read, TickType_t ticks_to_wait)
{
    return i2s_channel_read(_audio_handle->_rx_handle, data, size, bytes_read, ticks_to_wait);
}

static esp_err_t _write(const void *data, size_t size)
{
    esp_err_t ret = ESP_OK;

    if(data == NULL || size == 0) {
        return ESP_OK;
    }

    if(size > AUDIO_QUEUE_NODE_SIZE) {
        ESP_LOGE(TAG, "payload %u exceeds node capacity %u", (unsigned)size, (unsigned)AUDIO_QUEUE_NODE_SIZE);
        return ESP_ERR_INVALID_SIZE;
    }

    uint8_t index;
    for(index = 0; index < AUDIO_QUEUE_SIZE; index++) {
        if(!_audio_queue_nodes[index]->is_busy) {
            break;
        }
    }
    if(index == AUDIO_QUEUE_SIZE) {
        ESP_LOGW(TAG, "audio queue full");
        return ESP_ERR_NO_MEM;
    }

    audio_queue_node_t *node = _audio_queue_nodes[index];
    memcpy(node->data, data, size);
    node->size = size;
    node->is_busy = true;
    xQueueSend(_audio_handle->_queue, &node, pdMS_TO_TICKS(500));
    return ret;
    // return i2s_channel_write(_audio_handle->_tx_handle, data, size, bytes_written, ticks_to_wait);
}

static esp_err_t _play(const char *source, audio_play_format_t format)
{
    if(_audio_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if(source == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    switch(format) {
        case AUDIO_PLAY_FORMAT_WAV:
            extern esp_err_t wav_play(audio_handle_t handle, const char *path);
            return wav_play(_audio_handle, source);
        case AUDIO_PLAY_FORMAT_PCM:
        case AUDIO_PLAY_FORMAT_MP3:
            ESP_LOGW(TAG, "audio format %d not supported", format);
            return ESP_ERR_NOT_SUPPORTED;
        default:
            ESP_LOGE(TAG, "unknown audio format %d", format);
            return ESP_ERR_INVALID_ARG;
    }
}

static esp_err_t _wait_queue_idle(void)
{
    if(_audio_handle == NULL || _audio_handle->_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const TickType_t delay_ticks = pdMS_TO_TICKS(5);
    const int max_attempts = 200; // ~1s

    for(int attempt = 0; attempt < max_attempts; ++attempt) {
        bool busy = false;
        for(int i = 0; i < AUDIO_QUEUE_SIZE; ++i) {
            if(_audio_queue_nodes[i]->is_busy) {
                busy = true;
                break;
            }
        }

        if(!busy && uxQueueMessagesWaiting(_audio_handle->_queue) == 0) {
            return ESP_OK;
        }

        vTaskDelay(delay_ticks);
    }

    ESP_LOGW(TAG, "timeout waiting for audio queue to drain");
    xQueueReset(_audio_handle->_queue);
    for(int i = 0; i < AUDIO_QUEUE_SIZE; ++i) {
        _audio_queue_nodes[i]->is_busy = false;
        _audio_queue_nodes[i]->size = 0;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t _set_output_format(uint32_t sample_rate, uint16_t bits_per_sample, uint16_t channel_count)
{
    if(_audio_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if(channel_count == 0 || channel_count > 2) {
        ESP_LOGE(TAG, "unsupported channel count %u", (unsigned)channel_count);
        return ESP_ERR_INVALID_ARG;
    }

    if(bits_per_sample != 16 && bits_per_sample != 24 && bits_per_sample != 32) {
        ESP_LOGE(TAG, "unsupported bit depth %u", (unsigned)bits_per_sample);
        return ESP_ERR_INVALID_ARG;
    }

    if(sample_rate < 8000 || sample_rate > 48000) {
        ESP_LOGE(TAG, "unsupported sample rate %u", (unsigned)sample_rate);
        return ESP_ERR_INVALID_ARG;
    }

    if(_audio_handle->_tx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t wait_ret = _wait_queue_idle();
    if(wait_ret != ESP_OK) {
        ESP_LOGW(TAG, "queue drain result: %s", esp_err_to_name(wait_ret));
    }

    esp_err_t ret = i2s_channel_disable(_audio_handle->_tx_handle);
    if(ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "failed to disable tx channel: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_data_bit_width_t data_width = I2S_DATA_BIT_WIDTH_16BIT;
    switch(bits_per_sample) {
        case 16:
            data_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case 24:
            data_width = I2S_DATA_BIT_WIDTH_24BIT;
            break;
        case 32:
            data_width = I2S_DATA_BIT_WIDTH_32BIT;
            break;
        default:
            break;
    }

    i2s_slot_mode_t slot_mode = (channel_count == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    ret = i2s_channel_reconfig_std_clock(_audio_handle->_tx_handle, &clk_cfg);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to reconfig clock: %s", esp_err_to_name(ret));
        return ret;
    }

    i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(data_width, slot_mode);
    slot_cfg.slot_mask = (channel_count == 1) ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_BOTH;

    ret = i2s_channel_reconfig_std_slot(_audio_handle->_tx_handle, &slot_cfg);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to reconfig slot: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2s_channel_enable(_audio_handle->_tx_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to enable tx channel: %s", esp_err_to_name(ret));
        return ret;
    }

    _audio_handle->_output_sample_rate = sample_rate;
    _audio_handle->_bits_per_sample = bits_per_sample;
    _audio_handle->_channel_count = channel_count;

    return ESP_OK;
}

static void _output_task(void *arg)
{
    esp_err_t ret = ESP_OK;
    for(;;) {
        audio_queue_node_t *node = NULL;
        if(xQueueReceive(_audio_handle->_queue, &node, portMAX_DELAY) == pdTRUE) {
            ret = i2s_channel_write(_audio_handle->_tx_handle, node->data, node->size, NULL, pdMS_TO_TICKS(1000));
            if(ret != ESP_OK) {
                ESP_LOGE(TAG, "i2s_channel_write failed: %s", esp_err_to_name(ret));
            }
            node->is_busy = false;
        }
    }
}

esp_err_t _del()
{
    if(_audio_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_OK;

    if(_audio_handle->_tx_handle) {
        ret = i2s_channel_disable(_audio_handle->_tx_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to disable tx channel: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = i2s_del_channel(_audio_handle->_tx_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to delete tx channel: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    if(_audio_handle->_rx_handle) {
        ret = i2s_channel_disable(_audio_handle->_rx_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to disable rx channel: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = i2s_del_channel(_audio_handle->_rx_handle);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to delete rx channel: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    vQueueDelete(_audio_handle->_queue);

    for(uint8_t i = 0; i < AUDIO_QUEUE_SIZE; i++) {
        if(_audio_queue_nodes[i]) {
            free(_audio_queue_nodes[i]->data);
            free(_audio_queue_nodes[i]);
            _audio_queue_nodes[i] = NULL;
        }
    }

    if(_output_task_handle) {
        vTaskDelete(_output_task_handle);
        _output_task_handle = NULL;
    }

    free(_audio_handle);
    _audio_handle = NULL;

    return ESP_OK;
}

esp_err_t audio_init(int input_sample_rate, int output_sample_rate, gpio_num_t bclk, gpio_num_t ws, gpio_num_t dout, gpio_num_t din)
{
    esp_err_t ret = ESP_OK;

    audio_handle_t _handle = (audio_handle_t)calloc(1, sizeof(struct audio_t));
    assert(_handle != NULL);

    _handle->_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(audio_queue_node_t*));
    assert(_handle->_queue != NULL);

    for(uint8_t i = 0; i < AUDIO_QUEUE_SIZE; i++) {
        _audio_queue_nodes[i] = (audio_queue_node_t*)calloc(1, sizeof(audio_queue_node_t));
        assert(_audio_queue_nodes[i] != NULL);
        _audio_queue_nodes[i]->data = heap_caps_calloc(1, AUDIO_QUEUE_NODE_SIZE, MALLOC_CAP_SPIRAM);
        assert(_audio_queue_nodes[i]->data != NULL);
        _audio_queue_nodes[i]->size = 0;
        _audio_queue_nodes[i]->is_busy = false;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &_handle->_tx_handle, &_handle->_rx_handle));

    i2s_std_config_t std_cfg = {
		.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(input_sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = dout,
            .din = din,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(_handle->_tx_handle, &std_cfg));

    std_cfg.clk_cfg.sample_rate_hz = output_sample_rate;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(_handle->_rx_handle, &std_cfg));

    if(_handle->_tx_handle) {
        ESP_ERROR_CHECK(i2s_channel_enable(_handle->_tx_handle));
    }

    if(_handle->_rx_handle) {
        ESP_ERROR_CHECK(i2s_channel_enable(_handle->_rx_handle));
    }

    _handle->read = _read;
    _handle->write = _write;
    _handle->play = _play;
    _handle->del = _del;
    _handle->set_output_format = _set_output_format;

    _handle->_output_sample_rate = output_sample_rate;
    _handle->_input_sample_rate = input_sample_rate;
    _handle->_bits_per_sample = 16;
    _handle->_channel_count = 1;

    _audio_handle = _handle;

    xTaskCreate(_output_task, "audio output", 4096, NULL, 5, &_output_task_handle);

    return ret;
}

