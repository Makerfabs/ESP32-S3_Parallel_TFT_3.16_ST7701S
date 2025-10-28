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
static esp_err_t _set_output_format(uint32_t sample_rate, uint32_t bits_per_sample, i2s_slot_mode_t ch);
extern esp_err_t wav_play(bsp_audio_handle_t handle, const char *path);


static const char *TAG = "bsp_audio";

static bsp_audio_handle_t _audio_handle         = NULL;
static TaskHandle_t   _audio_queue_task_handle  = NULL;
static i2s_chan_handle_t _tx_handle             = NULL;
static i2s_chan_handle_t _rx_handle             = NULL;
static bsp_audio_queue_node_t*                  _audio_queue_nodes[AUDIO_QUEUE_SIZE];


static esp_err_t _read(void *data, size_t to_read, size_t *bytes_read, uint32_t ticks_to_wait)
{
    // esp_err_t ret = ESP_OK;

    // int32_t bit32_buffer[to_read / sizeof(int32_t)];
    // ret = i2s_channel_read(_rx_handle, bit32_buffer, to_read, bytes_read, ticks_to_wait);
    // if(ret != ESP_OK) {
    //     ESP_LOGE(TAG, "i2s_channel_read failed: %s", esp_err_to_name(ret));
    //     return ret;
    // }


    // to_read = *bytes_read / sizeof(int32_t);
    // int16_t *data16 = (int16_t *)data;
    // for(int i = 0; i < to_read; i++) {
    //     int32_t value = bit32_buffer[i] >> 12;
    //     data16[i] = (value > INT16_MAX) ? INT16_MAX : (value < INT16_MIN) ? INT16_MIN : (int16_t)value;
    // }
    return i2s_channel_read(_rx_handle, data, to_read, bytes_read, ticks_to_wait);
}

static esp_err_t _write(void *data, size_t to_write, size_t *bytes_written, uint32_t ticks_to_wait)
{
    // const size_t samples = to_write / sizeof(int32_t);

    // int32_t buffer[samples];
    // int32_t volume_factor = 0.75 * 65536;

    // int16_t *data16 = (int16_t *)data;
    // for(size_t i = 0; i < samples; i++) {
    //     int64_t temp = (int64_t)data16[i] * volume_factor;
    //     if(temp > INT32_MAX) {
    //         buffer[i] = INT32_MAX;
    //     } else if(temp < INT32_MIN) {
    //         buffer[i] = INT32_MIN;
    //     } else {
    //         buffer[i] = (int32_t)(temp);
    //     }
    // }

    return i2s_channel_write(_tx_handle, data, to_write, bytes_written, ticks_to_wait);
}

static esp_err_t _add_queue_node(void *data, size_t size)
{ 
    esp_err_t ret = ESP_OK;

    if(data == NULL || size == 0) {
        return ESP_ERR_INVALID_ARG;
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

    bsp_audio_queue_node_t *node = _audio_queue_nodes[index];
    memcpy(node->data, data, size);
    node->size = size;
    node->is_busy = true;
    if(xQueueSend(_audio_handle->_audio_queue, &node, pdMS_TO_TICKS(500)) != pdPASS) {
        ESP_LOGW(TAG, "audio queue send timeout");
        node->is_busy = false;
        return ESP_ERR_TIMEOUT;
    }
    return ret;
}

static esp_err_t _play(const char *source, audio_play_format_t format)
{
    if(_audio_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if(source == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // switch(format) {
    //     case AUDIO_PLAY_FORMAT_WAV:
    //         extern esp_err_t wav_play(bsp_audio_handle_t handle, const char *path);
    //         return wav_play(_audio_handle, source);
    //     case AUDIO_PLAY_FORMAT_PCM:
    //     case AUDIO_PLAY_FORMAT_MP3:
    //         ESP_LOGW(TAG, "audio format %d not supported", format);
    //         return ESP_ERR_NOT_SUPPORTED;
    //     default:
    //         ESP_LOGE(TAG, "unknown audio format %d", format);
    //         return ESP_ERR_INVALID_ARG;
    // }
    return ESP_OK;
}

static esp_err_t _mute(AUDIO_PLAYER_MUTE_SETTING setting)
{
    if(_audio_handle == NULL && _tx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const size_t silent_size = 512;

    if(setting == AUDIO_PLAYER_MUTE) {
        int16_t silent_buf[silent_size];
        memset(silent_buf, 0, sizeof(silent_buf));
        for(int i = 0; i < 50; i++) {
            i2s_channel_write(_tx_handle, silent_buf, silent_size, NULL, pdMS_TO_TICKS(100));
        }
    }
    return ESP_OK;
}

static esp_err_t _wait_queue_idle(void)
{
    if(_audio_handle == NULL || _audio_handle->_audio_queue == NULL) {
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

        if(!busy && uxQueueMessagesWaiting(_audio_handle->_audio_queue) == 0) {
            return ESP_OK;
        }

        vTaskDelay(delay_ticks);
    }

    ESP_LOGW(TAG, "timeout waiting for audio queue to drain");
    xQueueReset(_audio_handle->_audio_queue);
    for(int i = 0; i < AUDIO_QUEUE_SIZE; ++i) {
        _audio_queue_nodes[i]->is_busy = false;
        _audio_queue_nodes[i]->size = 0;
    }
    return ESP_ERR_TIMEOUT;
}

static esp_err_t _set_output_format(uint32_t sample_rate, uint32_t bits_per_sample, i2s_slot_mode_t ch)
{
    esp_err_t ret = ESP_OK;

    if(_audio_handle == NULL && _tx_handle == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if(ch == 0 || ch > 2) {
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

    // ret = _wait_queue_idle();
    // if(ret != ESP_OK) {
    //     ESP_LOGW(TAG, "queue drain result: %s", esp_err_to_name(ret));
    // }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG((i2s_data_bit_width_t)bits_per_sample, ch),
        .gpio_cfg = BSP_AUDIO_I2S_GPIO_CFG,
    };

    ret |= i2s_channel_disable(_tx_handle);
    ret |= i2s_channel_reconfig_std_clock(_tx_handle, &std_cfg.clk_cfg);
    ret |= i2s_channel_reconfig_std_slot(_tx_handle, &std_cfg.slot_cfg);
    ret |= i2s_channel_enable(_tx_handle);

    _audio_handle->output_sample_rate   = sample_rate;
    _audio_handle->bits_per_sample      = bits_per_sample;
    _audio_handle->channel_count        = ch;

    return ret;
}

static void _audio_queue_task(void *arg)
{
    esp_err_t ret = ESP_OK;
    for(;;) {
        bsp_audio_queue_node_t *node = NULL;
        if(xQueueReceive(_audio_handle->_audio_queue, &node, portMAX_DELAY) == pdTRUE) {
            ret = i2s_channel_write(_tx_handle, node->data, node->size, NULL, pdMS_TO_TICKS(1000));
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

    if(_tx_handle) ret |= i2s_channel_disable(_tx_handle), ret |= i2s_del_channel(_tx_handle);
    if(_rx_handle) ret |= i2s_channel_disable(_rx_handle), ret |= i2s_del_channel(_rx_handle);



    vQueueDelete(_audio_handle->_audio_queue);

    for(uint8_t i = 0; i < AUDIO_QUEUE_SIZE; i++) {
        if(_audio_queue_nodes[i]) {
            free(_audio_queue_nodes[i]->data);
            free(_audio_queue_nodes[i]);
            _audio_queue_nodes[i] = NULL;
        }
    }

    if(_audio_queue_task_handle) {
        vTaskDelete(_audio_queue_task_handle);
        _audio_queue_task_handle = NULL;
    }

    return ESP_OK;
}

esp_err_t bsp_audio_init(bsp_audio_handle_t *handle, bool enable_audio_queue_task)
{
    esp_err_t ret = ESP_OK;

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ret |= i2s_new_channel(&chan_cfg, &_tx_handle, &_rx_handle);

    i2s_std_config_t std_cfg = BSP_AUDIO_I2S_DUPLEX_MONO_CFG(AUDIO_SAMPLE_RATE);
    // {
	// 	.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
    //     .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(AUDIO_DATA_BIT_WIDTH, AUDIO_SLOT_MODE),
    //     .gpio_cfg = {
    //         .mclk = I2S_GPIO_UNUSED,
    //         .bclk = AUDIO_I2S_GPIO_BCLK,
    //         .ws = AUDIO_I2S_GPIO_WS,
    //         .dout = AUDIO_I2S_GPIO_DOUT,
    //         .din = AUDIO_I2S_GPIO_DIN,
    //         .invert_flags = {
    //             .mclk_inv = false,
    //             .bclk_inv = false,
    //             .ws_inv = false
    //         }
    //     }
    // };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

    ret |= i2s_channel_init_std_mode(_tx_handle, &std_cfg);
    ret |= i2s_channel_init_std_mode(_rx_handle, &std_cfg);

    if(_tx_handle) ret |= i2s_channel_enable(_tx_handle);
    if(_rx_handle) ret |= i2s_channel_enable(_rx_handle);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to init i2s: %s", esp_err_to_name(ret));
        return ret;
    }
    bsp_audio_handle_t _handle = (bsp_audio_handle_t)calloc(1, sizeof(struct bsp_audio_t));
    assert(_handle != NULL);

    _handle->read = _read;
    _handle->write = _write;
    _handle->add_queue_node = _add_queue_node;
    _handle->play = _play;
    _handle->mute = _mute;
    _handle->del = _del;
    _handle->set_output_format = _set_output_format;

    _handle->output_sample_rate = AUDIO_SAMPLE_RATE;
    _handle->input_sample_rate = AUDIO_SAMPLE_RATE;
    _handle->bits_per_sample = AUDIO_BITS_PER_SAMPLE;
    _handle->channel_count = AUDIO_SLOT_MODE;

    if(enable_audio_queue_task) {
        _handle->_audio_queue = xQueueCreate(AUDIO_QUEUE_SIZE, sizeof(bsp_audio_queue_node_t*));
        assert(_handle->_audio_queue != NULL);

        for(uint8_t i = 0; i < AUDIO_QUEUE_SIZE; i++) {
            _audio_queue_nodes[i] = (bsp_audio_queue_node_t*)calloc(1, sizeof(bsp_audio_queue_node_t));
            assert(_audio_queue_nodes[i] != NULL);
            _audio_queue_nodes[i]->data = heap_caps_calloc(1, AUDIO_QUEUE_NODE_SIZE, MALLOC_CAP_SPIRAM);
            assert(_audio_queue_nodes[i]->data != NULL);
            _audio_queue_nodes[i]->size = 0;
            _audio_queue_nodes[i]->is_busy = false;
        }
        // xTaskCreate(_audio_queue_task, "audio queue", 4096, NULL, 5, &_audio_queue_task_handle);
    }

    *(handle) = _handle;
    _audio_handle = _handle;
    return ret;
}

