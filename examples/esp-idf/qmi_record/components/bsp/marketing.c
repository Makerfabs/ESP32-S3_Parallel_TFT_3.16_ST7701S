// Marketing demo: simple slideshow that shows assets images one per second with basic fade-in

#include "bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>

#include "audio/bsp_audio.h"
#include "qmi8658/bsp_qmi8658.h"
#include "sdcard/bsp_sdcard.h"
#include "audio/wav.h"




#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include "weather.h"
#include <stdio.h>

// Extern image descriptors generated under components/bsp/assets
extern const lv_image_dsc_t img1;
extern const lv_image_dsc_t img2;
extern const lv_image_dsc_t img3;
extern const lv_image_dsc_t img4;
extern const lv_image_dsc_t img5;


#define AUDIO_EVENT_RECORDING       (1 << 0)
#define AUDIO_EVENT_SAVING          (1 << 1)
#define AUDIO_EVENT_PLAYING         (1 << 2)
#define AUDIO_EVENT_PLAYING_STOP    (1 << 3)

#define ONE_G                   9.80665f    // 重力加速度 (m/s^2)
#define ACCEL_TOLERANCE_G       2.7f        // 容忍度（公差），例如 4.5 m/s^2

#define QMI8658_ORIENTATION_DURATION_MS 1000

typedef enum {
    ORIENTATION_FLAT_Z,      // 平放（Z轴垂直于地面）
    ORIENTATION_UPRIGHT_Y,   // 竖立（Y轴垂直于地面）
    ORIENTATION_SIDELAY_X,   // 侧放（X轴垂直于地面）
    ORIENTATION_UNKNOWN      // 倾斜或运动中
} qmi8658_orientation_e;

typedef struct {
    qmi8658_orientation_e current_stable_orientation; // 当前持续稳定的姿态
    qmi8658_orientation_e last_stable_orientation;  // 上一次检测到的姿态
    TickType_t stable_start_tick;                    // 姿态开始稳定的时间点
    TickType_t required_ticks;                       // 保持姿态所需的时钟周期数
} qmi8658_orientation_info_t;

static const char *TAG = "marketing";

typedef struct {
	TaskHandle_t task;
	volatile bool is_completed;
	uint32_t timeout_ticks; // interval between images
} marketing_ctx_t;

static bsp_audio_handle_t _g_audio_handle = NULL;
static bsp_qmi8658_handle_t _g_imu_handle = NULL;
static qmi8658_orientation_e _g_orientation = ORIENTATION_UNKNOWN;

static EventGroupHandle_t _g_audio_event_group = NULL;
static qmi8658_orientation_info_t _g_orientation_info;

static lv_obj_t *state_label = NULL;


static void lv_update_state()
{
    if(_g_orientation_info.current_stable_orientation != _g_orientation_info.last_stable_orientation) {
        char text[64] = {0};
        switch(_g_orientation_info.current_stable_orientation) {
            case ORIENTATION_UPRIGHT_Y:
                snprintf(text, sizeof(text), "Recording...");
                break;
            case ORIENTATION_SIDELAY_X:
                snprintf(text, sizeof(text), "Playing...");
                break;
            default:
                snprintf(text, sizeof(text), "Idle...");
                break;
        }
        bsp_display_lock(0);
        lv_label_set_text(state_label, text);
        bsp_display_unlock();
    }
}



static void imu_switch_jpg_task(void *arg)
{
    bsp_extra_qmi8658_init(&_g_imu_handle);

    float change_rate = 2.5f;
    /* baseline 用于低通滤波噪声，避免因瞬时方向相消导致误判 */
    float baseline = 0.0f;
    /* 切换后的最小冷却时间，避免一次挥动来回导致两次切换（ms） */
    const TickType_t SWITCH_COOLDOWN_MS = 600;
    TickType_t last_switch_tick = 0;

    const lv_image_dsc_t *imgs[] = { &img1, &img2, &img3, &img4, &img5 };
    const size_t img_cnt = sizeof(imgs) / sizeof(imgs[0]);
    size_t idx = 0;

    bsp_display_lock(0);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    lv_obj_t *img_obj = lv_image_create(scr);
    lv_image_set_src(img_obj, imgs[idx]);
    bsp_display_unlock();

    while(true) {
        float ax, ay, az;
        float gx, gy, gz;
        if(_g_imu_handle->read_accel(&ax, &ay, &az) == ESP_OK &&
           _g_imu_handle->read_gyro(&gx, &gy, &gz) == ESP_OK) {
            /* 用绝对值之和作为 motion magnitude，避免分量相互抵消 */
            float mag = fabs(ax) + fabs(ay) + fabs(az) + fabs(gx) + fabs(gy) + fabs(gz);

            /* 简单一阶低通求 baseline（可调） */
            const float alpha = 0.12f; /* 低通系数，越小响应越慢 */
            baseline = (1.0f - alpha) * baseline + alpha * mag;

            TickType_t now = xTaskGetTickCount();

            /* 只有当 mag 明显高于 baseline 且超过冷却时间时才切换 */
            if ((mag - baseline) > change_rate && (now - last_switch_tick) >= pdMS_TO_TICKS(SWITCH_COOLDOWN_MS)) {
                idx = (idx + 1) % img_cnt;
                bsp_display_lock(0);
                lv_image_set_src(img_obj, imgs[idx]);
                bsp_display_unlock();
                last_switch_tick = now;
                ESP_LOGI(TAG, "Significant motion detected, switched image index=%d.", idx);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}



/*
 * @brief  for record example
 */
static void _detect_orientation(float ax, float ay, float az)
{
    // 我们只关注竖立（Y 轴）和侧放（X 轴）。其他情况归为 UNKNOWN。
    qmi8658_orientation_e raw_orientation = ORIENTATION_UNKNOWN;
    if (fabs(ay) > (ONE_G - ACCEL_TOLERANCE_G) && fabs(ax) <= ACCEL_TOLERANCE_G && fabs(az) <= ACCEL_TOLERANCE_G) {
        raw_orientation = ORIENTATION_UPRIGHT_Y;
    } else if (fabs(ax) > (ONE_G - ACCEL_TOLERANCE_G) && fabs(ay) <= ACCEL_TOLERANCE_G && fabs(az) <= ACCEL_TOLERANCE_G) {
        raw_orientation = ORIENTATION_SIDELAY_X;
    } else {
        raw_orientation = ORIENTATION_UNKNOWN;
    }

    TickType_t now = xTaskGetTickCount();

    // 如果检测到的状态与上次不同，重置稳定计时器
    if (_g_orientation != raw_orientation) {
        _g_orientation = raw_orientation;
        _g_orientation_info.stable_start_tick = now;
        // 不立即改变 current_stable_orientation，除非新状态稳定足够长时间
    } else {
        // 同样的检测结果持续出现，检查是否达到稳定时长
        if (raw_orientation == ORIENTATION_UPRIGHT_Y || raw_orientation == ORIENTATION_SIDELAY_X) {
            if ((now - _g_orientation_info.stable_start_tick) >= _g_orientation_info.required_ticks) {
                // 只有在稳定时间满足后，才将当前稳定姿态设置为检测到的姿态
                _g_orientation_info.last_stable_orientation = _g_orientation_info.current_stable_orientation;
                _g_orientation_info.current_stable_orientation = raw_orientation;
            }
        } else {
            // raw_orientation 为 UNKNOWN 时，不清除 current_stable_orientation，保持先前已确认的状态
            // 这样可以满足 "只要之后没有检测侧放，就一直是竖立" 的需求
        }
    }

    // 将全局 _g_orientation 与当前稳定姿态保持一致（方便其他逻辑使用）
    // _g_orientation = _g_orientation_info.current_stable_orientation;

    // 返回原始检测值（或更有用的：当前稳定姿态）。为了上层使用稳定结果，返回当前稳定姿态。
    // return _g_orientation;
}



static void play_task(void *arg)
{
    while(true) {
        EventBits_t bits = xEventGroupWaitBits(_g_audio_event_group, 
                            AUDIO_EVENT_PLAYING | AUDIO_EVENT_PLAYING_STOP,
                            pdFALSE,
                            pdFALSE,
                            portMAX_DELAY);

        if(bits & AUDIO_EVENT_PLAYING) {
            xEventGroupClearBits(_g_audio_event_group, AUDIO_EVENT_PLAYING);
            bsp_extra_player_play_file("/sdcard/record.wav");
            ESP_LOGI(TAG, "Playing recorded audio...");
        }

        if(bits & AUDIO_EVENT_PLAYING_STOP) {
            xEventGroupClearBits(_g_audio_event_group, AUDIO_EVENT_PLAYING_STOP);
            if(bsp_extra_player_get_state() == AUDIO_PLAYER_STATE_PLAYING) {
                ESP_LOGI(TAG, "Stopping playing recorded audio...");
                bsp_extra_player_stop();
            }
        }

    }
}

static void sd_card_writer_task(void *arg) 
{  
    ESP_LOGI(TAG, "SD Card Writer started.");

    size_t bytes_written;
    size_t total_bytes_written = 0;

    while (true) {
        xEventGroupWaitBits(_g_audio_event_group, AUDIO_EVENT_SAVING, pdTRUE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(TAG, "Saving audio to SD card...");

        FILE *f = fopen("/sdcard/record.wav", "wb");
        if (f == NULL) {
            ESP_LOGE(TAG, "Failed to open record.wav for writing!");
            vTaskDelete(NULL);
        }

        wav_header_t wav_header;
        fwrite(&wav_header, sizeof(wav_header), 1, f);

        while(true) {
            bsp_audio_queue_node_t *block;
            if (xQueueReceive(_g_audio_handle->_audio_queue, &block, pdMS_TO_TICKS(500)) == pdPASS) {
                bytes_written = fwrite(block->data, 1, block->size, f);
                if(bytes_written) total_bytes_written += bytes_written;
                // ESP_LOGI(TAG, "read %d node busy= %d", block->size, block->is_busy);
                block->is_busy = false;
            }
            if (_g_orientation_info.current_stable_orientation != ORIENTATION_UPRIGHT_Y && uxQueueMessagesWaiting(_g_audio_handle->_audio_queue) == 0) {
                // Update WAV header with correct sizes
                wav_header = (wav_header_t)WAV_HEADER_PCM_DEFAULT(total_bytes_written, AUDIO_BITS_PER_SAMPLE, AUDIO_SAMPLE_RATE, AUDIO_SLOT_MODE);
                if(f) fseek(f, 0, SEEK_SET), fwrite(&wav_header, sizeof(wav_header), 1, f), fclose(f);
                ESP_LOGI(TAG, "Finished saving audio to SD card, total bytes=%d", total_bytes_written);
                total_bytes_written = 0;
                // bsp_extra_player_play_file("/sdcard/record.wav");
                xEventGroupSetBits(_g_audio_event_group, AUDIO_EVENT_PLAYING);
                break; 
            }
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        xEventGroupClearBits(_g_audio_event_group, AUDIO_EVENT_SAVING);
        f = NULL;
        ESP_LOGI(TAG, "SD Card Writer stopped.");
    }
    vTaskDelete(NULL);
}



static void i2s_recorder_task(void *arg) 
{
    uint8_t buffer[AUDIO_QUEUE_NODE_SIZE];
    while(true) {
        xEventGroupWaitBits(_g_audio_event_group, AUDIO_EVENT_RECORDING, pdTRUE, pdFALSE, portMAX_DELAY);

        ESP_LOGI(TAG, "I2S Recorder started.");

        while(_g_orientation_info.current_stable_orientation == ORIENTATION_UPRIGHT_Y) {
            size_t bytes_read;
            esp_err_t ret = _g_audio_handle->read(buffer, AUDIO_QUEUE_NODE_SIZE, &bytes_read, pdMS_TO_TICKS(500));
            if (ret == ESP_OK) {
                _g_audio_handle->add_queue_node(buffer, bytes_read);
            } else {
                ESP_LOGE(TAG, "I2S read error: %s", esp_err_to_name(ret));
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        xEventGroupClearBits(_g_audio_event_group, AUDIO_EVENT_RECORDING);
        ESP_LOGI(TAG, "I2S Recorder stopped.");
    }
    vTaskDelete(NULL);
}


static void mic_speak_task(void *arg)
{
    marketing_ctx_t *ctx = (marketing_ctx_t *)arg;
    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

     _g_audio_handle->mute(AUDIO_PLAYER_MUTE);

    while (true) {
        float ax, ay, az;
        if(_g_imu_handle->read_accel(&ax, &ay, &az) == ESP_OK) {
            _detect_orientation(ax, ay, az);
            /* _detect_orientation 已会在内部更新 _g_orientation_info.current_stable_orientation
               并同步到全局 _g_orientation。如果需要，这里也可以直接使用 temp。 */
            ESP_LOGI(TAG, "cur Orientation: %d, cur stabel state = %d", _g_orientation, _g_orientation_info.current_stable_orientation);
        }

        switch (_g_orientation_info.current_stable_orientation) {
            case ORIENTATION_SIDELAY_X:
                break;
            case ORIENTATION_UPRIGHT_Y:
                xEventGroupSetBits(_g_audio_event_group, AUDIO_EVENT_RECORDING | AUDIO_EVENT_SAVING | AUDIO_EVENT_PLAYING_STOP);
                break;
            default:
                break;
        }
        lv_update_state();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Should not reach here
    vTaskDelete(NULL);
    return;
}

void marketing_task(void *arg)
{
    state_label = lv_label_create(lv_scr_act());
    lv_obj_set_align(state_label, LV_ALIGN_CENTER);
    lv_obj_set_width(state_label, LV_HOR_RES - 40);
    lv_obj_set_style_text_align(state_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(state_label, &lv_font_montserrat_24, 0);
    // lv_label_set_long_mode(state_label, LV_LABEL_LONG_SCROLL);
    lv_label_set_text(state_label, "Idle...");

    // Allocate context on heap because task will run after this function returns
    marketing_ctx_t *_task_ctx = pvPortMalloc(sizeof(marketing_ctx_t));
    if (_task_ctx == NULL) {
        ESP_LOGE(TAG, "failed to alloc marketing ctx");
        return;
    }
    _task_ctx->task = NULL;
    _task_ctx->is_completed = false;
    _task_ctx->timeout_ticks = 10000; // 10 seconds per image by default

    TickType_t deadline;
    BaseType_t r;

    _g_audio_event_group = xEventGroupCreate();
    if (_g_audio_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create audio event group");
        vTaskDelete(NULL);
        return;
    }

    xEventGroupClearBits(_g_audio_event_group, AUDIO_EVENT_RECORDING | AUDIO_EVENT_PLAYING | \
        AUDIO_EVENT_SAVING | AUDIO_EVENT_PLAYING_STOP);


    ESP_ERROR_CHECK(bsp_sdcard_init());
    bsp_extra_qmi8658_init(&_g_imu_handle);
    bsp_extra_audio_init(&_g_audio_handle);
    bsp_extra_player_init();

    _g_orientation_info.current_stable_orientation = ORIENTATION_UNKNOWN;
    _g_orientation_info.last_stable_orientation = ORIENTATION_UNKNOWN;
    _g_orientation_info.stable_start_tick = 0;
    _g_orientation_info.required_ticks = pdMS_TO_TICKS(QMI8658_ORIENTATION_DURATION_MS);

    xTaskCreate(i2s_recorder_task, "i2s_task", 1024 * 8, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(sd_card_writer_task, "sd_task", 1024 * 6, NULL, tskIDLE_PRIORITY + 3, NULL);
    xTaskCreate(play_task, "play_task", 1024 * 4, NULL, tskIDLE_PRIORITY + 1, NULL);
    r = xTaskCreate(mic_speak_task, "mic_speak_task", 1024 * 14, _task_ctx, tskIDLE_PRIORITY + 3, &_task_ctx->task);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create mic_speak_task");
        goto _exit;
    }
    ESP_LOGI(TAG, "marketing task completed");

_exit:

    // vPortFree(_task_ctx);
    vTaskDelete(NULL);
}

#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

