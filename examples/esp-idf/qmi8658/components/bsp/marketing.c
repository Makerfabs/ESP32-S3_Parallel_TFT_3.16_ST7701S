// Marketing demo: simple slideshow that shows assets images one per second with basic fade-in

#include "bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>

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

static const char *TAG = "marketing";

typedef struct {
	TaskHandle_t task;
	volatile bool is_completed;
	uint32_t timeout_ticks; // interval between images
} marketing_ctx_t;


static void display_qmi_task(void *arg)
{
    marketing_ctx_t *ctx = (marketing_ctx_t *)arg;
    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    const lv_image_dsc_t *imgs[] = { &img1, &img2, &img3, &img4, &img5 };
    const size_t img_cnt = sizeof(imgs) / sizeof(imgs[0]);
    size_t idx = 0;

    // Create image object on the screen
    bsp_display_lock(0);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    // lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    lv_obj_t *img_obj = lv_image_create(scr);
    // lv_obj_center(img_obj);
    bsp_display_unlock();

    /* Flip-to-change logic inside the user-specified time window:
     * We respect the existing while(xTaskGetTickCount() - start_time < ctx->timeout_ticks)
     * as the allowed detection period. Within this window we poll the IMU (if available)
     * and switch images when a flip is detected (az < flip_threshold). If IMU is not
     * available we fallback to advancing images every second until timeout.
     */
    qmi8658_dev_t imu_dev;
    qmi8658_data_t data;
    bool have_imu = false;
    size_t cur_idx = 0;

    /* attempt to init I2C and QMI8658; if any step fails, fall back to timed slideshow */
    esp_err_t err = bsp_i2c_init();
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_handle_get();
    if (err == ESP_OK && i2c_handle != NULL) {
        err = qmi8658_init(&imu_dev, i2c_handle, QMI8658_ADDRESS_HIGH);
        if (err == ESP_OK) {
            have_imu = true;
            qmi8658_set_accel_range(&imu_dev, QMI8658_ACCEL_RANGE_8G);
            qmi8658_set_accel_odr(&imu_dev, QMI8658_ACCEL_ODR_1000HZ);
            qmi8658_set_gyro_range(&imu_dev, QMI8658_GYRO_RANGE_512DPS);
            qmi8658_set_gyro_odr(&imu_dev, QMI8658_GYRO_ODR_1000HZ);
            qmi8658_set_accel_unit_mps2(&imu_dev, true);
            qmi8658_set_gyro_unit_rads(&imu_dev, true);
            /* enable accel only to save power/bus time */
            // qmi8658_enable_sensors(&imu_dev, QMI8658_ENABLE_ACCEL);
            /* small settle delay */
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            ESP_LOGW(TAG, "qmi8658_init failed: %d, falling back to timed slideshow", err);
            have_imu = false;
        }
    } else {
        ESP_LOGW(TAG, "bsp_i2c unavailable or init failed, falling back to timed slideshow");
        have_imu = false;
    }

    const TickType_t min_switch_ms = 500; /* minimal interval between switches */
    const float flip_threshold = -6.0f; /* accel Z threshold (m/s^2) to consider flipped */
    TickType_t last_switch = xTaskGetTickCount() - pdMS_TO_TICKS(min_switch_ms);

    TickType_t start_time = xTaskGetTickCount();
    /* show initial image once before entering detection window */
    bsp_display_lock(0);
    lv_image_set_src(img_obj, imgs[cur_idx]);
    bsp_display_unlock();

    /* edge-detection helpers */
    float prev_az = 0.0f;
    const float gyro_motion_threshold = 0.5f; /* rad/s summed magnitude threshold */
    if (have_imu) {
        /* read initial Z to seed prev_az */
        if (qmi8658_read_accel(&imu_dev, &data.accelX, &data.accelY, &data.accelZ) == ESP_OK) {
            prev_az = data.accelZ;
        }
    }

    while (xTaskGetTickCount() - start_time < ctx->timeout_ticks) {
        if (have_imu) {
            bool ready = false;
            if (qmi8658_is_data_ready(&imu_dev, &ready) == ESP_OK && ready) {
                float ax=0, ay=0, az=0;
                float gx=0, gy=0, gz=0;
                if (qmi8658_read_accel(&imu_dev, &ax, &ay, &az) == ESP_OK) {
                    /* read gyro; ignore error if not available */
                    (void)qmi8658_read_gyro(&imu_dev, &gx, &gy, &gz);

                    float gyro_motion = fabsf(gx) + fabsf(gy) + fabsf(gz);
                    ESP_LOGD(TAG, "accel: %.2f %.2f %.2f  gyro: %.2f", ax, ay, az, gyro_motion);

                    /* require an edge crossing (prev_az >= threshold -> az < threshold)
                     * and some angular motion to avoid switching when stationary and already upside-down.
                     */
                    if (prev_az >= flip_threshold && az < flip_threshold && gyro_motion >= gyro_motion_threshold &&
                        (xTaskGetTickCount() - last_switch) >= pdMS_TO_TICKS(min_switch_ms)) {
                        cur_idx = (cur_idx + 1) % img_cnt;
                        bsp_display_lock(0);
                        lv_image_set_src(img_obj, imgs[cur_idx]);
                        bsp_display_unlock();
                        last_switch = xTaskGetTickCount();
                    }

                    prev_az = az;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            /* No IMU: do not auto-advance (must have flip to change) */
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }



    // cleanup
    bsp_display_lock(0);
    if (img_obj && lv_obj_is_valid(img_obj)) lv_obj_del(img_obj);
    lv_obj_clean(scr);
    bsp_display_unlock();

    ctx->is_completed = true;

    // free ctx allocated by starter
    // vPortFree(ctx);
    vTaskDelete(NULL);
}


static void weather_task(void *arg)
{
    marketing_ctx_t *ctx = (marketing_ctx_t *)arg;
    if (ctx == NULL) {
        vTaskDelete(NULL);
        return;
    }

    /* Prepare screen: single label, no styles, no extra layout */
    bsp_display_lock(0);
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);

    /* Main content label (other weather info) */
    lv_obj_t *content = lv_label_create(scr);
    lv_obj_set_width(content, lv_disp_get_hor_res(NULL) - 20);
    lv_label_set_long_mode(content, LV_LABEL_LONG_WRAP);
    lv_obj_align(content, LV_ALIGN_CENTER, 0, -12);

    /* Update time label placed below main content */
    lv_obj_t *time_label = lv_label_create(scr);
    lv_label_set_long_mode(time_label, LV_LABEL_LONG_WRAP);
    lv_obj_align(time_label, LV_ALIGN_CENTER, 0, 48);

    /* Try to set a 24px font for main content. If the project has a specific
     * font declaration such as lv_font_montserrat_24, use it. Otherwise fall
     * back to default font. We guard with a weak reference macro if not
     * available; using LVGL API to set style font directly.
     */
#if defined(LV_FONT_MONTSERRAT_24) || defined(lv_font_montserrat_24)
    lv_obj_set_style_text_font(content, &lv_font_montserrat_24, 0);
#else
    /* fallback: use default font but set a larger line spacing via style if desired */
    lv_obj_set_style_text_font(content, lv_scr_act() ? lv_obj_get_style_text_font(lv_scr_act(), 0) : LV_FONT_DEFAULT, 0);
#endif

    /* Keep time_label using a smaller/default font */
    lv_obj_set_style_text_font(time_label, lv_obj_get_style_text_font(lv_scr_act(), 0), 0);

    bsp_display_unlock();

    /* Build weather request */
    weather_config_t cfg = {0};
    cfg.type = WEATHER_HEFENG;
    cfg.api_key = "737ce9c987b34b508166092085bac65b";
    cfg.api_host = "kw487jn3du.re.qweatherapi.com";
    cfg.city = NULL;

    TickType_t start_time = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < ctx->timeout_ticks) {
        weather_info_t *info = weather_get(&cfg);
        if (!info) {
            bsp_display_lock(0);
            lv_label_set_text(content, "Failed to get weather. Check API key or network.");
            bsp_display_unlock();
        } else {
            char buf[512];
            const char *icon = "Cloudy";
            if (info->weather) {
                if (strstr(info->weather, "晴") || strstr(info->weather, "Clear")) icon = "Sunny";
                else if (strstr(info->weather, "雨") || strstr(info->weather, "Rain")) icon = "Rain";
                else if (strstr(info->weather, "雪") || strstr(info->weather, "Snow")) icon = "Snow";
                else if (strstr(info->weather, "雷") || strstr(info->weather, "Thunder")) icon = "Thunder";
                else if (strstr(info->weather, "雾") || strstr(info->weather, "Fog")) icon = "Fog";
            }

            /* Put most info in the main content label and the update_time on its own line/time_label */
            snprintf(buf, sizeof(buf), "%s  %.0f°C\nFeels like: %.1f°C\nHumidity: %.0f%%",
                     icon,
                     info->temperature,
                     info->feels_like,
                     info->humidity);

            bsp_display_lock(0);
            lv_label_set_text(content, buf);
            lv_label_set_text(time_label, info->update_time ? info->update_time : "-");
            bsp_display_unlock();

            weather_info_free(info);
        }

        vTaskDelay(pdMS_TO_TICKS(4000));
    }

    ctx->is_completed = true;
    vTaskDelete(NULL);
}

static void mic_speak_task(void *arg)
{
    marketing_ctx_t *ctx = (marketing_ctx_t *)arg;
    if(ctx == NULL) {
        ESP_LOGW(TAG, "mic_spe_task ctx is NULL");
        vTaskDelete(NULL);
        return;
    }
    /* We'll use the project's audio helper (audio_init + bsp_audio_handle_get)
     * to access I2S handles for recording/playback. Record 10 seconds of
     * 16-bit mono @16kHz PCM into a malloc'd buffer, then poll IMU every
     * 200ms and play back when a flip is detected. The monitoring window is
     * limited by ctx->timeout_ticks.
     */
    const int sample_rate = 16000;
    const int bits_per_sample = 16;
    const int channels = 1;
    const int record_seconds = 10; /* record duration */
    const size_t bytes_per_sample = (bits_per_sample + 7) / 8;
    const size_t record_bytes = (size_t)sample_rate * bytes_per_sample * channels * (size_t)record_seconds;

    /* initialize local I2S channels (do not call audio_init per user's request) */
    i2s_chan_handle_t tx_handle = NULL;
    i2s_chan_handle_t rx_handle = NULL;
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = AUDIO_I2S_GPIO_BCLK,
            .ws = AUDIO_I2S_GPIO_WS,
            .dout = AUDIO_I2S_GPIO_DOUT,
            .din = AUDIO_I2S_GPIO_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false
            }
        }
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;

    ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
    ret = i2s_channel_init_std_mode(rx_handle, &std_cfg);

    if (tx_handle) ret = i2s_channel_enable(tx_handle);
    if (rx_handle) ret = i2s_channel_enable(rx_handle);
    
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "i2s channel init failed");
        vTaskDelete(NULL);
        return;
    }

    uint8_t *rec_buf = (uint8_t *)heap_caps_malloc(record_bytes, MALLOC_CAP_SPIRAM);
    if (rec_buf == NULL) {
        ESP_LOGW(TAG, "PSRAM alloc failed, falling back to malloc for %u bytes", (unsigned)record_bytes);
        rec_buf = (uint8_t *)malloc(record_bytes);
    }
    if (rec_buf == NULL) {
        ESP_LOGE(TAG, "failed to alloc record buffer (%u bytes)", (unsigned)record_bytes);
        vTaskDelete(NULL);
        return;
    }
    memset(rec_buf, 0, record_bytes);

    ESP_LOGI(TAG, "Start recording %d seconds (%u bytes)", record_seconds, (unsigned)record_bytes);

    const size_t CHUNK = 4 * 1024;
    size_t offset = 0;
    while (offset < record_bytes) {
        size_t to_read = CHUNK;
        if (record_bytes - offset < to_read) to_read = record_bytes - offset;
        size_t bytes_read = 0;
        ret = i2s_channel_read(rx_handle, rec_buf + offset, to_read, &bytes_read, portMAX_DELAY);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "i2s_channel_read error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (bytes_read == 0) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        offset += bytes_read;
    }

    ESP_LOGI(TAG, "Recording complete, bytes=%u", (unsigned)offset);

    /* Prepare IMU for flip detection (reuse logic from display_qmi_task) */
    esp_err_t ierr = bsp_i2c_init();
    i2c_master_bus_handle_t i2c_handle = bsp_i2c_handle_get();
    qmi8658_dev_t imu_dev;
    qmi8658_data_t data;
    bool have_imu = false;
    if (ierr == ESP_OK && i2c_handle != NULL) {
        if (qmi8658_init(&imu_dev, i2c_handle, QMI8658_ADDRESS_HIGH) == ESP_OK) {
            have_imu = true;
            qmi8658_set_accel_range(&imu_dev, QMI8658_ACCEL_RANGE_8G);
            qmi8658_set_accel_odr(&imu_dev, QMI8658_ACCEL_ODR_1000HZ);
            qmi8658_set_gyro_range(&imu_dev, QMI8658_GYRO_RANGE_512DPS);
            qmi8658_set_gyro_odr(&imu_dev, QMI8658_GYRO_ODR_1000HZ);
            qmi8658_set_accel_unit_mps2(&imu_dev, true);
            qmi8658_set_gyro_unit_rads(&imu_dev, true);
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            ESP_LOGW(TAG, "qmi8658_init failed, continuing without IMU");
            have_imu = false;
        }
    } else {
        ESP_LOGW(TAG, "bsp_i2c unavailable, continuing without IMU");
        have_imu = false;
    }

    float prev_az = 0.0f;
    const float flip_threshold = -6.0f;
    const float gyro_motion_threshold = 0.5f;
    TickType_t last_switch = xTaskGetTickCount();
    const TickType_t min_switch_ms = pdMS_TO_TICKS(500);
    if (have_imu) {
        if (qmi8658_read_accel(&imu_dev, &data.accelX, &data.accelY, &data.accelZ) == ESP_OK) {
            prev_az = data.accelZ;
        }
    }

    /* Monitoring loop: every 200ms check for flip; timeout controlled by ctx->timeout_ticks */
    TickType_t start_time = xTaskGetTickCount();
    while (xTaskGetTickCount() - start_time < ctx->timeout_ticks) {
        if (have_imu) {
            bool ready = false;
            if (qmi8658_is_data_ready(&imu_dev, &ready) == ESP_OK && ready) {
                float ax=0, ay=0, az=0;
                float gx=0, gy=0, gz=0;
                if (qmi8658_read_accel(&imu_dev, &ax, &ay, &az) == ESP_OK) {
                    (void)qmi8658_read_gyro(&imu_dev, &gx, &gy, &gz);
                    float gyro_motion = fabsf(gx) + fabsf(gy) + fabsf(gz);
                    ESP_LOGD(TAG, "mic accel: %.2f %.2f %.2f  gyro: %.2f", ax, ay, az, gyro_motion);

                    if (prev_az >= flip_threshold && az < flip_threshold && gyro_motion >= gyro_motion_threshold &&
                        (xTaskGetTickCount() - last_switch) >= min_switch_ms) {
                        ESP_LOGI(TAG, "Flip detected - playing recorded audio");

                        /* Play back recorded buffer in chunks */
                        size_t play_offset = 0;
                        while (play_offset < offset) {
                            size_t to_write = CHUNK;
                            if (offset - play_offset < to_write) to_write = offset - play_offset;
                            size_t bytes_written = 0;
                            esp_err_t wret = i2s_channel_write(tx_handle, rec_buf + play_offset, to_write, &bytes_written, pdMS_TO_TICKS(1000));
                            if (wret != ESP_OK) {
                                ESP_LOGW(TAG, "i2s_channel_write failed: %s", esp_err_to_name(wret));
                                break;
                            }
                            if (bytes_written == 0) {
                                vTaskDelay(pdMS_TO_TICKS(10));
                                continue;
                            }
                            play_offset += bytes_written;
                        }

                        last_switch = xTaskGetTickCount();
                    }

                    prev_az = az;
                }
            }
        } else {
            /* No IMU: nothing to check, just sleep */
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ESP_LOGI(TAG, "mic_speak_task monitoring finished");
}


void marketing_task(void *arg)
{
    // Allocate context on heap because task will run after this function returns
    marketing_ctx_t *_task_ctx = pvPortMalloc(sizeof(marketing_ctx_t));
    if (_task_ctx == NULL) {
        ESP_LOGE(TAG, "failed to alloc marketing ctx");
        return;
    }
    _task_ctx->task = NULL;
    _task_ctx->is_completed = false;
    _task_ctx->timeout_ticks = 10000; // 6 seconds per image by default

    TickType_t deadline;
    BaseType_t r;

    r = xTaskCreate(display_qmi_task, "display_qmi_task", 1024 * 5, _task_ctx, tskIDLE_PRIORITY + 1, &_task_ctx->task);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create display_qmi_task");
        vPortFree(_task_ctx);
        goto _exit;
    }
    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(_task_ctx->timeout_ticks);
    while(!_task_ctx->is_completed && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if(!_task_ctx->is_completed && _task_ctx->task != NULL) {
        vTaskDelete(_task_ctx->task);
        _task_ctx->task = NULL;
    }
#if 0
    vTaskDelay(pdMS_TO_TICKS(500));

    /** Display weather task */
    _task_ctx->task = NULL;
    _task_ctx->is_completed = false;
    _task_ctx->timeout_ticks = 10000; // 15 seconds for weather display

    r = xTaskCreate(weather_task, "weather_task", 1024 * 6, _task_ctx, tskIDLE_PRIORITY + 1, &_task_ctx->task);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create weather_task");
        goto _exit;
    }
    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(_task_ctx->timeout_ticks);
    while(!_task_ctx->is_completed && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    _task_ctx->is_completed = true;
    if(!_task_ctx->is_completed && _task_ctx->task != NULL) {
        vTaskDelete(_task_ctx->task);
        _task_ctx->task = NULL;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    _task_ctx->task = NULL;
    _task_ctx->is_completed = false;
    _task_ctx->timeout_ticks = 20000;

    r = xTaskCreate(mic_speak_task, "mic_speak_task", 1024 * 10, _task_ctx, tskIDLE_PRIORITY + 1, &_task_ctx->task);
    if (r != pdPASS) {
        ESP_LOGE(TAG, "failed to create mic_speak_task");
        goto _exit;
    }
    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(_task_ctx->timeout_ticks);
    while(!_task_ctx->is_completed && xTaskGetTickCount() < deadline) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    if(!_task_ctx->is_completed && _task_ctx->task != NULL) {
        vTaskDelete(_task_ctx->task);
        _task_ctx->task = NULL;
    }
#endif
    ESP_LOGI(TAG, "marketing task completed");

_exit:

    vPortFree(_task_ctx);
    vTaskDelete(NULL);
}




#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

