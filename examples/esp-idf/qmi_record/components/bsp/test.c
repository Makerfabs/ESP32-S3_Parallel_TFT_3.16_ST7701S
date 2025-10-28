// #include "bsp_board.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "pcf85063a.h"

// #if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
// #include "lvgl.h"

// static const char *TAG = "test_display";

// static pcf85063a_datetime_t Set_Time = {
//     .year = 2025,
//     .month = 07,
//     .day = 30,
//     .dotw = 3,   // Day of the week: 0 = Sunday
//     .hour = 9,
//     .min = 0,
//     .sec = 0
// };

// static void clear_screen_and_show_task_info(const char *task_name) 
// {
//     bsp_display_lock(0);
    
//     lv_obj_t *scr = lv_scr_act();
//     lv_obj_clean(scr);
//     lv_obj_set_style_bg_color(scr, lv_color_make(255, 255, 255), 0);
    
//     lv_obj_t *label = lv_label_create(scr);
//     lv_obj_set_width(label, lv_disp_get_hor_res(NULL) - 40);
//     lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
//     lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
//     lv_obj_set_style_text_color(label, lv_color_make(0, 0, 0), 0);
//     lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
//     lv_label_set_text_fmt(label, "current task:\n%s", task_name);
    
//     bsp_display_unlock();
// }

// #endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

// // 任务控制结构
// typedef struct {
//     TaskHandle_t task_handle;
//     bool task_completed;
//     TickType_t timeout_ticks;
// } task_control_t;

// static void display_color_test_task_wrapper(void *arg)
// {
//     task_control_t *ctrl = (task_control_t *)arg;
//     TickType_t start_time = xTaskGetTickCount();

//     struct {
//         lv_color_t color;
//         const char *name;
//     } test_colors[] = {
//         {lv_color_make(255, 0, 0),     "红色 (Red)"},
//         {lv_color_make(0, 255, 0),     "绿色 (Green)"},
//         {lv_color_make(0, 0, 255),     "蓝色 (Blue)"},
//         {lv_color_make(255, 255, 0),   "黄色 (Yellow)"},
//         {lv_color_make(255, 0, 255),   "洋红色 (Magenta)"},
//         {lv_color_make(0, 255, 255),   "青色 (Cyan)"},
//         {lv_color_make(255, 255, 255), "白色 (White)"},
//         {lv_color_make(0, 0, 0),       "黑色 (Black)"},
//     };

//     int num_colors = sizeof(test_colors) / sizeof(test_colors[0]);

//     for (int i = 0; i < num_colors; i++) {
//         if ((xTaskGetTickCount() - start_time) >= ctrl->timeout_ticks) {
//             ESP_LOGW(TAG, "屏幕颜色测试超时!");
//             break;
//         }

//         bsp_display_lock(0);

//         lv_obj_t *scr = lv_scr_act();

//         lv_obj_set_style_bg_color(scr, test_colors[i].color, 0);

//         bsp_display_unlock();

//         ESP_LOGI(TAG, "显示颜色: %s", test_colors[i].name);

//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }

//     ESP_LOGI(TAG, "屏幕颜色显示测试完成!");
    
//     ctrl->task_completed = true;
//     vTaskDelete(NULL);
// }

// void mic_speaker_test_task(void *arg)
// {
//     task_control_t *ctrl = (task_control_t *)arg;
    
//     esp_err_t ret = audio_init(16000, 16000, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
//     if(ret != ESP_OK) {
//         ESP_LOGE(TAG, "音频初始化失败!");
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }

//     audio_handle_t audio_handle = bsp_audio_handle_get();

//     const size_t bytes_per_sample = 2; // 16-bit slot
//     const size_t buf_samples = 1024; // more frames for 16-bit
//     const size_t buf_bytes = buf_samples * bytes_per_sample;

//     uint8_t *read_buf = (uint8_t *)malloc(buf_bytes);
//     uint8_t *write_buf = (uint8_t *)malloc(buf_bytes);

//     if (!read_buf || !write_buf) {
//         ESP_LOGE(TAG, "Out of memory for buffers");
//         free(read_buf);
//         free(write_buf);
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }

//     // 清屏并显示当前为mic_speaker task
//     clear_screen_and_show_task_info("mic speaker test");

//     // 记录任务开始时间
//     TickType_t start_time = xTaskGetTickCount();

//     while ((xTaskGetTickCount() - start_time) < ctrl->timeout_ticks) {
//         size_t bytes_read = 0;
//         esp_err_t err = i2s_channel_read(audio_handle->_rx_handle, read_buf, buf_bytes, &bytes_read, 1000);
//         if (err == ESP_ERR_TIMEOUT) {
//             // no data within timeout
//             continue;
//         } else if (err != ESP_OK) {
//             ESP_LOGW(TAG, "i2s_channel_read error: %d", err);
//             vTaskDelay(pdMS_TO_TICKS(10));
//             continue;
//         }

//         if (bytes_read == 0) continue;

//         // For 16-bit mode we directly passthrough the bytes read
//         size_t bytes_to_write = bytes_read;
//         size_t bytes_written = 0;
//         err = i2s_channel_write(audio_handle->_tx_handle, read_buf, bytes_to_write, &bytes_written, 1000);
//         if (err != ESP_OK) {
//             ESP_LOGW(TAG, "i2s_channel_write error: %d", err);
//         }
//     }

//     ESP_LOGI(TAG, "麦克风扬声器测试完成");

//     free(read_buf);
//     free(write_buf);

    
//     // 等待资源完全释放
//     vTaskDelay(pdMS_TO_TICKS(200));
    
//     ctrl->task_completed = true;
//     vTaskDelete(NULL);
// }

// static void play_wav_sd_task(void *arg)
// {
//     task_control_t *ctrl = (task_control_t *)arg;

//     sd_init("/sdcard");
//     sd_handle_t sd_handle = sd_handle_get();
//     if(sd_handle == NULL) {
//         ESP_LOGE(TAG, "SD卡初始化失败!");
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }
//     sd_handle->print_sd_files();

//     // 添加延迟让音频系统稳定
//     vTaskDelay(pdMS_TO_TICKS(100));

//     audio_handle_t audio_handle = bsp_audio_handle_get();
//     if(audio_handle == NULL) {
//         ESP_LOGE(TAG, "音频句柄获取失败!");
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }

//     // 清屏并显示当前为play_wav_sd task
//     clear_screen_and_show_task_info("play wav from sd");

//     ESP_LOGI(TAG, "开始播放WAV文件...");

//     // 播放WAV文件
//     extern esp_err_t wav_play(audio_handle_t handle, const char *filepath);
//     wav_play(audio_handle, "/sdcard/cn.wav");

//     ctrl->task_completed = true;

//     vTaskDelete(NULL);
    
// }

// void imu_pcf_test_task(void *arg)
// {
//     task_control_t *ctrl = (task_control_t *)arg;

//     // clear_screen_and_show_task_info("imu pcf test");

//     bsp_display_lock(0);
//     lv_obj_t *scr = lv_scr_act();
//     lv_obj_clean(scr);

//     lv_obj_set_style_bg_color(scr, lv_color_make(255, 255, 255), 0);

//     lv_obj_t *pcf_data_label = lv_label_create(scr);
//     lv_label_set_text(pcf_data_label, "Initializing PCF data...");
//     lv_obj_set_width(pcf_data_label, 300);
//     lv_label_set_long_mode(pcf_data_label, LV_LABEL_LONG_WRAP);
//     lv_obj_align(pcf_data_label, LV_ALIGN_TOP_LEFT, 10, 40);

//     lv_obj_t *imu_data_label = lv_label_create(scr);
//     lv_label_set_text(imu_data_label, "Initializing sensor data...");
//     lv_obj_set_width(imu_data_label, 300);
//     lv_label_set_long_mode(imu_data_label, LV_LABEL_LONG_WRAP);
//     lv_obj_align(imu_data_label, LV_ALIGN_TOP_LEFT, 10, 300);

//     bsp_display_unlock();

//     TickType_t start_time = xTaskGetTickCount();

//     esp_err_t err = bsp_i2c_init();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "bsp_i2c_init failed: %s", esp_err_to_name(err));
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }

//     i2c_master_bus_handle_t i2c_handle = bsp_i2c_handle_get();
//     if (i2c_handle == NULL) {
//         ESP_LOGE(TAG, "bsp_i2c_handle_get returned NULL");
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }

//     qmi8658_dev_t imu_dev;
//     qmi8658_data_t data;
//     pcf85063a_dev_t pcf_dev;
//     pcf85063a_datetime_t Now_time;

//     esp_err_t ret = qmi8658_init(&imu_dev, i2c_handle, QMI8658_ADDRESS_HIGH);
//     if (ret != ESP_OK) {
//         ESP_LOGE("IMU", "Failed to initialize QMI8658 (error: %d)", ret);
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//     }
//     qmi8658_set_accel_range(&imu_dev, QMI8658_ACCEL_RANGE_8G);
//     qmi8658_set_accel_odr(&imu_dev, QMI8658_ACCEL_ODR_1000HZ);
//     qmi8658_set_gyro_range(&imu_dev, QMI8658_GYRO_RANGE_512DPS);
//     qmi8658_set_gyro_odr(&imu_dev, QMI8658_GYRO_ODR_1000HZ);

//     qmi8658_set_accel_unit_mps2(&imu_dev, true);
//     qmi8658_set_gyro_unit_rads(&imu_dev, true);

//     /* initialize PCF device */
//     ret = pcf85063a_init(&pcf_dev, i2c_handle, PCF85063A_ADDRESS);
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "pcf85063a_init failed: %s", esp_err_to_name(ret));
//         ctrl->task_completed = true;
//         vTaskDelete(NULL);
//         return;
//     }
//     pcf85063a_set_time_date(&pcf_dev, Set_Time);
//     // pcf85063a_set_alarm(&pcf_dev, Set_Alarm_Time);
//     // pcf85063a_enable_alarm(&pcf_dev);


//     char display_text[400];
//     char datetime_str[256];  // Buffer to store formatted date-time string
//     char time_display_str[512]; // Buffer for screen display
//     while((xTaskGetTickCount() - start_time) < ctrl->timeout_ticks) {
//         bool ready;
//         esp_err_t ret = qmi8658_is_data_ready(&imu_dev, &ready);
//         if (ret == ESP_OK && ready) {
//             ret = qmi8658_read_sensor_data(&imu_dev, &data);
//             if (ret == ESP_OK) {
//                 bsp_display_lock(0);
                
//                 snprintf(display_text, sizeof(display_text),
//                     "Accel: X=%+.4f m/s^2\n"
//                     "       Y=%+.4f m/s^2\n"
//                     "       Z=%+.4f m/s^2\n"
//                     "Gyro:  X=%+.4f rad/s\n"
//                     "       Y=%+.4f rad/s\n"
//                     "       Z=%+.4f rad/s\n"
//                     "Temp:  %.2f degC\n"
//                     "Timestamp: %lu",
//                     data.accelX, data.accelY, data.accelZ,
//                     data.gyroX, data.gyroY, data.gyroZ,
//                     data.temperature, data.timestamp);
                
//                 lv_label_set_text(imu_data_label, display_text);
//                 bsp_display_unlock();
//             } else {
//                 ESP_LOGE(TAG, "Failed to read sensor data (error: %d)", ret);
//             }
//         } else {
//             ESP_LOGE(TAG, "Data not ready or error reading status (error: %d)", ret);
//         }

//         pcf85063a_get_time_date(&pcf_dev, &Now_time);

//         pcf85063a_datetime_to_str(datetime_str, Now_time);
//         // ESP_LOGI("pcf85063a", "Now_time is %s", datetime_str);
//         snprintf(time_display_str, sizeof(time_display_str), 
//                 "Current Time:\n%s\n\n"
//                 "Year: %d  Month: %d  Day: %d\n"
//                 "Hour: %02d  Min: %02d  Sec: %02d\n"
//                 "Day of Week: %d",
//                 datetime_str,
//                 Now_time.year, Now_time.month, Now_time.day,
//                 Now_time.hour, Now_time.min, Now_time.sec,
//                 Now_time.dotw);
//         bsp_display_lock(0);
//         lv_label_set_text(pcf_data_label, time_display_str);
//         bsp_display_unlock();

//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
//     ctrl->task_completed = true;
//     vTaskDelete(NULL);
// }

// void test_task()
// {
//     // 测试1: 屏幕颜色
//     task_control_t display_ctrl = {
//         .task_handle = NULL,
//         .task_completed = false,
//         .timeout_ticks = pdMS_TO_TICKS(10000)  /* 10 seconds */
//     };

//     xTaskCreate(display_color_test_task_wrapper, "display_test", 4096, &display_ctrl, tskIDLE_PRIORITY + 1, &display_ctrl.task_handle);
//     TickType_t display_deadline = xTaskGetTickCount() + display_ctrl.timeout_ticks;
//     while (!display_ctrl.task_completed && xTaskGetTickCount() < display_deadline) {
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
//     if (!display_ctrl.task_completed && display_ctrl.task_handle != NULL) {
//         vTaskDelete(display_ctrl.task_handle);
//         display_ctrl.task_handle = NULL;
//     }

//     vTaskDelay(pdMS_TO_TICKS(500));

//     // 测试2: IMU和PCF85063A实时时钟
//     task_control_t imu_pcf_ctrl = {
//         .task_handle = NULL,
//         .task_completed = false,
//         .timeout_ticks = pdMS_TO_TICKS(12000)
//     };
    
//     xTaskCreate(imu_pcf_test_task, "imu_pcf_test", 1024 * 6, &imu_pcf_ctrl, tskIDLE_PRIORITY + 1, &imu_pcf_ctrl.task_handle);
//     TickType_t imu_pcf_deadline = xTaskGetTickCount() + imu_pcf_ctrl.timeout_ticks;
//     while (!imu_pcf_ctrl.task_completed && xTaskGetTickCount() < imu_pcf_deadline) {
//         vTaskDelay(pdMS_TO_TICKS(500));
//     }
//     if(!imu_pcf_ctrl.task_completed && imu_pcf_ctrl.task_handle != NULL) {
//         vTaskDelete(imu_pcf_ctrl.task_handle);
//         imu_pcf_ctrl.task_handle = NULL;
//     }

//     ESP_LOGI("TAG", "IMU和PCF测试阶段结束");
//     vTaskDelay(pdMS_TO_TICKS(500));
    
//     // 测试3: 麦克风扬声器测试
//     task_control_t mic_speaker_ctrl = {
//         .task_handle = NULL,
//         .task_completed = false,
//         .timeout_ticks = pdMS_TO_TICKS(12000)
//     };

//     BaseType_t result = xTaskCreate(mic_speaker_test_task, "mic_speaker_test", 4096, &mic_speaker_ctrl, tskIDLE_PRIORITY + 1, &mic_speaker_ctrl.task_handle);
//     if (result != pdPASS) {
//         ESP_LOGE("TAG", "Failed to create mic_speaker_test task");
//     }
//     TickType_t mic_deadline = xTaskGetTickCount() + mic_speaker_ctrl.timeout_ticks;
//     while (!mic_speaker_ctrl.task_completed && xTaskGetTickCount() < mic_deadline) {
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
//     if (!mic_speaker_ctrl.task_completed && mic_speaker_ctrl.task_handle != NULL) {
//         vTaskDelete(mic_speaker_ctrl.task_handle);
//         mic_speaker_ctrl.task_handle = NULL;
//     }
   
//     vTaskDelay(pdMS_TO_TICKS(500));
    
//     // 测试4: SD卡播放WAV文件测试
//     task_control_t play_wav_ctrl = {
//         .task_handle = NULL,
//         .task_completed = false,
//         .timeout_ticks = pdMS_TO_TICKS(21000)
//     };
    
//     xTaskCreate(play_wav_sd_task, "play_wav_sd", 1024 * 4, &play_wav_ctrl, tskIDLE_PRIORITY + 1, &play_wav_ctrl.task_handle);
//     TickType_t play_wav_deadline = xTaskGetTickCount() + play_wav_ctrl.timeout_ticks;
//     while (!play_wav_ctrl.task_completed && xTaskGetTickCount() < play_wav_deadline) {
//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
//     if (!play_wav_ctrl.task_completed && play_wav_ctrl.task_handle != NULL) {
//         vTaskDelete(play_wav_ctrl.task_handle);
//         play_wav_ctrl.task_handle = NULL;
//     }

//     clear_screen_and_show_task_info("All tests completed");
// }