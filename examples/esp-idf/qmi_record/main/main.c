#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_event.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#include "bsp_board.h"
#include "sdcard/bsp_sdcard.h"
#include "audio/bsp_audio.h"

#include "qmi8658/bsp_qmi8658.h"

#include "freertos/semphr.h"
#include "esp_log.h"

#include "protocol_examples_common.h"
// #include "lvgl.h"
#include "lv_demos.h"


static const char *TAG = "main";

void app_main(void)
{
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ESP_ERROR_CHECK(nvs_flash_init());
    // }

    // bsp_extra_weather();

    /* Start display with a slower LVGL timer to reduce LCD SPI traffic
     * timer_period_ms controls how often the LVGL internal timer runs (ms).
     * Increasing it reduces LVGL updates and display flushes, freeing SPI for SD access.
     */
    bsp_display_cfg_t disp_cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
            .sw_rotate = false,
        }
    };
    // disp_cfg.lvgl_port_cfg.task_stack = 7168 * 2;
    /* set a larger LVGL timer period (ms) to reduce refresh frequency */
    disp_cfg.lvgl_port_cfg.timer_period_ms = 50; /* 500 ms */
    bsp_display_start_with_config(&disp_cfg);


    // bsp_sdcard_init();

    // ESP_ERROR_CHECK(bsp_extra_audio_init());

    // ESP_ERROR_CHECK(bsp_extra_player_init());
    // ESP_ERROR_CHECK(bsp_extra_player_play_file("/sdcard/xyz.mp3"));


    // ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // ESP_ERROR_CHECK(example_connect());

    // extern void test_task();
    // test_task();

    extern void marketing_task(void *arg);
    // // marketing_task();
    xTaskCreate(marketing_task, "marketing_task", 1026 * 10, NULL, 5, NULL);
}