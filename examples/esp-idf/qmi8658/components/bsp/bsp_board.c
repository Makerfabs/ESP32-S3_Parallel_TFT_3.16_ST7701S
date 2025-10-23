#include "bsp_board.h"

#include "driver/spi_master.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_st7701.h"
#include "driver/ledc.h"
#include "assert.h"
#include "esp_spiffs.h"

#include "weather.h"
#include "esp_timer.h"


static const char *TAG = "board";

// @brief spi configuration
static bool spi_initalized = false;


// @brief i2c configuration
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 3, 0)
static i2c_master_bus_handle_t i2c_bus_handle = NULL;
#endif
static bool i2c_initalized = false;


int do_i2cdetect_cmd(i2c_master_bus_handle_t tool_bus_handle)
{
    uint8_t address;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
    for (int i = 0; i < 128; i += 16) {
        printf("%02x: ", i);
        for (int j = 0; j < 16; j++) {
            fflush(stdout);
            address = i + j;
            esp_err_t ret = i2c_master_probe(tool_bus_handle, address, 50);
            if (ret == ESP_OK) {
                printf("%02x ", address);
            } else if (ret == ESP_ERR_TIMEOUT) {
                printf("UU ");
            } else {
                printf("-- ");
            }
        }
        printf("\r\n");
    }

    return 0;
}

esp_err_t bsp_i2c_init(void)
{
    esp_err_t ret = ESP_OK;
    if(i2c_initalized) {
        return ret;
    }

    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = CONFIG_BSP_I2C_SDA_NUM,
        .scl_io_num = CONFIG_BSP_I2C_SCL_NUM,
        .i2c_port = CONFIG_BSP_I2C_NUM,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = 1,
    };

    ret = i2c_new_master_bus(&bus_cfg, &i2c_bus_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed");
        return ret;
    }

    i2c_initalized = true;

    // do_i2cdetect_cmd(i2c_bus_handle);
    return ret;
}

esp_err_t bsp_i2c_deinit(void)
{
    esp_err_t ret = ESP_OK;

    if(!i2c_initalized) {
        return ret;
    }
    ret = i2c_del_master_bus(i2c_bus_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus deinit failed");
        return ret;
    }

    i2c_initalized = false;
    return ESP_OK;
}

i2c_master_bus_handle_t bsp_i2c_handle_get(void)
{
    return i2c_bus_handle;
}

esp_err_t bsp_spi_init(void)
{
    esp_err_t ret = ESP_OK;
    if(spi_initalized) {
        return ret;
    }
#if CONFIG_BSP_SPI2_ENABLE
    spi_bus_config_t buscfg = {
        .miso_io_num = CONFIG_BSP_SPI2_MISO_NUM,
        .mosi_io_num = CONFIG_BSP_SPI2_MOSI_NUM,
        .sclk_io_num = CONFIG_BSP_SPI2_SCLK_NUM,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4 * 1000,
    };
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "spi bus init failed");
        return ret;
    }
    spi_initalized = true;
#endif
    return ret;
}


esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    ESP_ERROR_CHECK(ret);

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister("storage");
}

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

static const st7701_lcd_init_cmd_t lcd_init_cmds[] = 
{
  //   cmd   data        data_size  delay_ms 1
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xEF,(uint8_t []){0x08},1,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x10},5,0},
  {0xC0,(uint8_t []){0xE5,0x02},2,0},
  {0xC1,(uint8_t []){0x15,0x0A},2,0},
  {0xC2,(uint8_t []){0x07,0x02},2,0},
  {0xCC,(uint8_t []){0x10},1,0},
  {0xB0,(uint8_t []){0x00,0x08,0x51,0x0D,0xCE,0x06,0x00,0x08,0x08,0x24,0x05,0xD0,0x0F,0x6F,0x36,0x1F},16,0},
  {0xB1,(uint8_t []){0x00,0x10,0x4F,0x0C,0x11,0x05,0x00,0x07,0x07,0x18,0x02,0xD3,0x11,0x6E,0x34,0x1F},16,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x11},5,0},
  {0xB0,(uint8_t []){0x4D},1,0},
  {0xB1,(uint8_t []){0x37},1,0},
  {0xB2,(uint8_t []){0x87},1,0},
  {0xB3,(uint8_t []){0x80},1,0},
  {0xB5,(uint8_t []){0x4A},1,0},
  {0xB7,(uint8_t []){0x85},1,0},
  {0xB8,(uint8_t []){0x21},1,0},
  {0xB9,(uint8_t []){0x00,0x13},2,0},
  {0xC0,(uint8_t []){0x09},1,0},
  {0xC1,(uint8_t []){0x78},1,0},
  {0xC2,(uint8_t []){0x78},1,0},
  {0xD0,(uint8_t []){0x88},1,0},
  {0xE0,(uint8_t []){0x80,0x00,0x02},3,100},
  {0xE1,(uint8_t []){0x0F,0xA0,0x00,0x00,0x10,0xA0,0x00,0x00,0x00,0x60,0x60},11,0},
  {0xE2,(uint8_t []){0x30,0x30,0x60,0x60,0x45,0xA0,0x00,0x00,0x46,0xA0,0x00,0x00,0x00},13,0},
  {0xE3,(uint8_t []){0x00,0x00,0x33,0x33},4,0},
  {0xE4,(uint8_t []){0x44,0x44},2,0},
  {0xE5,(uint8_t []){0x0F,0x4A,0xA0,0xA0,0x11,0x4A,0xA0,0xA0,0x13,0x4A,0xA0,0xA0,0x15,0x4A,0xA0,0xA0},16,0},
  {0xE6,(uint8_t []){0x00,0x00,0x33,0x33},4,0},
  {0xE7,(uint8_t []){0x44,0x44},2,0},
  {0xE8,(uint8_t []){0x10,0x4A,0xA0,0xA0,0x12,0x4A,0xA0,0xA0,0x14,0x4A,0xA0,0xA0,0x16,0x4A,0xA0,0xA0},16,0},
  {0xEB,(uint8_t []){0x02,0x00,0x4E,0x4E,0xEE,0x44,0x00},7,0},
  {0xED,(uint8_t []){0xFF,0xFF,0x04,0x56,0x72,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x27,0x65,0x40,0xFF,0xFF},16,0},
  {0xEF,(uint8_t []){0x08,0x08,0x08,0x40,0x3F,0x64},6,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xE8,(uint8_t []){0x00,0x0E},2,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x00},5,0},
  {0x11,(uint8_t []){0x00},0,120},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x13},5,0},
  {0xE8,(uint8_t []){0x00,0x0C},2,10},
  {0xE8,(uint8_t []){0x00,0x00},2,0},
  {0xFF,(uint8_t []){0x77,0x01,0x00,0x00,0x00},5,0},
  {0x3A,(uint8_t []){0x55},1,0},
  {0x36,(uint8_t []){0x00},1,0},
  {0x35,(uint8_t []){0x00},1,0},
  {0x29,(uint8_t []){0x00},0,20},
};

esp_err_t bsp_display_brightness_init(void)
{
    // Setup LEDC peripheral for PWM backlight control
    const ledc_channel_config_t LCD_backlight_channel = {
        .gpio_num = BSP_LCD_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LCD_LEDC_CH,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = 1,
        .duty = 0,
        .hpoint = 0
    };
    const ledc_timer_config_t LCD_backlight_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = 1,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(ledc_timer_config(&LCD_backlight_timer));
    ESP_ERROR_CHECK(ledc_channel_config(&LCD_backlight_channel));
    return ESP_OK;
}

esp_err_t bsp_display_new_with_handles(bsp_lcd_handles_t *ret_handles)
{
    esp_err_t ret = ESP_OK;
    
    // ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "Brightness init failed");

    spi_line_config_t line_config = 
    {
        .cs_io_type = IO_TYPE_GPIO,             // Set to `IO_TYPE_GPIO` if using GPIO, same to below
        .cs_gpio_num = EXAMPLE_LCD_IO_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = EXAMPLE_LCD_IO_SPI_SCK,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = EXAMPLE_LCD_IO_SPI_SDO,
        .io_expander = NULL,                        // Set to NULL if not using IO expander
    };
    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle), err, TAG, "new panel IO failed");

    ESP_LOGI(TAG, "Install ST7701 panel driver");

    esp_lcd_rgb_panel_config_t rgb_config =
        {
            .clk_src = LCD_CLK_SRC_DEFAULT,
            .psram_trans_align = 64,
            .bounce_buffer_size_px = 10 * BSP_LCD_H_RES,
            .num_fbs = 2,
            .data_width = 16,
            .bits_per_pixel = 16,
            .de_gpio_num = EXAMPLE_LCD_IO_RGB_DE,
            .pclk_gpio_num = EXAMPLE_LCD_IO_RGB_PCLK,
            .vsync_gpio_num = EXAMPLE_LCD_IO_RGB_VSYNC,
            .hsync_gpio_num = EXAMPLE_LCD_IO_RGB_HSYNC,
            .flags.fb_in_psram = true,
            .disp_gpio_num = -1,
            .data_gpio_nums = {
                // BGR
                // EXAMPLE_LCD_IO_RGB_R0,
                // EXAMPLE_LCD_IO_RGB_R1,
                // EXAMPLE_LCD_IO_RGB_R2,
                // EXAMPLE_LCD_IO_RGB_R3,
                // EXAMPLE_LCD_IO_RGB_R4,
                // EXAMPLE_LCD_IO_RGB_G0,
                // EXAMPLE_LCD_IO_RGB_G1,
                // EXAMPLE_LCD_IO_RGB_G2,
                // EXAMPLE_LCD_IO_RGB_G3,
                // EXAMPLE_LCD_IO_RGB_G4,
                // EXAMPLE_LCD_IO_RGB_G5,
                EXAMPLE_LCD_IO_RGB_B0,
                EXAMPLE_LCD_IO_RGB_B1,
                EXAMPLE_LCD_IO_RGB_B2,
                EXAMPLE_LCD_IO_RGB_B3,
                EXAMPLE_LCD_IO_RGB_B4,
                EXAMPLE_LCD_IO_RGB_G0,
                EXAMPLE_LCD_IO_RGB_G1,
                EXAMPLE_LCD_IO_RGB_G2,
                EXAMPLE_LCD_IO_RGB_G3,
                EXAMPLE_LCD_IO_RGB_G4,
                EXAMPLE_LCD_IO_RGB_G5,
                EXAMPLE_LCD_IO_RGB_R0,
                EXAMPLE_LCD_IO_RGB_R1,
                EXAMPLE_LCD_IO_RGB_R2,
                EXAMPLE_LCD_IO_RGB_R3,
                EXAMPLE_LCD_IO_RGB_R4,
            },
            .timings = {
                .pclk_hz = 18 * 1000 * 1000,
                .h_res = BSP_LCD_H_RES,
                .v_res = BSP_LCD_V_RES,
                .hsync_back_porch = 30,
                .hsync_front_porch = 30, // 30
                .hsync_pulse_width = 6,
                .vsync_back_porch = 20,  // 10-100 40
                .vsync_front_porch = 20, // 10-100 70
                .vsync_pulse_width = 40,
            },

        };
    st7701_vendor_config_t vendor_config = 
    {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds,      // Uncomment these line if use custom initialization commands
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = 
        {
        .mirror_by_cmd = 0,       // Only work when `enable_io_multiplex` is set to 0
        .enable_io_multiplex = 1, /**
                                    * Set to 1 if panel IO is no longer needed after LCD initialization.
                                    * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
                                    * Please set it to 1 to release the pins.
                                    * ENABLED: Allow SD card to use SPI after LCD init (RGB mode)
                                    */
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = 
    {
        .reset_gpio_num = EXAMPLE_LCD_IO_RGB_RESET,     // Set to -1 if not use
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,     // Implemented by LCD command `36h`
        .bits_per_pixel = 16,    // Implemented by LCD command `3Ah` (16/18/24)
        .vendor_config = &vendor_config,
    };     
    esp_lcd_panel_handle_t disp_panel = NULL;
    ESP_GOTO_ON_ERROR(esp_lcd_new_panel_st7701(io_handle, &panel_config, &disp_panel), err, TAG, "new panel st7701 failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_reset(disp_panel), err, TAG, "LCD panel reset failed");
    ESP_GOTO_ON_ERROR(esp_lcd_panel_init(disp_panel), err, TAG, "LCD panel init failed");

    /* Return all handles */
    ret_handles->io = io_handle;
    // ret_handles->spi_bus = (esp_lcd_spi_bus_handle_t)BSP_LCD_HOST;
    ret_handles->panel = disp_panel;
    ret_handles->control = NULL;

    ESP_LOGI(TAG, "Display initialized");

    return ret;

err:
    if (disp_panel) {
        esp_lcd_panel_del(disp_panel);
    }
    if (io_handle) {
        esp_lcd_panel_io_del(io_handle);
    }
    // spi_bus_free((spi_host_device_t)BSP_LCD_HOST);
    return ret;
}

static lv_display_t *bsp_display_lcd_init(const bsp_display_cfg_t *cfg)
{
    assert(cfg != NULL);
    bsp_lcd_handles_t lcd_panels;
    ESP_ERROR_CHECK(bsp_display_new_with_handles(&lcd_panels));

    /* Add LCD screen */
    ESP_LOGI(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_panels.io,
        .panel_handle = lcd_panels.panel,
        .control_handle = lcd_panels.control,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
#if LVGL_VERSION_MAJOR >= 9
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
        .color_format = LV_COLOR_FORMAT_RGB888,
#else
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
#endif
        .flags = {
            .buff_dma = cfg->flags.buff_dma,
            .buff_spiram = cfg->flags.buff_spiram,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = (BSP_LCD_BIGENDIAN ? true : false),
#endif
            .sw_rotate = false,                /* Avoid tearing is not supported for SW rotation */
#if CONFIG_BSP_DISPLAY_LVGL_FULL_REFRESH
            .full_refresh = true,
#elif CONFIG_BSP_DISPLAY_LVGL_DIRECT_MODE
            .direct_mode = true,
#endif
        }
    };

    lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
            .bb_mode = true,
            .avoid_tearing = false,
        }
    };
    // return lvgl_port_add_disp(&disp_cfg);
    return lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    lv_display_t *disp;

    assert(cfg != NULL);

    ESP_ERROR_CHECK(lvgl_port_init(&cfg->lvgl_port_cfg));

    ESP_ERROR_CHECK(bsp_display_brightness_init());

    assert((disp = bsp_display_lcd_init(cfg)) != NULL);

    // BSP_NULL_CHECK(disp_indev = bsp_display_indev_init(disp), NULL);

    return disp;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = BSP_LCD_DRAW_BUFF_SIZE,
        .double_buffer = BSP_LCD_DRAW_BUFF_DOUBLE,
        .flags = {
#if CONFIG_BSP_LCD_COLOR_FORMAT_RGB888
            .buff_dma = false,
#else
            .buff_dma = false,
#endif
            .buff_spiram = true,
            .sw_rotate = false,
        }
    };

    return bsp_display_start_with_config(&cfg);
}


// lv_indev_t *bsp_display_get_input_dev(void)
// {
//     return disp_indev;
// }

void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation)
{
    lv_disp_set_rotation(disp, rotation);
}

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

#endif // BSP_CONFIG_NO_GRAPHIC_LIB






/**************************************************************************************************
 *
 * extra functions
 *
 *  - record
 *  - pcf   
 **************************************************************************************************/

 void bsp_extra_record()
 {
    audio_init(16000, 16000, AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);

    audio_handle_t audio_handle = bsp_audio_handle_get();

    const size_t buffer_size = 16000 * 2 * 5;

    uint8_t * buffer = (uint8_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
    if (buffer == NULL) {
        ESP_LOGE(TAG, "bsp_extra_record malloc buffer failed");
        return;
        vTaskDelete(NULL);
    }

    while(true) {
        FILE *fp = fopen("/sdcard/record.wav", "wb");
        if (fp == NULL) {
            ESP_LOGE(TAG, "Failed to open file for writing");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        size_t bytes_read = 0;
        ui_log_append("Recording...");
        size_t i;
        for(i = 0; i < buffer_size; i += bytes_read) {
            size_t read_len = (buffer_size - i) > 2048 ? 2048 : (buffer_size - i);
            audio_handle->read(buffer + i, read_len, &bytes_read, pdMS_TO_TICKS(1000));
            ESP_LOGI(TAG, "Recorded %d bytes", bytes_read);
        }
        ui_log_append("read %lu bytes", buffer_size);

        // 写入wav头，直接用数组写入
        uint8_t wav_header[44] = {
            'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ',
            16, 0, 0, 0, 1, 0, 1, 0, 64, 0, 0, 0, 128, 62, 0, 0,
            2, 0, 16, 0, 'd', 'a', 't', 'a', 0, 0, 0, 0
        };
        // 计算文件大小并写入wav头
        uint32_t file_size = buffer_size + 44 - 8;
        wav_header[4] = (uint8_t)(file_size & 0xFF);
        wav_header[5] = (uint8_t)((file_size >> 8) & 0xFF);
        wav_header[6] = (uint8_t)((file_size >> 16) & 0xFF);
        wav_header[7] = (uint8_t)((file_size >> 24) & 0xFF);
        uint32_t data_size = buffer_size;
        wav_header[40] = (uint8_t)(data_size & 0xFF);
        wav_header[41] = (uint8_t)((data_size >> 8) & 0xFF);
        wav_header[42] = (uint8_t)((data_size >> 16) & 0xFF);
        wav_header[43] = (uint8_t)((data_size >> 24) & 0xFF);

        fwrite(wav_header, 1, 44, fp);
        fwrite(buffer, 1, buffer_size, fp);
        fflush(fp);
        fclose(fp);

        vTaskDelay(pdMS_TO_TICKS(1000));


        ui_log_append("Playing \n");
        audio_handle->play("/sdcard/record.wav", AUDIO_PLAY_FORMAT_WAV);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}


void bsp_extra_weather()
{
    static weather_config_t config = {
        .api_key = "737ce9c987b34b508166092085bac65b",
        .api_host = "kw487jn3du.re.qweatherapi.com",    // 和风天气需要配置host
        .city = NULL, // city为NULL自动根据IP地址获取位置,也可以指定城市
        // .city = "北京",
        .type = WEATHER_HEFENG  //可更改为api配置WEATHER_AMAP或WEATHER_XINZHI
    };

    // 立即获取一次天气信息
    weather_info_t *info = weather_get(&config);
    if (info) {
        ui_weather_show(info);
        weather_info_free(info);
    }
}

