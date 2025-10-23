#pragma once

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/sdmmc_host.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "audio/bsp_audio.h"
#include "sdcard/bsp_sdcard.h"
#include "qmi8658/bsp_qmi8658.h"

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "esp_lvgl_port.h"
#include "lvgl.h"
#endif


// @brief audio configuration
/* 如果使用 Duplex I2S 模式，请注释下面一行 */
// #define AUDIO_I2S_METHOD_SIMPLEX

#ifdef AUDIO_I2S_METHOD_SIMPLEX
#define AUDIO_I2S_MIC_GPIO_WS       GPIO_NUM_2
#define AUDIO_I2S_MIC_GPIO_SCK      GPIO_NUM_42
#define AUDIO_I2S_MIC_GPIO_DIN      GPIO_NUM_41
#define AUDIO_I2S_SPK_GPIO_DOUT     GPIO_NUM_19
#define AUDIO_I2S_SPK_GPIO_BCLK     GPIO_NUM_20
#define AUDIO_I2S_SPK_GPIO_LRCK     GPIO_NUM_1
#else
#define AUDIO_I2S_GPIO_WS           GPIO_NUM_43
#define AUDIO_I2S_GPIO_BCLK         GPIO_NUM_44
#define AUDIO_I2S_GPIO_DIN          GPIO_NUM_2
#define AUDIO_I2S_GPIO_DOUT         GPIO_NUM_19

#endif


//@brief SD Card GPIOs
#define BSP_SD_CS                    (GPIO_NUM_42)
#define BSP_SD_MOUNT_POINT           "/sdcard"


// @brief LCD configuration
#define BSP_LCD_COLOR_FORMAT        (ESP_LCD_COLOR_FORMAT_RGB565)
/* LCD display color bytes endianess */
#define BSP_LCD_BIGENDIAN           (0)
/* LCD display color bits */
#define BSP_LCD_BITS_PER_PIXEL      (16)
/* LCD display color space */
#define BSP_LCD_COLOR_SPACE         (ESP_LCD_COLOR_SPACE_RGB)
#define BSP_LCD_H_RES               (320)
#define BSP_LCD_V_RES               (840)
#define LCD_LEDC_CH                 CONFIG_BSP_DISPLAY_BRIGHTNESS_LEDC_CH

#define BSP_LCD_SCLK                (GPIO_NUM_39)
#define BSP_LCD_MOSI                (GPIO_NUM_40)
#define BSP_LCD_MISO                (GPIO_NUM_NC)
#define BSP_LCD_RST                 (GPIO_NUM_NC)
#define BSP_LCD_DC                  (GPIO_NUM_21)
#define BSP_LCD_BL                  (GPIO_NUM_46)
#define BSP_LCD_CS                  (GPIO_NUM_45)
#define EXAMPLE_LCD_IO_SPI_CS       GPIO_NUM_45
#define EXAMPLE_LCD_IO_SPI_SCK      GPIO_NUM_39
#define EXAMPLE_LCD_IO_SPI_SDO      GPIO_NUM_40
#define EXAMPLE_LCD_IO_RGB_DE       GPIO_NUM_7
#define EXAMPLE_LCD_IO_RGB_PCLK     GPIO_NUM_6
#define EXAMPLE_LCD_IO_RGB_VSYNC    GPIO_NUM_4
#define EXAMPLE_LCD_IO_RGB_HSYNC    GPIO_NUM_5
#define EXAMPLE_LCD_IO_RGB_DISP 
#define EXAMPLE_LCD_IO_RGB_RESET    GPIO_NUM_NC
#define EXAMPLE_LCD_IO_RGB_R0       GPIO_NUM_12
#define EXAMPLE_LCD_IO_RGB_R1       GPIO_NUM_11
#define EXAMPLE_LCD_IO_RGB_R2       GPIO_NUM_8
#define EXAMPLE_LCD_IO_RGB_R3       GPIO_NUM_16
#define EXAMPLE_LCD_IO_RGB_R4       GPIO_NUM_15
#define EXAMPLE_LCD_IO_RGB_G0       GPIO_NUM_0
#define EXAMPLE_LCD_IO_RGB_G1       GPIO_NUM_14
#define EXAMPLE_LCD_IO_RGB_G2       GPIO_NUM_10
#define EXAMPLE_LCD_IO_RGB_G3       GPIO_NUM_9
#define EXAMPLE_LCD_IO_RGB_G4       GPIO_NUM_3
#define EXAMPLE_LCD_IO_RGB_G5       GPIO_NUM_13
#define EXAMPLE_LCD_IO_RGB_B0       GPIO_NUM_48
#define EXAMPLE_LCD_IO_RGB_B1       GPIO_NUM_47
#define EXAMPLE_LCD_IO_RGB_B2       GPIO_NUM_1
#define EXAMPLE_LCD_IO_RGB_B3       GPIO_NUM_21
#define EXAMPLE_LCD_IO_RGB_B4       GPIO_NUM_41
#define BSP_LCD_PIXEL_CLOCK_HZ      (40)
#define BSP_LCD_DRAW_BUFF_SIZE      (BSP_LCD_H_RES * 100) // Frame buffer size in pixels
#define BSP_LCD_DRAW_BUFF_DOUBLE    (1)

/**************************************************************************************************
 *
 * I2C interface
 *
 * There are multiple devices connected to SPI peripheral:
 *  - qmi8658
 *  - pcf   
 **************************************************************************************************/

esp_err_t bsp_i2c_init(void);

i2c_master_bus_handle_t bsp_i2c_handle_get(void);

int do_i2cdetect_cmd(i2c_master_bus_handle_t tool_bus_handle);



/**************************************************************************************************
 *
 * SPI interface
 *
 * There are multiple devices connected to SPI peripheral:
 *  - LCD Display
 *  - SD Card
 **************************************************************************************************/

esp_err_t bsp_spi_init(void);




/**************************************************************************************************
 *
 * LCD interface
 *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling and LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * Display's backlight must be enabled explicitly by calling bsp_display_backlight_on()
 **************************************************************************************************/

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)

/**
 * @brief BSP display configuration structure
 *
 */
typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;  /*!< LVGL port configuration */
    uint32_t        buffer_size;    /*!< Size of the buffer for the screen in pixels */
    bool            double_buffer;  /*!< True, if should be allocated two buffers */
    struct {
        unsigned int buff_dma: 1;    /*!< Allocated LVGL buffer will be DMA capable */
        unsigned int buff_spiram: 1; /*!< Allocated LVGL buffer will be in PSRAM */
        unsigned int sw_rotate: 1;   /*!< Use software rotation (slower), The feature is unavailable under avoid-tear mode */
    } flags;
} bsp_display_cfg_t;

/**
 * @brief BSP display return handles
 *
 */
typedef struct {
    esp_lcd_spi_bus_handle_t    spi_bus;        /*!< spi bus handle */
    esp_lcd_panel_io_handle_t   io;            /*!< ESP LCD IO handle */
    esp_lcd_panel_handle_t      panel;         /*!< ESP LCD panel (color) handle */
    esp_lcd_panel_handle_t      control;       /*!< ESP LCD panel (control) handle */
} bsp_lcd_handles_t;

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start(void);

/**
 * @brief Initialize display
 *
 * This function initializes SPI, display controller and starts LVGL handling task.
 * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
 *
 * @param cfg display configuration
 *
 * @return Pointer to LVGL display or NULL when error occured
 */
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

/**
 * @brief Get pointer to input device (touch, buttons, ...)
 *
 * @note The LVGL input device is initialized in bsp_display_start() function.
 *
 * @return Pointer to LVGL input device or NULL when not initialized
 */
lv_indev_t *bsp_display_get_input_dev(void);

/**
 * @brief Take LVGL mutex
 *
 * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
 * @return true  Mutex was taken
 * @return false Mutex was NOT taken
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 */
void bsp_display_unlock(void);

/**
 * @brief Rotate screen
 *
 * Display must be already initialized by calling bsp_display_start()
 *
 * @param[in] disp Pointer to LVGL display
 * @param[in] rotation Angle of the display rotation
 */
void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0


 void bsp_extra_record();
 void bsp_extra_weather();

/* Show weather info on LVGL UI (implemented in components/bsp/ui_weather.c) */
/* Avoid including weather.h from managed component in public header to reduce include noise.
 * Use opaque pointer; implementation will cast to weather_info_t. */
void ui_weather_show(const void *info);

/* Update weather info on LVGL UI without re-initializing (for periodic updates) */
void ui_weather_update(const void *info);

/* ----------------- Simple UI helpers (implemented in components/bsp/ui.c) ----------------- */
/**
 * @brief Initialize simple UI widgets (call after display started)
 */
void ui_init(void);

/**
 * @brief Set the UI log text (replaces existing content)
 * @param txt NULL terminated string
 */
void ui_log_set(const char *txt);

/**
 * @brief Append formatted text to the UI log area
 */
void ui_log_append(const char *fmt, ...);

/**
 * @brief Clear UI log
 */
void ui_log_clear(void);


esp_err_t bsp_spiffs_mount(void);