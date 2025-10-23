#include "bsp_board.h"
#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#include "lvgl.h"
#include <stdio.h>
#include <string.h>

#include "weather.h"

static lv_obj_t *weather_screen = NULL;
static lv_obj_t *label_city = NULL;
static lv_obj_t *label_details = NULL;

void ui_weather_create(void)
{
    if (weather_screen) return;
    bsp_display_lock(0);
    weather_screen = lv_obj_create(NULL);

    label_city = lv_label_create(weather_screen);
    lv_obj_align(label_city, LV_ALIGN_TOP_MID, 0, 10);


    label_details = lv_label_create(weather_screen);
    lv_obj_set_width(label_details, lv_obj_get_width(weather_screen) - 20);
    lv_obj_align(label_details, LV_ALIGN_CENTER, 0, 60);
    
    bsp_display_unlock();
}

void ui_weather_update(const void *info_ptr)
{
    const weather_info_t *info = (const weather_info_t *)info_ptr;
    if (!info) return;

    char buf[128];
    char details[512];
    details[0] = '\0';
    bsp_display_lock(0);
    /* Location */
    if (info->location_info && info->location_info->city) {
        // snprintf(buf, sizeof(buf), "Location: %s", info->location_info->city);
        snprintf(buf, sizeof(buf), "Location: %s", "shenzhen");
        lv_label_set_text(label_city, buf);
    } else {
        lv_label_set_text(label_city, "Location: unknown");
    }

    /* Build multi-line details similar to weather_print_info */
    int off = 0;
    if (info->temperature != 0.0f)  off += snprintf(details + off, sizeof(details) - off, "Temperature: %.1f C\n", info->temperature);
    if (info->feels_like != 0.0f)   off += snprintf(details + off, sizeof(details) - off, "Feels like: %.1f C\n", info->feels_like);
    if (info->humidity != 0.0f)     off += snprintf(details + off, sizeof(details) - off, "Humidity: %.1f%%\n", info->humidity);
    // if (info->wind_dir)             off += snprintf(details + off, sizeof(details) - off, "Wind dir: %s\n", info->wind_dir);
    if (info->wind_scale)           off += snprintf(details + off, sizeof(details) - off, "Wind scale: %s\n", info->wind_scale);
    if (info->wind_speed != 0.0f)   off += snprintf(details + off, sizeof(details) - off, "Wind speed: %.1f km/h\n", info->wind_speed);
    if (info->precip != 0.0f)       off += snprintf(details + off, sizeof(details) - off, "Precipitation: %.1f mm\n", info->precip);
    if (info->pressure != 0.0f)     off += snprintf(details + off, sizeof(details) - off, "Pressure: %.1f hPa\n", info->pressure);
    if (info->visibility != 0.0f)   off += snprintf(details + off, sizeof(details) - off, "Visibility: %.1f km\n", info->visibility);
    if (info->cloud != 0.0f)        off += snprintf(details + off, sizeof(details) - off, "Cloud: %.1f%%\n", info->cloud);
    if (info->dew_point != 0.0f)    off += snprintf(details + off, sizeof(details) - off, "Dew point: %.1f C\n", info->dew_point);

    if (off > 0) {
        lv_label_set_text(label_details, details);
    } else {
        lv_label_set_text(label_details, "");
    }

    lv_scr_load(weather_screen);
    bsp_display_unlock();
}

void ui_weather_show(const void *info_ptr)
{
    const weather_info_t *info = (const weather_info_t *)info_ptr;
    if (!info) return;
    
    // 只在第一次调用时初始化UI
    ui_init();
    ui_weather_create();
    
    // 更新天气数据
    ui_weather_update(info);
}
#else
void ui_weather_show(const weather_info_t *info) {(void)info;}
void ui_weather_update(const weather_info_t *info) {(void)info;}
#endif
