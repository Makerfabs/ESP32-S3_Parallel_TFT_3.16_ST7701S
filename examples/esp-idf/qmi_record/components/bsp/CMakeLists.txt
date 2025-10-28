set(APPS_DIR ./)
file(GLOB_RECURSE APPS_C_SRCS ${APPS_DIR}/*.c)
file(GLOB_RECURSE APPS_CPP_SRCS ${APPS_DIR}/*.cpp)

idf_component_register(SRCS ${APPS_C_SRCS} ${APPS_CPP_SRCS}
                    INCLUDE_DIRS ${APPS_DIR}
                    REQUIRES driver esp_lvgl_port qmi8658 pcf85063a nvs_flash esp_event esp_wifi esp_http_client fatfs esp-audio-player
                    PRIV_REQUIRES esp_lcd spiffs esp_lcd_st7701 lvgl
) 
