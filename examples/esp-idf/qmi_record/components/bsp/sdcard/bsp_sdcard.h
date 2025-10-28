#pragma once

#include "bsp_board.h"

typedef struct sd_t* bsp_sd_handle_t;

struct bsp_sd_t{
};

/**
 * @brief Mount SD card and initialize handle (persistent mount)
 * 
 * @param mount_point Mount point path (e.g., "/sdcard")
 * @param _handle SD card handle to initialize
 * @return esp_err_t ESP_OK on success
 */
esp_err_t bsp_sdcard_init(void);

esp_err_t bsp_sdcard_deinit(void);
