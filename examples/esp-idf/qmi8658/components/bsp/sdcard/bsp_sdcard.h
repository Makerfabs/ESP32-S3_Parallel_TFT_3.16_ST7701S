#pragma once

#include "bsp_board.h"

typedef struct sd_t* sd_handle_t;

struct sd_t{
    sdmmc_card_t* card;

    esp_err_t (*print_sd_files)();
};

/**
 * @brief Mount SD card and initialize handle (persistent mount)
 * 
 * @param mount_point Mount point path (e.g., "/sdcard")
 * @param _handle SD card handle to initialize
 * @return esp_err_t ESP_OK on success
 */
esp_err_t sd_init(const char *mount_point);


sd_handle_t sd_handle_get();