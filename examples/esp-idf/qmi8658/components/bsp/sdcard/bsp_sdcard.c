#include "bsp_sdcard.h"

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <dirent.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>

static const char *TAG = "bsp_sdcard";
static sd_handle_t sd_handle = NULL;


sd_handle_t sd_handle_get()
{
    return sd_handle;
}

static esp_err_t _print_sd_files()
{
    DIR *dir = NULL;
    struct dirent *entry;
    struct stat st;

    if (sd_handle == NULL) {
        ESP_LOGE(TAG, "handle is NULL");
        return ESP_ERR_INVALID_ARG;
    }
    
    dir = opendir(BSP_SD_MOUNT_POINT);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open mount point '%s'", BSP_SD_MOUNT_POINT);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Listing files in %s:", BSP_SD_MOUNT_POINT);
    while ((entry = readdir(dir)) != NULL) {
        char path[512];
        snprintf(path, sizeof(path), "%s/%s", BSP_SD_MOUNT_POINT, entry->d_name);
        if (stat(path, &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                ESP_LOGI(TAG, "  [DIR]  %s", entry->d_name);
            } else {
                ESP_LOGI(TAG, "  [FILE] %s  (%lld bytes)", entry->d_name, (long long)st.st_size);
            }
        } else {
            ESP_LOGW(TAG, "  [?] %s (stat failed)", entry->d_name);
        }
    }

    closedir(dir);
    return ESP_OK;
}

esp_err_t sd_init(const char *mount_point) 
{
    esp_err_t ret = ESP_OK;

    if (mount_point == NULL) {
        ESP_LOGE(TAG, "mount point is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    sd_handle_t _sd_handle = (sd_handle_t)malloc(sizeof(struct sd_t));
    if (_sd_handle == NULL) {
        ESP_LOGE(TAG, "Failed to allocate sd_handle");
        return ESP_ERR_NO_MEM;
    }

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    // host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    ESP_RETURN_ON_ERROR(bsp_spi_init(), TAG, "Failed to initialize SPI bus");

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = BSP_SD_CS;
    slot_config.host_id = host.slot;


    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &_sd_handle->card);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, _sd_handle->card);

    /* bind helper */
    _sd_handle->print_sd_files = _print_sd_files;

    sd_handle = _sd_handle;

    return ret;
}