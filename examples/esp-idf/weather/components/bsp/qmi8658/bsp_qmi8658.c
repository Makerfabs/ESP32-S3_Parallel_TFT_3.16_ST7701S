#include "bsp_qmi8658.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "bsp_qmi8658";


static qmi8658_handle_t _qmi8658_handle = NULL;

qmi8658_handle_t bsp_qmi8658_handle_get()
{
    return _qmi8658_handle;
}

static esp_err_t _qmi8658_set()
{
    esp_err_t ret = ESP_OK;

    ret |= qmi8658_set_accel_range(&_qmi8658_handle->dev, QMI8658_ACCEL_RANGE_8G);
    ret |= qmi8658_set_accel_odr(&_qmi8658_handle->dev, QMI8658_ACCEL_ODR_1000HZ);
    ret |= qmi8658_set_gyro_range(&_qmi8658_handle->dev, QMI8658_GYRO_RANGE_512DPS);
    ret |= qmi8658_set_gyro_odr(&_qmi8658_handle->dev, QMI8658_GYRO_ODR_1000HZ);

    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure QMI8658 sensor");
        return ret;
    }

    qmi8658_set_accel_unit_mps2(&_qmi8658_handle->dev, true);
    qmi8658_set_gyro_unit_rads(&_qmi8658_handle->dev, true);

    return ret;
}

static esp_err_t _qmi8658_instance_enableSensors(uint8_t flags)
{
    if (!_qmi8658_handle) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(_qmi8658_handle->lock, pdMS_TO_TICKS(QMI8658_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = qmi8658_enable_sensors(&_qmi8658_handle->dev, flags);
    xSemaphoreGive(_qmi8658_handle->lock);
    return err;
}

static esp_err_t _qmi8658_instance_readAccel(float *x, float *y, float *z)
{
    if (!_qmi8658_handle || !x || !y || !z) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(_qmi8658_handle->lock, pdMS_TO_TICKS(QMI8658_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = qmi8658_read_accel(&_qmi8658_handle->dev, x, y, z);
    xSemaphoreGive(_qmi8658_handle->lock);
    return err;
}

static esp_err_t _qmi8658_instance_readGyro(float *x, float *y, float *z)
{
    if (!_qmi8658_handle || !x || !y || !z) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(_qmi8658_handle->lock, pdMS_TO_TICKS(QMI8658_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = qmi8658_read_gyro(&_qmi8658_handle->dev, x, y, z);
    xSemaphoreGive(_qmi8658_handle->lock);
    return err;
}

static esp_err_t _qmi8658_instance_readSensorData(qmi8658_data_t *data)
{
    if (!_qmi8658_handle || !data) return ESP_ERR_INVALID_ARG;
    if (xSemaphoreTake(_qmi8658_handle->lock, pdMS_TO_TICKS(QMI8658_LOCK_TIMEOUT_MS)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t err = qmi8658_read_sensor_data(&_qmi8658_handle->dev, data);
    xSemaphoreGive(_qmi8658_handle->lock);
    return err;
}

static void _qmi8658_instance_delete()
{
    if (!_qmi8658_handle) return;
    if (_qmi8658_handle->lock) {
        if (xSemaphoreTake(_qmi8658_handle->lock, pdMS_TO_TICKS(QMI8658_LOCK_TIMEOUT_MS)) == pdTRUE) {
            xSemaphoreGive(_qmi8658_handle->lock);
        }
        vSemaphoreDelete(_qmi8658_handle->lock);
        _qmi8658_handle->lock = NULL;
    }
    free(_qmi8658_handle);
}

esp_err_t bsp_qmi8658_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr)
{
    qmi8658_handle_t _handle = (qmi8658_handle_t)calloc(1, sizeof(struct qmi8658_t));
    if(!_handle) return ESP_ERR_NO_MEM;

    _handle->dev.bus_handle = bus_handle;
    _handle->dev.dev_handle = NULL;
    esp_err_t err = qmi8658_init(&_handle->dev, bus_handle, i2c_addr);
    if (err != ESP_OK) {
        free(_handle);
        return err;
    }

    _handle->lock = xSemaphoreCreateMutex();
    if (!_handle->lock) {
        /* cleanup underlying device if needed */
        free(_handle);
        return ESP_ERR_NO_MEM;
    }

    _handle->set            = _qmi8658_set;
    _handle->enableSensors  = _qmi8658_instance_enableSensors;
    _handle->readAccel      = _qmi8658_instance_readAccel;
    _handle->readGyro       = _qmi8658_instance_readGyro;
    _handle->readSensorData = _qmi8658_instance_readSensorData;
    _handle->deleteDevice   = _qmi8658_instance_delete;

    /* publish global handle so instance wrappers can access it */
    _qmi8658_handle = _handle;

    return ESP_OK;
}

void bsp_qmi8658_deinit()
{
    if (_qmi8658_handle) {
        _qmi8658_handle->deleteDevice();
        _qmi8658_handle = NULL;
    }
}
