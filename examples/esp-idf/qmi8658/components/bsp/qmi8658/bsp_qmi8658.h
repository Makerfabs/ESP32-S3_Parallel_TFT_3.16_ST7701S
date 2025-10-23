#pragma once

#include "bsp_board.h"

#include "qmi8658.h" // managed_component driver


#define QMI8658_LOCK_TIMEOUT_MS 2000

typedef struct qmi8658_t* qmi8658_handle_t;

struct qmi8658_t {
    qmi8658_dev_t dev;

    esp_err_t (*enableSensors)(uint8_t flags);
    esp_err_t (*readAccel)(float *x, float *y, float *z);
    esp_err_t (*readGyro)(float *x, float *y, float *z);
    esp_err_t (*readSensorData)(qmi8658_data_t *data);
    esp_err_t (*set)(void);

    void (*deleteDevice)(void);

    /* mutex */
    SemaphoreHandle_t lock;
};

/* Create/Destroy */
esp_err_t bsp_qmi8658_init(i2c_master_bus_handle_t bus_handle, uint8_t i2c_addr);
void bsp_qmi8658_deinit();
qmi8658_handle_t bsp_qmi8658_handle_get();