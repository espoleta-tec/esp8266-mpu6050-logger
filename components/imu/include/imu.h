//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#ifndef RTOS_SDK_LAB1_IMU_H
#define RTOS_SDK_LAB1_IMU_H

#ifndef ICM_20948
#ifndef MPU_6050
#define ICM_20948
#endif
#else
#define MPU_6050
#endif

#include "esp_err.h"

esp_err_t initImu(uint8_t address);

esp_err_t imu_read_sensors();

typedef struct T_IMU {
    uint8_t address;
    uint16_t accX;
    uint16_t accY;
    uint16_t accZ;
    uint16_t gyX;
    uint16_t gyY;
    uint16_t gyZ;
} T_IMU;

extern T_IMU IMU;

#endif //RTOS_SDK_LAB1_IMU_H
