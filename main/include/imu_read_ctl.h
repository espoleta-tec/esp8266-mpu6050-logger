//
// Created by Lazaro O'Farrill on 12/03/2023.
//

#ifndef RTOS_SDK_LAB1_IMU_READ_CTL_H
#define RTOS_SDK_LAB1_IMU_READ_CTL_H


#include "stdint.h"

esp_err_t imu_monitor_config();

void pendable_imu_read(void *pvParameter, uint32_t ulParameter2);

void read_imu_timer_callback();

esp_err_t toggle_reads();

#endif //RTOS_SDK_LAB1_IMU_READ_CTL_H
