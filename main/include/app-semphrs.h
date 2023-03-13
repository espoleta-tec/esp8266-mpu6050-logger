//
// Created by Lazaro O'Farrill on 12/03/2023.
//

#ifndef RTOS_SDK_LAB1_APP_SEMPHRS_H
#define RTOS_SDK_LAB1_APP_SEMPHRS_H

#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

QueueHandle_t EnableImuReadsQueue;

esp_err_t enable_queues_and_semaphores();

#endif //RTOS_SDK_LAB1_APP_SEMPHRS_H
