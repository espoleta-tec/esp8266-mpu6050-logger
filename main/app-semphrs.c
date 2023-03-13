//
// Created by Lazaro O'Farrill on 12/03/2023.
//


#include "app-semphrs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

esp_err_t enable_queues_and_semaphores() {
    esp_err_t err = ESP_OK;

    EnableImuReadsQueue = xQueueCreate(1, 1);
    if (EnableImuReadsQueue == NULL) {
        return ESP_FAIL;
    }

    return err;
}
