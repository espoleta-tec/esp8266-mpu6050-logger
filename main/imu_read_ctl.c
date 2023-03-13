//
// Created by Lazaro O'Farrill on 12/03/2023.
//

#include <esp_err.h>
#include <portmacro.h>
#include <driver/hw_timer.h>
#include "imu_read_ctl.h"
#include "imu.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "logger.h"
#include "config.h"

const uint32_t imuReadPeriodUs = 4000;


void pendable_imu_read(void *pvParameter, uint32_t ulParameter2) {
    esp_err_t err;
    err = imu_read_sensors();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }
}

void read_imu_timer_callback() {
    BaseType_t taskWoken = pdFALSE;

    xTimerPendFunctionCallFromISR(pendable_imu_read, (void *) 0, 0, &taskWoken);

    if (taskWoken) {
        portYIELD_FROM_ISR();
    }
}

esp_err_t imu_monitor_config() {
    esp_err_t err;

    //Configure Inertial Sensor
    err = initImu(IMU_ADDRESS);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    };

    logger_print_statusf("IMU Read Interval", "%dus", imuReadPeriodUs);

    //Init timer
    err = hw_timer_init(read_imu_timer_callback, NULL);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t toggle_reads() {
    esp_err_t err;
    bool timerEnabled = hw_timer_get_enable();
    if (timerEnabled) {
        err = hw_timer_disarm();
        if (err != ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
            return err;
        }
        gpio_set_level(LED_PIN, 1);
    } else {
        err = hw_timer_alarm_us(imuReadPeriodUs, true);
        if (err != ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
            return err;
        }
        gpio_set_level(LED_PIN, 0);
    }

    return err;
}