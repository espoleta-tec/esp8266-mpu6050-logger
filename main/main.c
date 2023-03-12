#include <sys/cdefs.h>
/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/


#include "freertos/FreeRTOSConfig.h"


#include "i2c_utils.h"
#include "imu.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "driver/hw_timer.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "logger.h"
#include "stdlib.h"

#define IMU_ADDRESS 0x68

const char *MAIN_TAG = "ESP8266";
static uint32_t count = 0;

static SemaphoreHandle_t sem_done_reading = NULL;
static QueueHandle_t msgQueue;

const char *PRINT_DATA_TASK_LABEL = "PRINT DATA";

_Noreturn void task_print_data(void *ignore) {
    while (1) {
        printf("%10d%10d%10d%10d%10d%10d%10u\n",
               IMU.accX, IMU.accY, IMU.accZ,
               IMU.gyX, IMU.gyY, IMU.gyZ, count);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

const char *READ_IMU_TASK_LABEL = "IMU_TASK";

TaskHandle_t readImuTaskHandle;


//void task_read_imu(void *ignore) {
//
//    while (1) {
//        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//
//        esp_err_t err;
//        err = imu_read_sensors();
//        if (err != ESP_OK) {
//            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
//            return;
//        }
//        xSemaphoreGive(sem_done_reading);
////        count++;
//    }
//}

void pendable_imu_read(void *pvParameter, uint32_t ulParameter2) {
    esp_err_t err;
    err = imu_read_sensors();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }
    count++;
}


void read_imu_timer_callback() {
    BaseType_t taskWoken = pdFALSE;

    xTimerPendFunctionCallFromISR(pendable_imu_read, (void *) 0, 0, &taskWoken);

    if (taskWoken) {
        portYIELD_FROM_ISR();
    }
}

void app_main(void) {
    esp_err_t err;
    init_i2c();
    fflush(stdout);
    ESP_LOGI(MAIN_TAG, "Starting system");

    logger_print_status_header();

    err = initImu(IMU_ADDRESS);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    };


    return;

    const uint32_t imuReadPeriodUs = 4000;
    err = hw_timer_init(read_imu_timer_callback, NULL);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }

    err = hw_timer_alarm_us(imuReadPeriodUs, true);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }


    logger_print_statusf("IMU Read Interval", "%dus", imuReadPeriodUs);

    vTaskDelay(pdMS_TO_TICKS(10));

    //Spawning processes
    xTaskCreate(task_print_data, PRINT_DATA_TASK_LABEL, 1024, NULL, 2, NULL);
}