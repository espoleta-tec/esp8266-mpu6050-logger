/* I2C example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/




#include "i2c_utils.h"
#include "imu.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"

#define IMU_ADDRESS 0x68

const char *IMU_READER_TAG = "IMU_TASK";

void printTask(void *ignore) {
    esp_err_t err;
    uint32_t count = 0, lastCount = 0;

    while (1) {
        count++;
        err = readLastImuValues();
        if (err != ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
            return;
        }
        if (count - lastCount > 1000) {
            printf("%10d\n", count);
            fflush(stdout);
            lastCount = count;
        }
//        printf("%10d%10d%10d%10d%10d%10d%10u\n",
//               IMU.accX, IMU.accY, IMU.accZ,
//               IMU.gyX, IMU.gyY, IMU.gyZ, count);
        vTaskDelay(1);
    }
}

void app_main(void) {
    esp_err_t err;
    init_i2c();
    fflush(stdout);
    printf("Reading imu\n");

    err = initImu(IMU_ADDRESS);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }

    xTaskCreate(printTask, IMU_READER_TAG, 2040, NULL, 10, NULL);
}