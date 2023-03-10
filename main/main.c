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

#define IMU_ADDRESS 0x68

const char *IMU_READER_TAG = "IMU_TASK";

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


    while (true) {
        err = readLastImuValues();
        if (err != ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
            return;
        }
        printf("%10d%10d%10d%10d%10d%10d\n",
               IMU.accX, IMU.accY, IMU.accZ,
               IMU.gyX, IMU.gyY, IMU.gyZ);

        vTaskDelay(1 / portTICK_RATE_MS);
    }

//    xTaskCreate(mpuReaderTask, IMU_READER_TAG, 2040, NULL, 10, NULL);
}