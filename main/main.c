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

#include "logger.h"
#include "stdlib.h"
#include "driver/gpio.h"
#include "adc_ctl.h"
#include "imu_read_ctl.h"
#include "config.h"


// Definitions
const char *MAIN_TAG = "ESP8266";
static uint32_t count = 0;

const char *PRINT_DATA_TASK_LABEL = "PRINT DATA";

_Noreturn void task_print_data(void *ignore) {
    while (1) {
        if (hw_timer_get_enable()) {
            printf("%10d%10d%10d%10d%10d%10d%10u\n",
                   IMU.accX, IMU.accY, IMU.accZ,
                   IMU.gyX, IMU.gyY, IMU.gyZ, count);
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}


void app_main(void) {
    esp_err_t err;
    init_i2c();
    fflush(stdout);
    ESP_LOGI(MAIN_TAG, "Starting system");

    logger_print_status_header();

    //Configure GPIO
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set, e.g.GPIO15/16
    io_conf.pin_bit_mask = GPIO_OUTPUT_MASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(LED_PIN, 1);


    err = imu_monitor_config();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }

    err = adc_config();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return;
    }

    //Spawning processes
    xTaskCreate(task_print_data, PRINT_DATA_TASK_LABEL, 1024, NULL, 2, NULL);
    xTaskCreate(task_read_adc, READ_ADC_TASK_LABEL, 1024, NULL, 4, NULL);
}