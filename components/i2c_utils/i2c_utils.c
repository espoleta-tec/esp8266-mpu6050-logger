//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#include "driver/gpio.h"
#include <stdio.h>
#include <string.h>
#include "i2c_utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/i2c.h"

#define SDA_PIN 4
#define SCL_PIN 5

esp_err_t init_i2c() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.scl_io_num = SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_stretch_tick = 300;
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER);
    i2c_param_config(I2C_NUM_0, &conf);

    return ESP_OK;
}

_Noreturn
void task_i2c_scanner(void *ignore) {
    while (1) {
        esp_err_t res;
        printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
        printf("00:         ");
        for (uint8_t i = 3; i < 0x78; i++) {
            uint8_t data[] = {-1};
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, 1/* expect ack */);
            i2c_master_write_byte(cmd, 12, 1/* expect ack */);
            i2c_master_stop(cmd);
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_READ, 1/* expect ack */);
            i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
            i2c_master_stop(cmd);
            res = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);

            if (i % 16 == 0)
                printf("\n%.2x:", i);
            if (res == 0) {
                printf(" %.2x-%d", i, data[0]);
            } else
                printf(" --");
        }
        printf("\n\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static char i2cScannerTag[] = "i2cscanner";

void start_scanner() {
    xTaskCreate(task_i2c_scanner, i2cScannerTag, 2048, NULL, 10, NULL);
}
