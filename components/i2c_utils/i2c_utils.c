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
            res = i2c_ping(i);

            if (i % 16 == 0)
                printf("\n%.2x:", i);
            if (res == 0) {
                printf(" %.2x", i);
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

esp_err_t i2c_ping(uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return err;
}

esp_err_t i2c_commit(i2c_cmd_handle_t cmd, uint64_t timeout) {
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, timeout / portTICK_RATE_MS);

    i2c_cmd_link_delete(cmd);

    return err;
}

i2c_cmd_handle_t i2c_commit_and_create(i2c_cmd_handle_t cmd, uint64_t timeout, esp_err_t *err) {
    if (err != NULL) {
        *err = i2c_commit(cmd, timeout);
    }

    if (*err != ESP_OK) {
        return NULL;
    }

    return i2c_cmd_link_create();
}

esp_err_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t data) {
    esp_err_t err;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_write_byte(cmd, data, 1);

    i2c_master_stop(cmd);

    err = i2c_commit(cmd, 10);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t *data, size_t data_length) {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    //Move head
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, reg, 1);
    i2c_master_stop(cmd);
    cmd = i2c_commit_and_create(cmd, 10, &err);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Read data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, data, data_length, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    err = i2c_commit(cmd, 10);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

