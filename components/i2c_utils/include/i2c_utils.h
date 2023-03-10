//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#ifndef RTOS_SDK_LAB1_I2C_UTILS_H
#define RTOS_SDK_LAB1_I2C_UTILS_H

#include "esp_err.h"
#include "driver/i2c.h"

esp_err_t init_i2c();

void start_scanner();

esp_err_t i2c_ping(uint8_t address);

esp_err_t i2c_commit(i2c_cmd_handle_t cmd, uint64_t timeout);

i2c_cmd_handle_t i2c_commit_and_create(i2c_cmd_handle_t cmd, uint64_t timeout, esp_err_t *err);

esp_err_t i2c_write_register(uint8_t addr, uint8_t reg, uint8_t data);

esp_err_t i2c_read_register(uint8_t addr, uint8_t reg, uint8_t *data, size_t data_length);


#endif //RTOS_SDK_LAB1_I2C_UTILS_H
