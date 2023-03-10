//
// Created by Lazaro O'Farrill on 09/03/2023.
//
#include "imu.h"

#ifdef MPU_6050

#include "driver/i2c.h"
#include "esp_log.h"
#include "stdint.h"
#include "i2c_utils.h"
#include "stdint.h"

//Registers
#define FIFO 0x23
#define ACCEL_XOUT_MS 0x3B
#define ACCEL_XOUT_LS 0x3C
#define ACCEL_YOUT_MS 0x3D
#define ACCEL_YOUT_LS 0x3E
#define ACCEL_ZOUT_MS 0x3F
#define ACCEL_ZOUT_LS 0x40


const uint8_t ACCEL_REGISTERS[] = {ACCEL_XOUT_MS,
                                   ACCEL_XOUT_LS,
                                   ACCEL_YOUT_MS,
                                   ACCEL_YOUT_LS,
                                   ACCEL_ZOUT_MS,
                                   ACCEL_ZOUT_LS};
const uint8_t ACCEL_REGISTERS_LENGTH = sizeof(ACCEL_REGISTERS) / sizeof(ACCEL_REGISTERS[0]);


static const char *TAG = "MPU6050";
MPU6050_T MPU6050 = {
        .address = 0x01,
        .accelX =  0,
        .accelY = 0,
        .accelZ = 0
};

const uint8_t available_addresses[] = {0x68, 0x69};

esp_err_t mpuReady = ESP_FAIL;

esp_err_t initMpu(uint8_t address) {
    esp_err_t err = ESP_OK;

    if (address == 0x68 || address == 0x69) {
        MPU6050.address = address;
    } else {
        err = ESP_FAIL;
    }

    err = i2c_ping(address);


    return err;
}

esp_err_t enableMpuFifo() {

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, FIFO, 1);
    i2c_master_write_byte(cmd, 0xFF, 1);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);


    return err;
}

esp_err_t readLastAcc() {
    const uint8_t readings[ACCEL_REGISTERS_LENGTH];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, ACCEL_REGISTERS[0], 1);
    i2c_master_stop(cmd);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050.address << 1) | I2C_MASTER_READ, 1);
    i2c_master_read(cmd, (uint8_t *) readings, ACCEL_REGISTERS_LENGTH, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        MPU6050.accelX = (readings[0] << 8) | readings[1];
        MPU6050.accelY = (readings[1] << 8) | readings[2];
        MPU6050.accelY = (readings[1] << 8) | readings[2];
    }

    return err;
}

esp_err_t printLastAcc() {
    esp_err_t err = ESP_FAIL;
    err = readLastAcc();
    printf("%10d%10d%10d", MPU6050.accelX, MPU6050.accelY, MPU6050.accelZ);

    return err;
}

#endif