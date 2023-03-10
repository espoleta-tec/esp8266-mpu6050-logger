//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#include "imu.h"

#ifdef ICM_20948

#include "esp_err.h"
#include "i2c_utils.h"
#include "driver/i2c.h"
#include "freertos/task.h"

//Registers
//Bank_0
#define WHO_AM_I 0x00
#define USER_CTRL 0x03
#define LP_CONFIG 0x05
#define PWR_MGMT_1 0x06
#define PWR_MGMT_2 0x07
#define INT_PIN_CFG 0x0F
#define INT_ENABLE 0x10
#define INT_ENABLE_1 0x11
#define INT_ENABLE_2 0x12
#define INT_ENABLE_3 0x13
#define I2C_MST_STATUS 0x17
#define INT_STATUS 0x19
#define INT_STATUS_1 0x1A
#define INT_STATUS_2 0x1B
#define INT_STATUS_3 0x1C
#define DELAY_TIMEH 0x28
#define DELAY_TIMEL 0x29
#define ACCEL_XOUT_H 0x2D
#define ACCEL_XOUT_L 0x2E
#define ACCEL_YOUT_H 0x2F
#define ACCEL_YOUT_L 0x30
#define ACCEL_ZOUT_H 0x31
#define ACCEL_ZOUT_L 0x32
#define GYRO_XOUT_H 0x33
#define GYRO_XOUT_L 0x34
#define GYRO_YOUT_H 0x35
#define GYRO_YOUT_L 0x36
#define GYRO_ZOUT_H 0x37
#define GYRO_ZOUT_L 0x38
#define TEMP_OUT_H 0x39
#define TEMP_OUT_L 0x3A
#define EXT_SLV_SENS_DATA_00 0x3B
#define EXT_SLV_SENS_DATA_01 0x3C
#define EXT_SLV_SENS_DATA_02 0x3D
#define EXT_SLV_SENS_DATA_03 0x3E
#define EXT_SLV_SENS_DATA_04 0x3F
#define EXT_SLV_SENS_DATA_05 0x40
#define EXT_SLV_SENS_DATA_06 0x41
#define EXT_SLV_SENS_DATA_07 0x42
#define EXT_SLV_SENS_DATA_08 0x43
#define EXT_SLV_SENS_DATA_09 0x44
#define EXT_SLV_SENS_DATA_10 0x45
#define EXT_SLV_SENS_DATA_11 0x46
#define EXT_SLV_SENS_DATA_12 0x47
#define EXT_SLV_SENS_DATA_13 0x48
#define EXT_SLV_SENS_DATA_14 0x49
#define EXT_SLV_SENS_DATA_15 0x4A
#define EXT_SLV_SENS_DATA_16 0x4B
#define EXT_SLV_SENS_DATA_17 0x4C
#define EXT_SLV_SENS_DATA_18 0x4D
#define EXT_SLV_SENS_DATA_19 0x4E
#define EXT_SLV_SENS_DATA_20 0x4F
#define EXT_SLV_SENS_DATA_21 0x50
#define EXT_SLV_SENS_DATA_22 0x51
#define EXT_SLV_SENS_DATA_23 0x52

//Common
#define FIFO_EN_1 0x66

#define REG_BANK_SEL 0x7F

//declarations
static esp_err_t selectBank(uint8_t bank);

static esp_err_t wakeUp();

static esp_err_t reset();

//definitions

esp_err_t initImu(uint8_t address) {
    esp_err_t err = ESP_OK;

    if (address != 0x68 && address != 0x69) {
        return ESP_ERR_INVALID_ARG;
    }

    err = i2c_ping(address);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    IMU.address = address;

    err = reset();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    vTaskDelay(10 / portTICK_RATE_MS);

    err = wakeUp();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t readLastImuValues() {
    esp_err_t err;

    const uint8_t readings[12] = {0};

    //select user bank 0
    err = selectBank(0);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = i2c_read_register(IMU.address, ACCEL_XOUT_H, (uint8_t *) readings, 12);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }


    IMU.accX = (readings[0] << 8) | readings[1];
    IMU.accY = (readings[2] << 8) | readings[3];
    IMU.accZ = (readings[4] << 8) | readings[5];
    IMU.gyX = (readings[6] << 8) | readings[7];
    IMU.gyY = (readings[8] << 8) | readings[9];
    IMU.gyZ = (readings[10] << 8) | readings[11];

    return err;
}

esp_err_t selectBank(uint8_t bank) {
    if (bank > 3) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //Move head
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, REG_BANK_SEL, 1);
    i2c_master_stop(cmd);
    cmd = i2c_commit_and_create(cmd, 10, &err);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Read REG_BANK_SEL
    uint8_t regBankValue = 0x00;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, &regBankValue, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    cmd = i2c_commit_and_create(cmd, 100, &err);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }


    //Write user bank
    regBankValue = (regBankValue & 0xcf);
    uint8_t newUserBankValue = (bank << 4) | regBankValue;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, REG_BANK_SEL, 1);
    i2c_master_write_byte(cmd, newUserBankValue, 1);
    i2c_master_stop(cmd);

    //Commit
    err = i2c_commit(cmd, 10);

    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t wakeUp() {
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    //Move head
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, PWR_MGMT_1 | I2C_MASTER_WRITE, 1);
    i2c_master_stop(cmd);

    cmd = i2c_commit_and_create(cmd, 10, &err);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Read register
    uint8_t powerManagementReg = 0;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_READ, 1);
    i2c_master_read_byte(cmd, &powerManagementReg, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    cmd = i2c_commit_and_create(cmd, 10, &err);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    printf("reggie is: %x", powerManagementReg);
    powerManagementReg = powerManagementReg & 0xbf;
    printf("reggie will be: %x", powerManagementReg);
    fflush(stdout);



    //Write new register value
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (IMU.address << 1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, PWR_MGMT_1, 1);
    i2c_master_write_byte(cmd, powerManagementReg, 1);
    i2c_master_stop(cmd);

    err = i2c_commit(cmd, 10);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t reset() {
    esp_err_t err = i2c_write_register(IMU.address, PWR_MGMT_1, 0x80);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }


    vTaskDelay(1 / portTICK_RATE_MS);

    return ESP_OK;
}

#endif