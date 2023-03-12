//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#include "imu.h"

#ifdef ICM_20948

#include "esp_err.h"
#include "i2c_utils.h"
#include "driver/i2c.h"
#include "freertos/task.h"
#include "logger.h"
#include "utils.h"

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

//User Bank 1
#define SELF_TEST_X_GYRO 0x02
#define SELF_TEST_Y_GYRO 0x03
#define SELF_TEST_Z_GYRO 0x04
#define SELF_TEST_X_ACCEL 0x0E
#define SELF_TEST_Y_ACCEL 0x0F
#define SELF_TEST_Z_ACCEL 0x10


//User Bank 2
#define GYRO_SMPLRT_DIV 0x00
#define GYRO_CONFIG_1 0x01
#define GYRO_CONFIG_2 0x02
#define ACCEL_CONFIG 0x14
#define ACCEL_CONFIG_2 0x15


//Common
#define FIFO_EN_1 0x66

#define REG_BANK_SEL 0x7F

//declarations
static esp_err_t selectBank(uint8_t bank);

static esp_err_t wakeUp();

static esp_err_t reset();

static esp_err_t config_accel();

static esp_err_t config_gyro();

static esp_err_t config_mag();

static esp_err_t self_test();

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

    vTaskDelay(1000 / portTICK_RATE_MS);

    err = wakeUp();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = config_gyro();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = config_accel();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = self_test();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}

esp_err_t imu_read_sensors() {
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


    IMU.accX = (((int16_t) readings[0]) << 8) | readings[1];
    IMU.accY = (((int16_t) readings[2]) << 8) | readings[3];
    IMU.accZ = (((int16_t) readings[4]) << 8) | readings[5];
    IMU.gyX = (((int16_t) readings[6]) << 8) | readings[7];
    IMU.gyY = (((int16_t) readings[8]) << 8) | readings[9];
    IMU.gyZ = (((int16_t) readings[10]) << 8) | readings[11];

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
    uint8_t powerManagementReg = 0;

    err = i2c_read_register(IMU.address, PWR_MGMT_1, &powerManagementReg, 1);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    powerManagementReg = powerManagementReg & 0xbf;

    err = i2c_write_register(IMU.address, PWR_MGMT_1, powerManagementReg);
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

typedef enum {
    ACCEL_FS_SEL_2G = 0 << 1,
    ACCEL_FS_SEL_4G = 1 << 1,
    ACCEL_FS_SEL_8G = 2 << 1,
    ACCEL_FS_SEL_16G = 3 << 1,
} accel_fs_sel;

typedef enum {
    ACCEL_FCHOICE_DISABLE,
    ACCEL_FCHOICE_ENABLE
} accel_fchoice;

typedef enum {
    ACCEL_DLPFCFG_0 = 0 << 3,
    ACCEL_DLPFCFG_1 = 1 << 3,
    ACCEL_DLPFCFG_2 = 2 << 3,
    ACCEL_DLPFCFG_3 = 3 << 3,
    ACCEL_DLPFCFG_4 = 4 << 3,
    ACCEL_DLPFCFG_5 = 5 << 3,
    ACCEL_DLPFCFG_6 = 6 << 3,
    ACCEL_DLPFCFG_7 = 7 << 3,
};

esp_err_t config_accel() {
    esp_err_t err;

    uint8_t accelConfigReg;

    err = i2c_read_register(IMU.address, ACCEL_CONFIG, &accelConfigReg, 1);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    accelConfigReg = utils_bit_mask(accelConfigReg, 0x76, &err);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    accelConfigReg = accelConfigReg | ACCEL_DLPFCFG_0 | ACCEL_FS_SEL_2G | ACCEL_FCHOICE_ENABLE;

    err = i2c_write_register(IMU.address, ACCEL_CONFIG, accelConfigReg);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}


typedef enum {
    GYRO_F_DISABLE, //Disable low pass filter
    GYRO_F_ENABLE //Enable low pass filter
} gyro_fchoice;

typedef enum {
    GYRO_FS_250 = 0 << 1,
    GYRO_FS_500 = 1 << 1,
    GYRO_FS_1000 = 2 << 1,
    GYRO_FS_2000 = 3 << 1,
} gyro_fs_sel;

typedef enum {
    GYRO_DLPFCFG_0 = 0 << 3,
    GYRO_DLPFCFG_1 = 1 << 3,
    GYRO_DLPFCFG_2 = 2 << 3,
    GYRO_DLPFCFG_3 = 2 << 3,
    GYRO_DLPFCFG_4 = 4 << 3,
    GYRO_DLPFCFG_5 = 5 << 3,
    GYRO_DLPFCFG_6 = 6 << 3,
    GYRO_DLPFCFG_7 = 7 << 3,
} gyro_dlpfcfg;

esp_err_t config_gyro() {
    esp_err_t err;
    uint8_t gyroConfig1Reg;


    err = selectBank(2);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = i2c_read_register(IMU.address, GYRO_CONFIG_1, &gyroConfig1Reg, 1);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    gyroConfig1Reg = gyroConfig1Reg & 0x80;
    gyroConfig1Reg = gyroConfig1Reg | GYRO_DLPFCFG_1 | GYRO_FS_1000 | GYRO_F_ENABLE;

    err = i2c_write_register(IMU.address, GYRO_CONFIG_1, gyroConfig1Reg);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    logger_print_statusf("Gyro configuration", "%d", gyroConfig1Reg);


    return err;
}

esp_err_t config_mag() {
    return 0;
}

typedef enum {
    AX_ST_EN_REG_DISABLE = 0 << 4,
    AX_ST_EN_REG_ENABLE = 1 << 4,
} ax_st_en_reg;

typedef enum {
    AY_ST_EN_REG_DISABLE = 0 << 3,
    AY_ST_EN_REG_ENABLE = 1 << 3,
} ay_st_en_reg;

typedef enum {
    AZ_ST_EN_REG_DISABLE = 0 << 2,
    AZ_ST_EN_REG_ENABLE = 1 << 2,
} az_st_en_reg;

typedef enum {
    DEC3_CFG_1_4,
    DEC3_CFG_8,
    DEC3_CFG_16,
    DEC3_CFG_32,
} dec3_cfg;

esp_err_t self_test() {
    esp_err_t err;
    uint8_t accelConfig2Register;

    err = selectBank(2);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = i2c_read_register(IMU.address, ACCEL_CONFIG_2, &accelConfig2Register, 1);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    accelConfig2Register = utils_bit_mask(accelConfig2Register, 0x75, &err);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Enable self test
    accelConfig2Register = accelConfig2Register | AX_ST_EN_REG_ENABLE | AY_ST_EN_REG_ENABLE | AZ_ST_EN_REG_ENABLE;
    err = i2c_write_register(IMU.address, ACCEL_CONFIG_2, accelConfig2Register);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //read with self test enabled
    err = imu_read_sensors();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    uint16_t testX = IMU.accX, testY = IMU.accY, testZ = IMU.accZ;

    accelConfig2Register = utils_bit_mask(accelConfig2Register, 0x76, &err);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //disable self test
    err = i2c_write_register(IMU.address, ACCEL_CONFIG_2, accelConfig2Register);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Read with self test disabled
    err = imu_read_sensors();
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    //Read self test references
    err = selectBank(1);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    uint8_t selfTestGyroRef[3], selfTestAccelRef[3];
    err = i2c_read_register(IMU.address, SELF_TEST_X_GYRO, selfTestGyroRef, 3);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    err = i2c_read_register(IMU.address, SELF_TEST_X_ACCEL, selfTestAccelRef, 3);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    logger_print_statusf("Self Test Response X", "%d --- Expected: %d",
                         testX - IMU.accX, selfTestAccelRef[0]);
    logger_print_statusf("Self Test Response Y", "%d --- Expected: %d",
                         testY - IMU.accY, selfTestAccelRef[1]);
    logger_print_statusf("Self Test Response Z", "%d --- Expected: %d",
                         testZ - IMU.accZ, selfTestAccelRef[2]);


    return err;
}

#endif