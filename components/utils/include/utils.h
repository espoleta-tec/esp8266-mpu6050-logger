//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#ifndef RTOS_SDK_LAB1_UTILS_H
#define RTOS_SDK_LAB1_UTILS_H

#include "esp_err.h"

/*
 * @param value: Value to mask
 * @param mask: 8 bit number MSB marks mask start and LSB mask end.
 */
uint8_t utils_bit_mask(uint8_t value, uint8_t mask, esp_err_t *err);

#endif //RTOS_SDK_LAB1_UTILS_H
