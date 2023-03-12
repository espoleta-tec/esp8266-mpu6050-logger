//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#include "utils.h"


uint8_t utils_bit_mask(uint8_t value, uint8_t mask, esp_err_t *err) {
    uint8_t maskStart = mask >> 4;
    uint8_t maskEnd = mask & 0xf;
    if (maskStart > 7 || maskEnd > 7 || maskStart < maskEnd) {
        *err = ESP_ERR_INVALID_ARG;
        return 0;
    }

    uint8_t maskBitMap = 0x00;
    for (int i = maskEnd; i <= maskStart; ++i) {
        maskBitMap = maskBitMap | (1 << maskEnd);
    }
    return value & maskBitMap;
}