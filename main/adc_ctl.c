//
// Created by Lazaro O'Farrill on 12/03/2023.
//

#include <esp_err.h>
#include <driver/adc.h>
#include "adc_ctl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_read_ctl.h"

const char *READ_ADC_TASK_LABEL = "ADC READ";

void task_read_adc(void *ignore) {
    esp_err_t err;

    while (1) {
        uint16_t adc_value_read;
        err = adc_read(&adc_value_read);
        if (err != ESP_OK) {
            ESP_ERROR_CHECK_WITHOUT_ABORT(err);
            return;
        }

        if (adc_value_read > 100) {
            toggle_reads();
            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

esp_err_t adc_config() {
    esp_err_t err;

    //Init ADC
    adc_config_t adcConfig;
    adcConfig.mode = ADC_READ_TOUT_MODE;
    adcConfig.clk_div = 8;
    err = adc_init(&adcConfig);
    if (err != ESP_OK) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(err);
        return err;
    }

    return err;
}
