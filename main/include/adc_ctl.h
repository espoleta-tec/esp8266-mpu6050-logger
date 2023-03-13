//
// Created by Lazaro O'Farrill on 12/03/2023.
//

#ifndef RTOS_SDK_LAB1_ADC_CTL_H
#define RTOS_SDK_LAB1_ADC_CTL_H
esp_err_t adc_config();

extern const char *READ_ADC_TASK_LABEL;
void task_read_adc(void *ignore);
#endif //RTOS_SDK_LAB1_ADC_CTL_H
