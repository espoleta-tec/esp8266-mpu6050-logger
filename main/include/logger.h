//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#ifndef RTOS_SDK_LAB1_LOGGER_H
#define RTOS_SDK_LAB1_LOGGER_H

void logger_print_status_header();

void logger_print_status(const char *label, const char *status);

void logger_print_statusf(const char *label, const char *statusFormat, ...);
#endif //RTOS_SDK_LAB1_LOGGER_H
