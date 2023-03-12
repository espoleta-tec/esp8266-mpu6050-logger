//
// Created by Lazaro O'Farrill on 09/03/2023.
//

#include "logger.h"
#include "stdio.h"
#include "stdarg.h"

static void printStatusDivider();

void printStatusDivider() {
    char divider[54];
    for (int i = 0; i < 53; i++) {
        divider[i] = '_';
    }
    divider[53] = '\0';

    puts(divider);
}

void logger_print_status_header() {
    logger_print_status("Parameter", "Status");
}


void logger_print_status(const char *label, const char *status) {
    printf("|%30s|%20s|\n", label, status);
}

void logger_print_statusf(const char *label, const char *statusFormat, ...) {
    printf("|%30s|", label);
    va_list args;
    va_start(args, statusFormat);
    vprintf(statusFormat, args);
    va_end(args);
    printf("|\n");
}


