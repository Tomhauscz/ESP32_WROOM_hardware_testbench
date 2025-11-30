// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Modified by Tomas Hak to support ESP-IDF I2C driver v2

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <unistd.h>
#include <malloc.h>

#include "esp32/rom/ets_sys.h"
#include <i2cdev.h>

static const char __attribute__((unused)) TAG_VL53L0X[] = "[TOF sensor]";

typedef struct vl53l0x_s vl53l0x_t;

typedef enum { 
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vl53l0x_vcselPeriodType;

typedef enum {
    Device_unresponsive,                // Device unresponsive (init state)
    CAL_timeout,                        // Single ref calibration timeout
    Bad_rate,                           // Signal rate limit incorrect value
    SPAD_timeout,                       // Single Photon Avalanche Diode (SPAD) read timeout
    Low_budget,                         // Measurement timing budget too low (min 20000us)
    High_budget,                        // Measurement timing budget too high
    VCSEL_prerange_period_invalid,      // Vertical Cavity Surface Emitting Laser (VCSEL) pulse period pre-range invalid
    VCSEL_finalrange_period_invalid,    // Vertical Cavity Surface Emitting Laser (VCSEL) pulse period final-range invalid
    Invalid_type,                       // VCSEL period type invalid
    Interrupt_status_timeout,           // Reading interrupt status register timeout
    Start_bit_clearing_timeout,         // Start bit clearing timeout

    Errors_count

} vl53l0x_error_t;

static char vl53l0x_error_messages_lookup_table[Errors_count][120] = {
    "Device unresponsive",                                                              // Device_unresponsive
    "Single ref calibration timeout",                                                   // CAL_timeout
    "Signal rate limit incorrect value",                                                // Bad_rate
    "Single Photon Avalanche Diode (SPAD) read timeout",                                // SPAD_timeout
    "Measurement timing budget too low (min 20000us)",                                  // Low_budget
    "Measurement timing budget too high",                                               // High_budget
    "Vertical Cavity Surface Emitting Laser (VCSEL) pulse period pre-range invalid",    // VCSEL_prerange_period_invalid
    "Vertical Cavity Surface Emitting Laser (VCSEL) pulse period final-range invalid",  // VCSEL_finalrange_period_invalid
    "VCSEL period type invalid",                                                        // Invalid_type
    "Reading interrupt status register timeout",                                        // Interrupt_status_timeout
    "Start bit clearing timeout"                                                        // Start_bit_clearing_timeout
};

esp_err_t vl53l0x_setSignalRateLimit(vl53l0x_t *, float limit_Mcps);
esp_err_t vl53l0x_getSignalRateLimit(i2c_dev_t *, float *sig_rate_lim);

esp_err_t vl53l0x_getSpadInfo(vl53l0x_t *, uint8_t *count, int *type_is_aperture);

esp_err_t vl53l0x_setMeasurementTimingBudget(vl53l0x_t *, uint32_t budget_us);
esp_err_t vl53l0x_getMeasurementTimingBudget(i2c_dev_t *, uint32_t *budget_us);

esp_err_t vl53l0x_setVcselPulsePeriod(vl53l0x_t *, vl53l0x_vcselPeriodType type, uint8_t period_pclks);
esp_err_t vl53l0x_getVcselPulsePeriod(i2c_dev_t *, vl53l0x_vcselPeriodType type, uint16_t *VcselPeriod);

esp_err_t vl53l0x_startContinuous(vl53l0x_t *, uint32_t period_ms);
esp_err_t vl53l0x_stopContinuous(vl53l0x_t *);
esp_err_t vl53l0x_readRangeContinuousMillimeters(vl53l0x_t *, uint16_t *range_value);
esp_err_t vl53l0x_readRangeSingleMillimeters(vl53l0x_t *, uint16_t *range_value);

// Set up I2C and create the vl53l0x structure, NULL means could not see device on I2C
vl53l0x_t *vl53l0x_config(i2c_dev_t *, gpio_num_t xshut_pin, uint8_t io_2v8);
// Initialise the VL53L0X
esp_err_t vl53l0x_init(vl53l0x_t *);

vl53l0x_error_t vl53l0x_getError(vl53l0x_t *);
char *vl53l0x_err_to_name(vl53l0x_error_t err_code);

#endif
