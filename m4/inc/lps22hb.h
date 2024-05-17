#pragma once
#include "hal.h"

// Device structure
typedef struct {
    I2C_Device i2c; // Device address
} LPS22HB_device;

void lps22hb_begin(LPS22HB_device* device);
int16_t lps22hb_read_temperature(LPS22HB_device* device);
int32_t lps22hb_read_pressure(LPS22HB_device* device);
