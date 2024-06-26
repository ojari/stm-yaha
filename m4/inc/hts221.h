#pragma once
#include "hal.h"

/**
 * @brief Structure to hold the status of the STATUS_REG register.
 *
 * This structure holds the status of the H_DA (Humidity data available) and T_DA (Temperature data available) bits from the STATUS_REG register.
 */
typedef struct {
    uint8_t h_da; // Humidity data available
    uint8_t t_da; // Temperature data available
} StatusReg;

typedef struct {
    I2C_Device i2c; // Device address

    float tempSlope;
    float tempIntercept;
    float humSlope;
    float humIntercept;
} HTS221_device;

void hts221_begin(HTS221_device* device);
float hts221_read_humidity(HTS221_device* device);
float hts221_read_temperature(HTS221_device* device);
void hts221_set_CTRL_REG1(HTS221_device* device, uint8_t pd, uint8_t bdu, uint8_t odr);
void hts221_set_CTRL_REG2(HTS221_device* device, uint8_t boot, uint8_t heater, uint8_t one_shot);
void hts221_set_CTRL_REG3(HTS221_device* device, uint8_t drdy_h_l, uint8_t pp_od, uint8_t drdy_en);
void hts221_read_STATUS_REG(HTS221_device* device, StatusReg *status);
int hts221_read_ident(HTS221_device* device);
void hts221_read_calibration(HTS221_device* device);
void hts221_print_registers(HTS221_device* device);
