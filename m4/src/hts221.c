#include <stdint.h>
#include "hts221.h"

// Register addresses
#define WHO_AM_I_ADDR     0x0F
#define AV_CONF_ADDR      0x10
#define CTRL_REG1_ADDR    0x20
#define CTRL_REG2_ADDR    0x21
#define CTRL_REG3_ADDR    0x22
#define STATUS_REG_ADDR   0x27
#define HUMIDITY_OUT_L_ADDR 0x28
#define HUMIDITY_OUT_H_ADDR 0x29
#define TEMP_OUT_L_ADDR   0x2A
#define TEMP_OUT_H_ADDR   0x2B
#define CALIB_0_ADDR      0x30

// CTRL_REG1_ADDR bits
//
#define PD_BIT  (1 << 7)
#define BDU_BIT (1 << 2)
// Output data rate (ODR) settings
#define ODR_ONESHOT 0x00
#define ODR_1HZ     0x01
#define ODR_7HZ     0x02
#define ODR_12_5HZ  0x03

// CTRL_REG2_ADDR bits
//
#define BOOT_BIT  (1 << 7)
#define HEATER_BIT (1 << 1)
#define ONE_SHOT_BIT 0x01

// CTRL_REG3_ADDR bits
//
#define DRDY_H_L_BIT  (1 << 7)
#define PP_OD_BIT (1 << 6)
#define DRDY_EN_BIT (1 << 2)

// STATUS_REG bits
//
#define H_DA_BIT 0x02  // Bit for Humidity data available
#define T_DA_BIT 0x01  // Bit for Temperature data available


// Default register values
#define WHO_AM_I_DEFAULT  0xBC
#define AV_CONF_DEFAULT   0x1B
#define CTRL_REG1_DEFAULT 0x00
#define CTRL_REG2_DEFAULT 0x00
#define CTRL_REG3_DEFAULT 0x00
#define STATUS_REG_DEFAULT 0x00


// Read a register value
static uint8_t read_register(HTS221_device* device, uint8_t reg_addr) {
    uint8_t value;
    I2C_read(&(device->i2c), reg_addr, &value, 1);
    return value;
}

// Write a register value
static void write_register(HTS221_device* device, uint8_t reg_addr, uint8_t value) {
    I2C_write(&(device->i2c), reg_addr, &value, 1);
}

// Initialize the HTS221 sensor
void hts221_begin(HTS221_device* device) {
    I2C_begin(&(device->i2c), 0xBE, 0xBF);
    // Check the WHO_AM_I register to verify the sensor's identity
    uint8_t who_am_i = read_register(device, WHO_AM_I_ADDR);
    if (who_am_i != WHO_AM_I_DEFAULT) {
        // Handle error, sensor not found or incorrect
    }

    // Configure the AV_CONF register
    write_register(device, AV_CONF_ADDR, AV_CONF_DEFAULT);

    // Configure the CTRL_REG1 register
    write_register(device, CTRL_REG1_ADDR, CTRL_REG1_DEFAULT);

    // Configure the CTRL_REG2 register
    write_register(device, CTRL_REG2_ADDR, CTRL_REG2_DEFAULT);

    // Configure the CTRL_REG3 register
    write_register(device, CTRL_REG3_ADDR, CTRL_REG3_DEFAULT);
}

int hts221_read_ident(HTS221_device* device) {
    return read_register(device, WHO_AM_I_ADDR);
}

// Read the humidity value from the sensor
float hts221_read_humidity(HTS221_device* device) {
    // Read the humidity registers
    uint8_t humidity_l = read_register(device, HUMIDITY_OUT_L_ADDR);
    uint8_t humidity_h = read_register(device, HUMIDITY_OUT_H_ADDR);

    // Combine the low and high bytes to get the humidity value
    uint16_t humidity_raw = (humidity_h << 8) | humidity_l;

    // Convert the raw value to a percentage
    float humidity = humidity_raw * 0.00610;

    return humidity;
}

// Read the temperature value from the sensor
float hts221_read_temperature(HTS221_device* device) {
    // Read the temperature registers
    uint8_t temp_l = read_register(device, TEMP_OUT_L_ADDR);
    uint8_t temp_h = read_register(device, TEMP_OUT_H_ADDR);

    // Combine the low and high bytes to get the temperature value
    uint16_t temp_raw = (temp_h << 8) | temp_l;

    // Convert the raw value to degrees Celsius
    float temperature = temp_raw * 0.00781;

    return temperature;
}

/**
 * @brief Set the CTRL_REG1 register.
 *
 * This function sets the PD (Power Down control), BDU (Block Data Update), and ODR (Output Data Rate) bits in the CTRL_REG1 register.
 *
 * @param pd Power Down control.
 *           0: power-down mode
 *           1: active mode
 *
 * @param bdu Block Data Update.
 *            0: continuous update
 *            1: output registers not updated until MSB and LSB reading
 *
 * @param odr Output Data Rate.
 *            00: one shot
 *            01: 1 Hz
 *            10: 7 Hz
 *            11: 12.5 Hz
 *
 * @return None.
 */
void hts221_set_CTRL_REG1(HTS221_device* device, uint8_t pd, uint8_t bdu, uint8_t odr) {
    uint8_t reg_value = 0;

    // Set the PD bit
    if (pd) {
        reg_value |= PD_BIT;
    }

    // Set the BDU bit
    if (bdu) {
        reg_value |= BDU_BIT;
    }

    // Set the ODR bits
    reg_value |= (odr & 0x03); // Only the last two bits are valid

    // Write the register value
    write_register(device, CTRL_REG1_ADDR, reg_value);
}


/**
 * @brief Set the CTRL_REG2 register.
 *
 * This function sets the BOOT, Heater, and One-shot bits in the CTRL_REG2 register.
 *
 * @param boot Reboot memory content.
 *             0: normal mode
 *             1: reboot memory content
 *
 * @param heater Heater control.
 *               0: heater disable
 *               1: heater enable
 *
 * @param one_shot One-shot enable.
 *                 0: waiting for start of conversion
 *                 1: start for a new dataset
 *
 * @return None.
 */
void hts221_set_CTRL_REG2(HTS221_device* device, uint8_t boot, uint8_t heater, uint8_t one_shot) {
    uint8_t reg_value = 0;

    // Set the BOOT bit
    if (boot) {
        reg_value |= BOOT_BIT;
    }

    // Set the Heater bit
    if (heater) {
        reg_value |= HEATER_BIT;
    }

    // Set the One-shot bit
    if (one_shot) {
        reg_value |= ONE_SHOT_BIT;
    }

    // Write the register value
    write_register(device, CTRL_REG2_ADDR, reg_value);
}


/**
 * @brief Set the CTRL_REG3 register.
 *
 * This function sets the DRDY_H_L, PP_OD, and DRDY_EN bits in the CTRL_REG3 register.
 *
 * @param drdy_h_l Data Ready output signal active high, low.
 *                 0: active high - default
 *                 1: active low
 *
 * @param pp_od Push-pull / Open Drain selection on pin 3 (DRDY).
 *              0: push-pull - default
 *              1: open drain
 *
 * @param drdy_en Data Ready enable.
 *                0: Data Ready disabled - default
 *                1: Data Ready signal available on pin 3
 *
 * @return None.
 */
void hts221_set_CTRL_REG3(HTS221_device* device, uint8_t drdy_h_l, uint8_t pp_od, uint8_t drdy_en) {
    uint8_t reg_value = 0;

    // Set the DRDY_H_L bit
    if (drdy_h_l) {
        reg_value |= DRDY_H_L_BIT;
    }

    // Set the PP_OD bit
    if (pp_od) {
        reg_value |= PP_OD_BIT;
    }

    // Set the DRDY_EN bit
    if (drdy_en) {
        reg_value |= DRDY_EN_BIT;
    }

    // Write the register value
    write_register(device, CTRL_REG3_ADDR, reg_value);
}

/**
 * @brief Read the STATUS_REG register.
 *
 * This function reads the H_DA (Humidity data available) and T_DA (Temperature data available) bits from the STATUS_REG register.
 *
 * @return A struct containing two fields:
 *         - h_da: Humidity data available.
 *               0: new data for humidity is not yet available
 *               1: new data for humidity is available
 *         - t_da: Temperature data available.
 *               0: new data for temperature is not yet available
 *               1: new data for temperature is available
 */
void hts221_read_STATUS_REG(HTS221_device* device, StatusReg *status) {
    uint8_t reg_value;

    // Read the register value
    reg_value = read_register(device, STATUS_REG_ADDR);

     // Extract the H_DA bit
    status->h_da = (reg_value & H_DA_BIT) ? 1 : 0;

    // Extract the T_DA bit
    status->t_da = (reg_value & T_DA_BIT) ? 1 : 0;
}

/**
 * @brief Read the calibration data from the sensor.
 *
 * This function reads the calibration data from the sensor's calibration registers.
 *
 * @param hum_cal_data A pointer to a HumidityCalibrationData struct to be filled with the humidity calibration data.
 * @param temp_cal_data A pointer to a TemperatureCalibrationData struct to be filled with the temperature calibration data.
 */
void hts221_read_calibration(HTS221_device* device, CalibrationData* hum_cal_data, CalibrationData* temp_cal_data) {
    // Read the calibration data from the sensor's calibration registers
    hum_cal_data->cal0 = read_register(device, 0x30);
    hum_cal_data->cal1 = read_register(device, 0x31);
    hum_cal_data->out0 = read_register(device, 0x36) | (read_register(device, 0x37) << 8);
    hum_cal_data->out1 = read_register(device, 0x3A) | (read_register(device, 0x3B) << 8);

    temp_cal_data->cal0 = read_register(device, 0x32) | ((read_register(device, 0x35) & 0x03) << 8);
    temp_cal_data->cal1 = read_register(device, 0x33) | ((read_register(device, 0x35) & 0x0C) << 6);
    temp_cal_data->out0 = read_register(device, 0x3C) | (read_register(device, 0x3D) << 8);
    temp_cal_data->out1 = read_register(device, 0x3E) | (read_register(device, 0x3F) << 8);
}


static void hts221_print_ctrl_reg(HTS221_device* device, uint8_t reg_addr, const char* reg_name) {
    uint8_t reg_value = read_register(device, reg_addr);

    Serial_print(reg_name);
    Serial_print(":\n");

    if (reg_addr == CTRL_REG1_ADDR) {
        Serial_print("  PD:       ");
        Serial_printi((reg_value & PD_BIT) == PD_BIT);
        Serial_print("\n");

        Serial_print("  BDU:      ");
        Serial_printi((reg_value & BDU_BIT) == BDU_BIT);
        Serial_print("\n");

        Serial_print("  ODR:      ");
        Serial_printi(reg_value & 0x03);
        Serial_print("\n");
    } else if (reg_addr == CTRL_REG2_ADDR) {
        Serial_print("  BOOT:     ");
        Serial_printi((reg_value & BOOT_BIT) == BOOT_BIT);
        Serial_print("\n");

        Serial_print("  HEATER:   ");
        Serial_printi((reg_value & HEATER_BIT) == HEATER_BIT);
        Serial_print("\n");

        Serial_print(" ONE_SHOT:  ");
        Serial_printi((reg_value & ONE_SHOT_BIT) == ONE_SHOT_BIT);
        Serial_print("\n");
    } else if (reg_addr == CTRL_REG3_ADDR) {
        Serial_print("  DRDY_H_L: ");
        Serial_printi((reg_value & DRDY_H_L_BIT) == DRDY_H_L_BIT);
        Serial_print("\n");

        Serial_print("  PP_OD:    ");
        Serial_printi((reg_value & PP_OD_BIT) == PP_OD_BIT);
        Serial_print("\n");

        Serial_print("  DRDY_EN:  ");
        Serial_printi((reg_value & DRDY_EN_BIT) == DRDY_EN_BIT);
        Serial_print("\n");
    }
}

void hts221_print_registers(HTS221_device* device) {
    hts221_print_ctrl_reg(device, CTRL_REG1_ADDR, "CTRL_REG1");
    hts221_print_ctrl_reg(device, CTRL_REG2_ADDR, "CTRL_REG2");
    hts221_print_ctrl_reg(device, CTRL_REG3_ADDR, "CTRL_REG3");
}