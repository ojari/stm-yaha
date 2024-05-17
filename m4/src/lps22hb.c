#include "lps22hb.h"

// Register addresses
#define INTERRUPT_CFG   0x0B
#define THS_P_L         0x0C
#define THS_P_H         0x0D
#define WHO_AM_I        0x0F
#define CTRL_REG1       0x10
#define CTRL_REG2       0x11
#define CTRL_REG3       0x12
#define FIFO_CTRL       0x14
#define REF_P_XL        0x15
#define REF_P_L         0x16
#define REF_P_H         0x17
#define RPDS_L          0x18
#define RPDS_H          0x19
#define RES_CONF        0x1A
#define INT_SOURCE      0x25
#define FIFO_STATUS     0x26
#define STATUS          0x27
#define PRESS_OUT_XL    0x28
#define PRESS_OUT_L     0x29
#define PRESS_OUT_H     0x2A
#define TEMP_OUT_L      0x2B
#define TEMP_OUT_H      0x2C
#define LPFP_RES        0x33

// Function to read a register
uint8_t lps22hb_read_register(LPS22HB_device* device, uint8_t reg_addr) {
    uint8_t value;
    I2C_read(&(device->i2c), reg_addr, &value, 1);
    return value;
}

// Function to write to a register
void lps22hb_write_register(LPS22HB_device* device, uint8_t reg_addr, uint8_t value) {
    I2C_write(&(device->i2c), reg_addr, &value, 1);
}

// Initialize the HTS221 sensor
void lps22hb_begin(LPS22HB_device* device) {
    I2C_begin(&(device->i2c), 0xB8, 0xB9);
    // Check the WHO_AM_I register to verify the sensor's identity
    uint8_t who_am_i = lps22hb_read_register(device, WHO_AM_I);
    if (who_am_i != 0xB1) {
        // Handle error, sensor not found or incorrect
    }
 }


// Higher-level functions would go here. For example:
int16_t lps22hb_read_temperature(LPS22HB_device* device) {
    uint8_t temp_l = lps22hb_read_register(device, TEMP_OUT_L);
    uint8_t temp_h = lps22hb_read_register(device, TEMP_OUT_H);

    // Combine the two bytes into a single 16-bit value
    int16_t temp = (temp_h << 8) | temp_l;

    // Convert the raw temperature value to degrees Celsius and return it
    return temp / 100.0;
}

// read pressure
int32_t lps22hb_read_pressure(LPS22HB_device* device) {
    uint8_t press_xl = lps22hb_read_register(device, PRESS_OUT_XL);
    uint8_t press_l = lps22hb_read_register(device, PRESS_OUT_L);
    uint8_t press_h = lps22hb_read_register(device, PRESS_OUT_H);

    // Combine the three bytes into a single 32-bit value
    int32_t pressure = (press_h << 16) | (press_l << 8) | press_xl;

    return pressure;
}

// INTERRUPT_CFG register bit positions
#define AUTORIFP_BIT 7
#define RESET_ARP_BIT 6
#define AUTOZERO_BIT 5
#define RESET_AZ_BIT 4
#define DIFF_EN_BIT 3
#define LIR_BIT 2
#define PLE_BIT 1
#define PHE_BIT 0

// INTERRUPT_CFG register bit masks
#define AUTORIFP_MASK (1 << AUTORIFP_BIT)
#define RESET_ARP_MASK (1 << RESET_ARP_BIT)
#define AUTOZERO_MASK (1 << AUTOZERO_BIT)
#define RESET_AZ_MASK (1 << RESET_AZ_BIT)
#define DIFF_EN_MASK (1 << DIFF_EN_BIT)
#define LIR_MASK (1 << LIR_BIT)
#define PLE_MASK (1 << PLE_BIT)
#define PHE_MASK (1 << PHE_BIT)

// Function to read a field from INTERRUPT_CFG
uint8_t lps22hb_read_interrupt_cfg(LPS22HB_device* device, uint8_t mask) {
    uint8_t reg_value = lps22hb_read_register(device, INTERRUPT_CFG);
    return (reg_value & mask) != 0;
}

// Function to write a field to INTERRUPT_CFG
void lps22hb_write_interrupt_cfg(LPS22HB_device* device, uint8_t mask, uint8_t value) {
    uint8_t reg_value = lps22hb_read_register(device, INTERRUPT_CFG);
    if (value) {
        reg_value |= mask;
    } else {
        reg_value &= ~mask;
    }
    lps22hb_write_register(device, INTERRUPT_CFG, reg_value);
}