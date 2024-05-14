#pragma once

#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern SPI_HandleTypeDef hspi3;

typedef enum {
  LOW = 0U,
  HIGH
} PinState;

typedef struct {
    uint16_t DevAddress; // Device address
} I2C_Device;


void Error_Handler(void);
void initClock(void);
void pinMode(uint8_t pin, uint16_t mode);
void digitalWrite(uint8_t pin, PinState state);
GPIO_PinState digitalRead(uint8_t pin);
void delay(uint32_t ms);

void SPI_begin(void);
void SPI_end(void);
uint8_t SPI_transfer(uint8_t data);

void Serial_begin(uint32_t baudrate);
void Serial_print(char* message);

void I2C_begin(I2C_Device* device, uint16_t DevAddress);
void I2C_write(I2C_Device* device, uint16_t MemAddress, uint8_t *pData, uint16_t Size);
void I2C_read(I2C_Device* device, uint16_t MemAddress, uint8_t *pData, uint16_t Size);

#define PA0 0
#define PA1 1
#define PA2 2
#define PA3 3
#define PA4 4
#define LED1 5  // PA5 - LED1
#define PA6 6
#define PA7 7
#define PA8 8
#define PA9 9
#define PA10 10
#define PA11 11
#define PA12 12
#define PA13 13
#define PA14 14
#define PA15 15
#define PB0 16
#define PB1 17
#define PB2 18
#define PB3 19
#define PB4 20
#define PB5 21
#define PB6 22  // ST_LINK_UART1_TX
#define PB7 23  // ST_LINK_UART1_RX
#define PB8 24
#define PB9 25
#define PB10 26  // INTERNAL_I2C2_SCL
#define PB11 27  // INTERNAL_I2C2_SDA
#define PB12 28
#define PB13 29
#define LED2 30  // PB14 - LED2
#define PB15 31
#define PC0 32
#define PC1 33
#define PC2 34
#define PC3 35
#define PC4 36
#define PC5 37
#define PC6 38
#define PC7 39
#define PC8 40
#define PC9 41
#define PC10 42
#define PC11 43
#define PC12 44
#define PC13 45
#define PC14 46
#define PC15 47
#define PD0 48
#define PD1 49
#define PD2 50
#define PD3 51
#define PD4 52
#define PD5 53
#define PD6 54
#define PD7 55
#define PD8 56
#define PD9 57
#define PD10 58
#define PD11 59
#define PD12 60
#define PD13 61
#define PD14 62
#define PD15 63
#define PE0 64
#define PE1 65
#define PE2 66
#define PE3 67
#define PE4 68
#define PE5 69
#define PE6 70
#define PE7 71
#define PE8 72
#define PE9 73
#define PE10 74
#define PE11 75
#define PE12 76
#define PE13 77
#define PE14 78
#define PE15 79
