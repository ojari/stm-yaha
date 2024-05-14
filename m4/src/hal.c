#include "hal.h"
#include "string.h"

UART_HandleTypeDef huart1;
SPI_HandleTypeDef hspi3;
I2C_HandleTypeDef hi2c2;

typedef struct {
    GPIO_TypeDef *Port;
    uint32_t Pin;
} PinConfig;

const PinConfig pins[] __attribute__((section(".rodata"))) = {
    {GPIOA, GPIO_PIN_0},   // 0
    {GPIOA, GPIO_PIN_1},   // 1
    {GPIOA, GPIO_PIN_2},   // 2
    {GPIOA, GPIO_PIN_3},   // 3
    {GPIOA, GPIO_PIN_4},   // 4
    {GPIOA, GPIO_PIN_5},   // 5
    {GPIOA, GPIO_PIN_6},   // 6
    {GPIOA, GPIO_PIN_7},   // 7
    {GPIOA, GPIO_PIN_8},   // 8
    {GPIOA, GPIO_PIN_9},   // 9
    {GPIOA, GPIO_PIN_10},  // 10
    {GPIOA, GPIO_PIN_11},  // 11
    {GPIOA, GPIO_PIN_12},  // 12
    {GPIOA, GPIO_PIN_13},  // 13
    {GPIOA, GPIO_PIN_14},  // 14
    {GPIOA, GPIO_PIN_15},  // 15
    {GPIOB, GPIO_PIN_0},   // 16
    {GPIOB, GPIO_PIN_1},   // 17
    {GPIOB, GPIO_PIN_2},   // 18
    {GPIOB, GPIO_PIN_3},   // 19
    {GPIOB, GPIO_PIN_4},   // 20
    {GPIOB, GPIO_PIN_5},   // 21
    {GPIOB, GPIO_PIN_6},   // 22
    {GPIOB, GPIO_PIN_7},   // 23
    {GPIOB, GPIO_PIN_8},   // 24
    {GPIOB, GPIO_PIN_9},   // 25
    {GPIOB, GPIO_PIN_10},  // 26
    {GPIOB, GPIO_PIN_11},  // 27
    {GPIOB, GPIO_PIN_12},  // 28
    {GPIOB, GPIO_PIN_13},  // 29
    {GPIOB, GPIO_PIN_14},  // 30
    {GPIOB, GPIO_PIN_15},  // 31
    {GPIOC, GPIO_PIN_0},   // 32
    {GPIOC, GPIO_PIN_1},   // 33
    {GPIOC, GPIO_PIN_2},   // 34
    {GPIOC, GPIO_PIN_3},   // 35
    {GPIOC, GPIO_PIN_4},   // 36
    {GPIOC, GPIO_PIN_5},   // 37
    {GPIOC, GPIO_PIN_6},   // 38
    {GPIOC, GPIO_PIN_7},   // 39
    {GPIOC, GPIO_PIN_8},   // 40
    {GPIOC, GPIO_PIN_9},   // 41
    {GPIOC, GPIO_PIN_10},  // 42
    {GPIOC, GPIO_PIN_11},  // 43
    {GPIOC, GPIO_PIN_12},  // 44
    {GPIOC, GPIO_PIN_13},  // 45
    {GPIOC, GPIO_PIN_14},  // 46
    {GPIOC, GPIO_PIN_15},  // 47
    {GPIOD, GPIO_PIN_0},   // 48
    {GPIOD, GPIO_PIN_1},   // 49
    {GPIOD, GPIO_PIN_2},   // 50
    {GPIOD, GPIO_PIN_3},   // 51
    {GPIOD, GPIO_PIN_4},   // 52
    {GPIOD, GPIO_PIN_5},   // 53
    {GPIOD, GPIO_PIN_6},   // 54
    {GPIOD, GPIO_PIN_7},   // 55
    {GPIOD, GPIO_PIN_8},   // 56
    {GPIOD, GPIO_PIN_9},   // 57
    {GPIOD, GPIO_PIN_10},  // 58
    {GPIOD, GPIO_PIN_11},  // 59
    {GPIOD, GPIO_PIN_12},  // 60
    {GPIOD, GPIO_PIN_13},  // 61
    {GPIOD, GPIO_PIN_14},  // 62
    {GPIOD, GPIO_PIN_15},  // 63
    {GPIOE, GPIO_PIN_0},   // 64
    {GPIOE, GPIO_PIN_1},   // 65
    {GPIOE, GPIO_PIN_2},   // 66
    {GPIOE, GPIO_PIN_3},   // 67
    {GPIOE, GPIO_PIN_4},   // 68
    {GPIOE, GPIO_PIN_5},   // 69
    {GPIOE, GPIO_PIN_6},   // 70
    {GPIOE, GPIO_PIN_7},   // 71
    {GPIOE, GPIO_PIN_8},   // 72
    {GPIOE, GPIO_PIN_9},   // 73
    {GPIOE, GPIO_PIN_10},  // 74
    {GPIOE, GPIO_PIN_11},  // 75
    {GPIOE, GPIO_PIN_12},  // 76
    {GPIOE, GPIO_PIN_13},  // 77
    {GPIOE, GPIO_PIN_14},  // 78
    {GPIOE, GPIO_PIN_15},  // 79
};


void Error_Handler(void)
{
	__disable_irq();
    while (1) {
    }
}

// Initialize the system clock
void initClock(void) {
    // Implement system clock initialization here
}

// Initialize GPIO
void pinMode(uint8_t pin, uint16_t mode) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clock
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Configure GPIO pin
    GPIO_InitStruct.Pin = pins[pin].Pin;
    GPIO_InitStruct.Mode = mode;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(pins[pin].Port, &GPIO_InitStruct);
}

// Write to a GPIO pin
void digitalWrite(uint8_t pin, PinState state) {
    HAL_GPIO_WritePin(pins[pin].Port, pins[pin].Pin, (GPIO_PinState)state);
}

// Read from a GPIO pin
GPIO_PinState digitalRead(uint8_t pin) {
    return HAL_GPIO_ReadPin(pins[pin].Port, pins[pin].Pin);
}

// Delay function
void delay(uint32_t ms) {
    HAL_Delay(ms);
}

// Initialize SPI
void SPI_begin(void) {
    hspi3.Instance = SPI1;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        Error_Handler();
    }
}

// Send data over SPI
uint8_t SPI_transfer(uint8_t data) {
    uint8_t receivedData;
    HAL_SPI_TransmitReceive(&hspi3, &data, &receivedData, 1, HAL_MAX_DELAY);
    return receivedData;
}

// Initialize UART
void Serial_begin(uint32_t baudrate) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = baudrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
}

// Send data over UART
void Serial_print(char* str) {
    HAL_UART_Transmit(&huart1, (uint8_t*)str, strlen(str), HAL_MAX_DELAY);
}

void Serial_printi(uint8_t value) {
    char buffer[10];
    uint8_t i = 0;
    uint8_t length = 0;
    uint8_t temp = value;

    // Calculate the length of the number
    do {
        length++;
        temp /= 10;
    } while (temp != 0);

    // Convert the number to a string
    do {
        buffer[length - i - 1] = '0' + (value % 10);
        value /= 10;
        i++;
    } while (value != 0);

    buffer[length] = '\0';
    Serial_print(buffer);
}

// Receive data over UART
void Serial_read(char* buffer, uint16_t size) {
    HAL_UART_Receive(&huart1, (uint8_t*)buffer, size, HAL_MAX_DELAY);
}


// I2C driver
//
void I2C_begin(I2C_Device* device, uint16_t writeAddress, uint16_t readAddresss) {
    device->writeAddress = writeAddress;
    device->readAddress = readAddresss;

    hi2c2.Instance = I2C2;
    hi2c2.Init.Timing = 0x10909CEC;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}

void I2C_write(I2C_Device* device, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
    HAL_I2C_Mem_Write(&hi2c2, device->writeAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}

void I2C_read(I2C_Device* device, uint16_t MemAddress, uint8_t *pData, uint16_t Size) {
    HAL_I2C_Mem_Read(&hi2c2, device->readAddress, MemAddress, I2C_MEMADD_SIZE_8BIT, pData, Size, HAL_MAX_DELAY);
}