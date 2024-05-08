#include <FreeRTOS.h>
#include <task.h>
#include "leds.h"
#include "main.h"

typedef struct BlinkingPattern_t {
    uint16_t onTime;
    uint16_t offTime;
} BlinkingPattern;

BlinkingPattern patterns[] = {
    {500, 500},  // Pattern 1: 500ms on, 500ms off
    {1000, 200}, // Pattern 2: 1s on, 200ms off
    {200, 800},  // Pattern 3: 200ms on, 800ms off
    // Add more patterns as needed
};

int currentPattern = 0;
int errorActive = 0;

void taskLedNotice(void *pvParameters) {
    while(1) {
        HAL_GPIO_WritePin(GPIOB, BLUE_LED_Pin, GPIO_PIN_SET);
        vTaskDelay(patterns[currentPattern].onTime);

        HAL_GPIO_WritePin(GPIOB, BLUE_LED_Pin, GPIO_PIN_RESET);
        vTaskDelay(patterns[currentPattern].offTime);
    }
}

void taskLedError(void *pvParameters) {
    HAL_GPIO_WritePin(GPIOB, BLUE_LED_Pin, GPIO_PIN_RESET);

    while(1) {
        if (errorActive) {
            HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, GPIO_PIN_SET);
            vTaskDelay(100);
            HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, GPIO_PIN_RESET);
            vTaskDelay(100);
        } else {
            vTaskDelay(1000);
        }
    }
}
