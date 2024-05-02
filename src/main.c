#include <stdint.h>
#include <stdio.h>
#include <stm32f7xx.h>
#include <FreeRTOS.h>
#include <task.h>

#define LED1_Pin GPIO_ODR_OD0
#define LED2_Pin GPIO_ODR_OD7
#define LED3_Pin GPIO_ODR_OD14


uint8_t temp_counter = 0;

void delay(int count) {
    for (volatile int i = 0; i < count; i++);
}

void taskBlink1(void *pvParameters) {
    while(1) {
        GPIOB->ODR |= LED1_Pin;
        vTaskDelay(1000);

        GPIOB->ODR &= ~LED1_Pin;
        vTaskDelay(1000);
    }
}

void taskBlink2(void *pvParameters) {
    while(1) {
        GPIOB->ODR |= LED2_Pin;
        vTaskDelay(500);

        GPIOB->ODR &= ~LED2_Pin;
        vTaskDelay(500);
    }
}

void initialise_clocks() {
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_ETHMACEN);

    // ethernet needs ports A,B,C and G
    SET_BIT(RCC->AHB1ENR, RCC_AHB1ENR_GPIOAEN |
                          RCC_AHB1ENR_GPIOBEN |
                          RCC_AHB1ENR_GPIOCEN |
                          RCC_AHB1ENR_GPIOGEN);
}

int main(void)
{
    initialise_clocks();

    // Enable tracing
    ITM->TCR |= (1 << 24);
    ITM->TER |= 1;  // enable trace on stimulus port 0

    printf("Hello World\n");

    // Configure PB7 as output
    GPIOB->MODER &= ~GPIO_MODER_MODER7;
    GPIOB->MODER |= GPIO_MODER_MODER7_0;

    // start FreeRTOS tasks
    xTaskCreate(taskBlink1, "Blink1", 128, NULL, 1, NULL);
    xTaskCreate(taskBlink2, "Blink2", 128, NULL, 1, NULL);

    vTaskStartScheduler();

	/*while(1) {
		//set PB7 to high
		GPIOB->ODR |= (LED1_Pin | LED2_Pin | LED3_Pin);
        delay(100000);
		//set PB7 back to zero
		GPIOB->ODR &= ~ (LED1_Pin | LED2_Pin | LED3_Pin);
        delay(100000);
        temp_counter++;

        if (temp_counter > 15) {
            temp_counter = 0;
        }
	}*/
    return 0;
}
