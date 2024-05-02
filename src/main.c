#include <stdint.h>
#include <stdio.h>
#include <stm32f7xx.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stm32f7xx_hal.h>

//#define LED1_Pin GPIO_ODR_OD0
//#define LED2_Pin GPIO_ODR_OD7
//#define LED3_Pin GPIO_ODR_OD14

#define LD3_Pin GPIO_PIN_14
#define LD2_Pin GPIO_PIN_7
#define USER_Btn_Pin GPIO_PIN_13
#define USER_Btn_GPIO_Port GPIOC

uint8_t temp_counter = 0;

void taskBlink1(void *pvParameters) {
    while(1) {
        HAL_GPIO_TogglePin(GPIOB, LD2_Pin);
        vTaskDelay(1000);
    }
}

void taskBlink2(void *pvParameters) {
    while(1) {
        HAL_GPIO_TogglePin(GPIOB, LD3_Pin);
        vTaskDelay(500);
    }
}


void Error_Handler(void)
{
  __disable_irq();
  while (1) {
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    //osKernelInitialize();

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
