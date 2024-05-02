#include "main.h"

extern ETH_HandleTypeDef heth;
extern TIM_HandleTypeDef htim1;

void NMI_Handler(void)
{
    Error_Handler();
}

void HardFault_Handler(void)
{
    Error_Handler();
}

void MemManage_Handler(void)
{
    Error_Handler();
}

void BusFault_Handler(void)
{
    Error_Handler();
}

void UsageFault_Handler(void)
{
    Error_Handler();
}

void DebugMon_Handler(void)
{
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void ETH_IRQHandler(void)
{
  HAL_ETH_IRQHandler(&heth);
}
