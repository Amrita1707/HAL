/*
 * msp.c
 *
 *  Created on: Dec 28, 2024
 *      Author: windows 11
 */
#include "stm32f4xx_hal.h"

//Low level processor specific inits
void HAL_MspInit(void)
{
	//Set up the priority grouping of the arm processor.
	//defualt PRIORITYGROUP_4 will be selected
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	//Enable the required system exceptions of the arm processor.
    SCB->SHCSR |= (0x7 << 16);

	//Configure the priority for the system exceptions
	HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
	HAL_NVIC_SetPriority(BusFault_IRQn,0,0);
	HAL_NVIC_SetPriority(UsageFault_IRQn,0,0);
}

//low level initialization associated to specific peripheral (uart2)
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpio_uart;
	//Enable the clock for usart2 peripheral
	__HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

	//Do the pin muxing configurations
	//PA2-USART2 Tx and PA3-USART2 Rx (AF7)
	gpio_uart.Pin = GPIO_PIN_2;
	gpio_uart.Mode = GPIO_MODE_AF_PP;
	gpio_uart.Pull = GPIO_PULLUP;
	gpio_uart.Speed = GPIO_SPEED_FREQ_LOW;
    //connect uart function to this pin
	gpio_uart.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio_uart);
    //configured PA3 as USART2 Rx
    gpio_uart.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio_uart);

	//Enable the IRQ and set up the priority (NVIC settings)
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_NVIC_SetPriority(USART2_IRQn, 15, 0);

}

