/*
 * it.c
 *
 *  Created on: Dec 28, 2024
 *      Author: windows 11
 */

#include "stm32f4xx_hal.h"
//Hnadle systick timer to ensure proper data transmission
void SysTick_Handler()
{
	//cube layer depends upon the global sysick (systic timer) variable
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();

}
