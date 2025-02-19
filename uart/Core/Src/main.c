/*
 * main.c
 *
 *  Created on: Dec 28, 2024
 *      Author: windows 11
 */

#include<string.h>
#include "stm32f4xx_hal.h"
#include "main.h"

void SystemClockConfig();
void UART2_Init();
void Errror_handler();

UART_HandleTypeDef uart2;
char *user_data = "Uart Transmission is done\r\n";
uint8_t convert_to_caps( uint8_t data)
{
	if( data >='a' && data <='z')
	{
		data = data - 32;
	}
	return data;
}

int main(void)
{
	uint16_t len = strlen(user_data);

	HAL_Init();
    SystemClockConfig();
    UART2_Init();
    if(HAL_UART_Transmit(&uart2,(uint8_t*)user_data,len,HAL_MAX_DELAY)!= HAL_OK)
    {
    	Errror_handler();
    }
    uint8_t rec_data;
    uint32_t count = 0;
    uint8_t  data_buffer[100];
    while(1)
    {
    	HAL_UART_Receive(&uart2, &rec_data,1, HAL_MAX_DELAY);
    	if(rec_data == '\r')
    	{
    		break;
    	}
    	else
    	{
    		data_buffer[count++] = convert_to_caps(rec_data);
    	}
    }
    data_buffer[count++] = '\r';
    HAL_UART_Transmit(&uart2,data_buffer,count,HAL_MAX_DELAY);

    while(1);

return 0;
}

//implement when need spcl clock settings insted it use internal RC oscillator
void SystemClockConfig()
{

}

//High level peripheral initialization (parameter initializations)
void UART2_Init()
{
  uart2.Instance = USART2;
  uart2.Init.BaudRate = 115200;
  uart2.Init.WordLength = UART_WORDLENGTH_8B;
  uart2.Init.StopBits = UART_STOPBITS_1;
  uart2.Init.Parity = UART_PARITY_NONE;
  uart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  uart2.Init.Mode = UART_MODE_TX_RX;
  if(HAL_UART_Init(&uart2) != HAL_OK)
  {
	  Errror_handler();
  }
}

void Errror_handler()
{
	while(1);
}
