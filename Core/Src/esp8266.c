#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "esp8266.h"

static void Esp8266_ClearBuffer(char *buffer)
{
	uint8_t index;
	uint8_t length = strlen(buffer);

	for(index = 0;index < length;index++)
	{
		buffer[index] = 0;
	}
}

static void Esp8266_Strcpy(char *mainBuffer, char *subBuffer)
{
	uint8_t index=0;
	uint8_t length = strlen(subBuffer);

	Esp8266_ClearBuffer(mainBuffer);
	for(index = 0;index < length;index++)
	{
		mainBuffer[index] = subBuffer[index];
	}
}

void Esp8266_Command(UART_HandleTypeDef UartX,char *buffer, char *command)
{
	Esp8266_Strcpy(buffer, command);
	HAL_UART_Transmit(&UartX,(uint8_t *)buffer, strlen(buffer),100);
}

/* Esp8266_Setup only use for this project you can use Esp8266_Command to set up but i have not still verify yet =)) */
void Esp8266_Init(UART_HandleTypeDef UartX)
{
	char aEspConfig[100]={0};
	sprintf(aEspConfig,"AT\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(2000);
	sprintf(aEspConfig,"ATE0\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(2000);
	sprintf(aEspConfig,"AT+CWMODE=1\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(2000);
	sprintf(aEspConfig,"AT+CIPMUX=0\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(2000);
	sprintf(aEspConfig,"AT+CWJAP=\"Tuan\",\"01234567\"\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(5000);
}
void Esp8266_Connect(UART_HandleTypeDef UartX)
{
	char aEspConfig[70]={0};
	sprintf(aEspConfig,"AT+CIPSTART=\"TCP\",\"dvtuandatn.000webhostapp.com\",80\r\n");
	HAL_UART_Transmit(&UartX,(uint8_t *)aEspConfig, strlen(aEspConfig),100);
	Esp8266_ClearBuffer(aEspConfig);
	HAL_Delay(2000);
}

void Esp8266_Sendata(UART_HandleTypeDef UartX,uint8_t data1,float data2,uint8_t data3)
{
   uint8_t length = 0;
   char aSizeToSend[20]={0};
   char aBufferOfData[150]={0};
   sprintf(aBufferOfData,"GET /insert.php?code=%d&water1=%f&month=%d HTTP/1.1\r\nHost: dvtuandatn.000webhostapp.com\r\n\r\n",data1,data2,data3);
   length=strlen(aBufferOfData);
   sprintf(aSizeToSend,"AT+CIPSEND=%i\r\n",length);
   HAL_UART_Transmit(&UartX,(uint8_t *)aSizeToSend, strlen(aSizeToSend),100);
   HAL_Delay(100);
   HAL_UART_Transmit(&UartX,(uint8_t *)aBufferOfData, strlen(aBufferOfData),100);
   HAL_Delay(1000);
   Esp8266_ClearBuffer(aSizeToSend);
   sprintf(aSizeToSend,"AT+CIPCLOSE\r\n");
   HAL_UART_Transmit(&UartX,(uint8_t *)aSizeToSend, strlen(aSizeToSend),100);
}
