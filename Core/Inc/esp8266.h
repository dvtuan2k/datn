#ifndef _ESP8266_H_
#define _ESP8266_H_

void Esp8266_Command(UART_HandleTypeDef UartX,char *buffer, char *command);
void Esp8266_Init(UART_HandleTypeDef UartX);
void Esp8266_Connect(UART_HandleTypeDef UartX);
void Esp8266_Sendata(UART_HandleTypeDef UartX,uint8_t data1,float data2,uint8_t data3);

#endif /*_ESP8266_H_*/
