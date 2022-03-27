#ifndef	_HM_BT4502_H
#define	_HM_BT4502_H

#include "gpio.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

#define BT4502_MAX_SERIAL_DATA_PACKAGE 240

typedef enum {
	BT4502_State_Advertising = 0,
	BT4502_State_Data_Received = 1,
	BT4502_State_Connected = 2,
	BT4502_State_Data_Sent = 3,
	BT4502_State_Sleep = 4,
	BT4502_State_Uninitialized = 5
} BT4502_State_t;

typedef struct
{
	GPIO_TypeDef* 			INT_GPIO;
	uint16_t 				INT_GPIO_Pin;
	GPIO_TypeDef* 			PDN_GPIO;
	uint16_t 				PDN_GPIO_Pin;
	GPIO_TypeDef* 			WAKEUP_GPIO;
	uint16_t 				WAKEUP_GPIO_Pin;
	UART_HandleTypeDef* 	huart;
	BT4502_State_t 			state;
	uint8_t					recv_buf[BT4502_MAX_SERIAL_DATA_PACKAGE];
	char*					auth_code;
} BT4502_t;

uint8_t BT4502_Init(BT4502_t* BT4502);
uint8_t BT4502_Send_Data(BT4502_t* BT4502, void* data, uint8_t length);
uint8_t BT4502_Send_String(BT4502_t* BT4502, const char* string);
void BT4502_INT_Pin_Callback(BT4502_t* BT4502);
uint8_t BT4502_Sleep(BT4502_t* BT4502);
uint8_t BT4502_WakeUp(BT4502_t* BT4502);

#endif /*_HM-BT4502_H */
