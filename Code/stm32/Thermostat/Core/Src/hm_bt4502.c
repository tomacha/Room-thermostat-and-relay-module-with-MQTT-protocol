#include "hm_bt4502.h"

uint8_t BT4502_Init(BT4502_t* BT4502)
{
	HAL_GPIO_WritePin(BT4502->PDN_GPIO, BT4502->PDN_GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(BT4502->PDN_GPIO, BT4502->PDN_GPIO_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(BT4502->WAKEUP_GPIO, BT4502->WAKEUP_GPIO_Pin, GPIO_PIN_SET);
	BT4502->state = BT4502_State_Advertising;

	return 0;
}

uint8_t BT4502_Send_Data(BT4502_t* BT4502, void* data, uint8_t length)
{
	HAL_GPIO_WritePin(BT4502->WAKEUP_GPIO, BT4502->WAKEUP_GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	if(HAL_UART_Transmit(BT4502->huart, (uint8_t*)data, length, HAL_MAX_DELAY) != 0)
	{
		return 1;
	}
	HAL_GPIO_WritePin(BT4502->WAKEUP_GPIO, BT4502->WAKEUP_GPIO_Pin, GPIO_PIN_SET);
	BT4502->state = BT4502_State_Data_Sent;
	return 0;
}

uint8_t BT4502_Send_String(BT4502_t* BT4502, const char* string)
{
	return BT4502_Send_Data(BT4502, (void *)string, strlen(string));
}

void BT4502_INT_Pin_Callback(BT4502_t* BT4502)
{
	uint16_t number = 1;
	if(HAL_GPIO_ReadPin(BT4502->INT_GPIO, BT4502->INT_GPIO_Pin) == GPIO_PIN_SET)
	{
		BT4502->state = BT4502_State_Data_Received;
	}
	else
	{
		HAL_UARTEx_ReceiveToIdle(BT4502->huart, BT4502->recv_buf, BT4502_MAX_SERIAL_DATA_PACKAGE, &number, HAL_MAX_DELAY);
	}
}

uint8_t BT4502_Sleep(BT4502_t* BT4502)
{
	if(BT4502->state != BT4502_State_Sleep)
	{
		HAL_GPIO_WritePin(BT4502->PDN_GPIO, BT4502->PDN_GPIO_Pin, GPIO_PIN_SET);
		BT4502->state = BT4502_State_Sleep;
		return 0;
	}
	else
	{
		return 1;
	}
}

uint8_t BT4502_WakeUp(BT4502_t* BT4502)
{
	if(BT4502->state == BT4502_State_Sleep)
	{
		HAL_GPIO_WritePin(BT4502->PDN_GPIO, BT4502->PDN_GPIO_Pin, GPIO_PIN_RESET);
		BT4502->state = BT4502_State_Advertising;
		return 0;
	}
	else
	{
		return 1;
	}
}
