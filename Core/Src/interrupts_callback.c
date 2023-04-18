#include "interrupts_callback.h"

extern volatile uint16_t Global_u16SlitCount;
extern xSemaphoreHandle send_message_semaphore;
extern xSemaphoreHandle receive_message_semaphore;
extern uint8_t rx_buffer[10];
extern uint8_t tx_buffer[10];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

		NVIC_ClearPendingIRQ(USART1_IRQn);

		xSemaphoreGiveFromISR(receive_message_semaphore, &xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		NVIC_ClearPendingIRQ(USART1_IRQn);
		xSemaphoreGiveFromISR(send_message_semaphore, &xHigherPriorityTaskWoken);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_0)
	{
		Global_u16SlitCount++;
	}
}
