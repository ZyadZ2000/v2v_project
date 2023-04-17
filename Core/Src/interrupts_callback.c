#include "interrupts_callback.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		HAL_UART_Receive_DMA(&huart1, rx_buffer, 10);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
