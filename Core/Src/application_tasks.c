#include "application_tasks.h"

volatile uint16_t Global_u16SlitCount = 0;
xSemaphoreHandle send_message_semaphore;
TaskHandle_t send_message_task_handle;

void Task_sendMessage(void *parameters) {
	vTaskSuspend(NULL);
	while(1){
		xSemaphoreTake(send_message_semaphore, portMAX_DELAY);
		HAL_UART_Transmit_DMA(&huart1, tx_buffer, 10);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		vTaskSuspend(NULL);
	}
}

void Task_speedCalculation(void *parameters) {

	static uint16_t Global_u16PreviousSpeed = 0;
	int16_t Local_u16DifferSpeed = 0;
	uint16_t Local_u16CurrentSpeed = 0;

	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	vTaskDelay(500 / portTICK_RATE_MS);

	while (1) {

		HAL_NVIC_DisableIRQ(EXTI0_IRQn);

		Local_u16CurrentSpeed = (1.6328 / 2) * Global_u16SlitCount;
		Local_u16DifferSpeed = Local_u16CurrentSpeed - Global_u16PreviousSpeed;

		if (Local_u16DifferSpeed <= SS_VELOCITY_THRESHOLD) {
			//Transmit UART Message using DMA
			vTaskResume(send_message_task_handle);
		}
		Global_u16PreviousSpeed = Local_u16CurrentSpeed;
		Global_u16SlitCount = 0;

		HAL_NVIC_EnableIRQ(EXTI0_IRQn);
		vTaskDelay(500 / portTICK_RATE_MS);
	}
}
