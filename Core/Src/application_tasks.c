#include "application_tasks.h"
#include "helper_functions.h"

extern volatile uint16_t Global_u16SlitCount;
extern xSemaphoreHandle send_message_semaphore;
extern xSemaphoreHandle receive_message_semaphore;
extern TaskHandle_t send_message_task_handle;
extern uint8_t tx_buffer[10];
extern uint8_t rx_buffer[10];

void Task_sendMessage(void *parameters) {
	xSemaphoreTake(send_message_semaphore, 0);
	vTaskSuspend(NULL);
	while (1) {
		HAL_UART_Transmit_DMA(&huart1, tx_buffer, 10);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
		vTaskSuspend(NULL);
		xSemaphoreTake(send_message_semaphore, portMAX_DELAY);
	}
}

void Task_handleReceivedMessage(void *parameters) {
	//Receive the message
	xSemaphoreTake(receive_message_semaphore, 0);
	while (1) {
		xSemaphoreTake(receive_message_semaphore, portMAX_DELAY);
		//Calculate if there is any danger
		//Display on LCD
		HAL_UART_Receive_DMA(&huart1, rx_buffer, 10);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
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
