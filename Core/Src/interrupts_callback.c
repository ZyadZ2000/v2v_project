#include "interrupts_callback.h"

extern volatile uint16_t slit_count;

extern xSemaphoreHandle send_message_semaphore;
extern xSemaphoreHandle receive_message_semaphore;
//extern xSemaphoreHandle touchScreen_semaphore;
//extern xSemaphoreHandle car_control_semaphore;

extern int8_t rx_buffer[34];
extern int8_t tx_buffer[34];

extern int8_t bluetooth_received_character;
extern int8_t car_control_character;
extern int8_t bluetooth_mode;
extern xSemaphoreHandle bluetooth_message_semaphore;

extern uint8_t icFlag;
extern uint32_t edge1Time;
extern uint32_t edge2Time;
extern uint8_t edgeNumber;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	if (huart->Instance == USART1) {
		NVIC_ClearPendingIRQ(USART1_IRQn);

		xSemaphoreGiveFromISR(receive_message_semaphore,
				&xHigherPriorityTaskWoken);

		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	} else if (huart->Instance == UART4) {
		if (bluetooth_received_character == '!')
			bluetooth_mode = BLTH_MESSAGE_MODE;
		switch (bluetooth_mode) {
		case BLTH_MESSAGE_MODE:
			NVIC_ClearPendingIRQ(USART1_IRQn);

			xSemaphoreGiveFromISR(bluetooth_message_semaphore,
					&xHigherPriorityTaskWoken);

			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken)
			;
			break;
		case BLTH_CAR_CTL_MODE:
			if (bluetooth_received_character != '\r' && bluetooth_received_character != '\n' ) {
				car_control_character = bluetooth_received_character;
			}
			HAL_UART_Receive_IT(&huart4,
					(uint8_t*) &bluetooth_received_character, 1);
			break;
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		NVIC_ClearPendingIRQ(USART1_IRQn);
		xSemaphoreGiveFromISR(send_message_semaphore,
				&xHigherPriorityTaskWoken);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {
		slit_count++;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (edgeNumber == 0) {
		edge1Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		edgeNumber = 1;
	} else if (edgeNumber == 1) {
		edge2Time = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		edgeNumber = 0;
		icFlag = 1;
	}
}

