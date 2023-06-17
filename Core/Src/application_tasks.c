#include "application_tasks.h"
#include "helper_functions.h"
#include "NMEA.h"
#include "uartRingBuffer.h"
#include <stdlib.h>
#include <math.h>
#include "CLCD_interface.h"
#include "CAR_CTRL_interface.h"
#include <string.h>

extern volatile uint16_t slit_count;

extern xSemaphoreHandle send_message_semaphore;
extern xSemaphoreHandle receive_message_semaphore;
extern xSemaphoreHandle touchScreen_semaphore;
extern xSemaphoreHandle bluetooth_message_semaphore;
extern TaskHandle_t send_message_task_handle;

extern int8_t tx_buffer[34];
extern int8_t rx_buffer[34];
extern int8_t arrest_message_buffer[11];

extern GPSSTRUCT GGAST;
extern ring_buffer *_rx_buffer;

extern char second_car_latitude[11];
extern char second_car_longitude[11];
extern char second_car_direction[8];

extern int8_t car_control_character;

extern double my_car_latitude;
extern double my_car_longitude;
extern char north_south;
extern char east_west;
extern double car_direction;

extern uint8_t Upper_Left[8];
extern uint8_t Upper_Right[8];
extern uint8_t Lower_Left[8];
extern uint8_t Lower_Right[8];
extern uint8_t Lower_Mid[8];
extern uint8_t Upper_Mid[8];

extern int8_t bluetooth_mode;
extern int8_t bluetooth_received_character;

extern int8_t message_sending_mode;

void Task_sendMessage(void *parameters) {
	xSemaphoreTake(send_message_semaphore, 0);
	vTaskSuspend(NULL);
	while (1) {
		switch(message_sending_mode){
		case WARNING_MSG_MODE:
			Build_Msg((char*) tx_buffer, my_car_latitude, my_car_longitude,
					north_south, east_west, car_direction);

			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) tx_buffer, 34);
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
			break;
		case ARREST_MSG_MODE:
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*) arrest_message_buffer, 11);
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
			message_sending_mode = WARNING_MSG_MODE;
			break;
		}
		vTaskSuspend(NULL);
		xSemaphoreTake(send_message_semaphore, portMAX_DELAY);
	}
}

void Task_handleReceivedMessage(void *parameters) {
	//Receive the message
	unsigned int i = 0, j = 0;
	int8_t message_start = 0;
	int8_t received_plate[7] = { 0 };
	double message_latitude, message_longitude, message_direction;
	double distance_calculation, bearing_difference;
	xSemaphoreTake(receive_message_semaphore, 0);
	HAL_UART_Receive_IT(&huart1, (uint8_t*)&message_start, 1);
	while (1) {
		xSemaphoreTake(receive_message_semaphore, portMAX_DELAY);
		switch (message_start) {
		case '#':
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, 33);
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
			xSemaphoreTake(receive_message_semaphore, portMAX_DELAY);
			//rx_buffer contains message
			//Extract up to N for latitude, up to E for longitude
			i = 0, j = 0;
			while (i < 34 && rx_buffer[i] != 'N' && rx_buffer[i] != 'S'
					&& j < 11) {
				second_car_latitude[j] = rx_buffer[i];
				i++;
				j++;
			}
			second_car_latitude[j] = '\0';
			i++;         //no need for NS
			j = 0;

			while (i < 34 && rx_buffer[i] != 'E' && rx_buffer[i] != 'W'
					&& j < 11) {
				second_car_longitude[j] = rx_buffer[i];
				i++;
				j++;
			}
			second_car_longitude[j] = '\0';
			i++;          //no need for EW
			j = 0;

			while (i < 34 && rx_buffer[i] != '#' && j < 8) {
				second_car_direction[j] = rx_buffer[i];
				i++;
				j++;
			}
			second_car_direction[j] = '\0';

			message_latitude = atof(second_car_latitude);
			message_longitude = atof(second_car_longitude);
			message_direction = atof(second_car_direction);

			//bearing_difference
			bearing_difference = fabs(message_direction - car_direction);

			CLCD_voidDisplayClear();

			CLCD_voidWriteSpeacialChar(Upper_Left, 0, 0, 13);
			CLCD_voidWriteSpeacialChar(Upper_Mid, 1, 0, 14);
			CLCD_voidWriteSpeacialChar(Upper_Right, 2, 0, 15);

			CLCD_voidWriteSpeacialChar(Lower_Left, 3, 1, 13);
			CLCD_voidWriteSpeacialChar(Lower_Mid, 4, 1, 14);
			CLCD_voidWriteSpeacialChar(Lower_Right, 5, 1, 15);

			CLCD_voidGoToXY(0, 0);

			CLCD_voidSendString("Warning: Car");

			CLCD_voidGoToXY(1, 2);

			CLCD_voidSendString("In Front");
			break;
		case '!':
			HAL_UART_Receive_DMA(&huart1, (uint8_t*)rx_buffer, 10); //!CAR123!\r\n\0
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
			xSemaphoreTake(receive_message_semaphore, portMAX_DELAY);
			i = 0;
			while (rx_buffer[i] != '!' && i < 6) {
				received_plate[i] = rx_buffer[i];
				i++;
			}
			received_plate[i] = '\0';
			if (strcmp((const char *)received_plate, (const char *)CAR_PLATE) == 0) {
				/* Decrease The Speed Gradually */
				if ((TIM3->CCR1) >= 10) {
					TIM3->CCR1 -= 1;
					TIM12->CCR1 -= 1;
				}
				/* Stop The RC Car */
				else {
					TIM3->CCR1 = 0;
					TIM12->CCR1 = 0;
				}
			}
			break;
		}

		HAL_UART_Receive_IT(&huart1, (uint8_t*)&message_start, 1);
#if 0
		if (bearing_difference < 45 || bearing_difference > 135) {
			//moving in same direction
			if (bearing_difference > 170 || bearing_difference < 10) {
				//same line
				distance_calculation = distance(my_car_latitude,
						my_car_longitude, message_latitude, message_longitude);
				//based on the distance_calculation decide on displaying the warning
				if (distance_calculation < 20) {
					//Display warning
					CLCD_voidDisplayClear();

					CLCD_voidWriteSpeacialChar(Upper_Left, 0, 0, 13);
					CLCD_voidWriteSpeacialChar(Upper_Mid, 1, 0, 14);
					CLCD_voidWriteSpeacialChar(Upper_Right, 2, 0, 15);

					CLCD_voidWriteSpeacialChar(Lower_Left, 3, 1, 13);
					CLCD_voidWriteSpeacialChar(Lower_Mid, 4, 1, 14);
					CLCD_voidWriteSpeacialChar(Lower_Right, 5, 1, 15);

					CLCD_voidGoToXY(0, 0);

					CLCD_voidSendString("Warning: Car");

					CLCD_voidGoToXY(1, 2);

					CLCD_voidSendString("In Front");

				}
			}
		}
#endif
	}
}

void Task_speedCalculation(void *parameters) {

	float current_speed = 0;
	float previous_speed = 0;

	float speed_difference = 0;

	TickType_t xLastWakeTime;

	taskENTER_CRITICAL();

	slit_count = 0;
	xLastWakeTime = xTaskGetTickCount();

	taskEXIT_CRITICAL();

	while (1) {

		vTaskDelayUntil(&xLastWakeTime, 500 / portTICK_RATE_MS);

		taskENTER_CRITICAL();

		/* (1.6328 / 2) = 0.8164 */
		//current_speed = (uint16_t) (0.8164 * slit_count); //In Cm/Sec
		current_speed = (slit_count / 20.0) * 120.0; //RPM
		slit_count = 0;

		taskEXIT_CRITICAL();

		speed_difference = current_speed - previous_speed;

		previous_speed = current_speed;

		if (speed_difference < SS_VELOCITY_THRESHOLD) {
			//Transmit UART Message using DMA
			//taskENTER_CRITICAL();
			HAL_NVIC_DisableIRQ(EXTI0_IRQn);
			vTaskResume(send_message_task_handle);
			HAL_NVIC_EnableIRQ(EXTI0_IRQn);
//			CLCD_voidDisplayClear();
//			CLCD_voidGoToXY(0, 0);
//			CLCD_voidSendString("Sending...");

			//taskEXIT_CRITICAL();
		}

	}
}

void Task_readingGPS(void *parameters) {
	int flagGGA = 1;
	char c1;
	char GGA[100];
	//char count = 0;
	//double latitude_sum = 0.0;
	//double longitude_sum = 0.0;

	char GGA_loop_index = 0;
	while (1) {
		flagGGA = 1;
		//count = 0;
		//latitude_sum = 0.0;
		//longitude_sum = 0.0;

		//char uartBuflong[100] ={0};
		//char uartBuflat[100] ={0};
		do {
			for (GGA_loop_index = 0; GGA_loop_index < 100; GGA_loop_index++) {
				GGA[(int) GGA_loop_index] = 0;
			}
			// extracting the message ($gga)
			//HAL_UART_Transmit(&huart6,(uint8_t *)"extracting the message", 23, 100);

			//MCAL_UART_u8ReceiveData(UART_3, (uint8_t *)&c);
			HAL_UART_Receive(&huart3, (uint8_t*) &c1, sizeof(c1), 1000);
			// start with the dollar sign if not then loop to find it
			for (int k = 0; c1 != '$'; k++) {
				// MCAL_UART_u8ReceiveData(UART_3, (uint8_t *)&c);
				HAL_UART_Receive(&huart3, (uint8_t*) &c1, sizeof(c1), 1000);
			}

			store_char(c1, _rx_buffer);
			//MCAL_UART_u8ReceiveData(UART_3,(uint8_t *) &c);
			HAL_UART_Receive(&huart3, (uint8_t*) &c1, sizeof(c1), 1000);
			// loop to store all the frame after dollar until next dollar recieved
			while (c1 != '$') {
				store_char(c1, _rx_buffer);
				//MCAL_UART_u8ReceiveData(UART_3,(uint8_t *) &c);
				HAL_UART_Receive(&huart3, (uint8_t*) &c1, sizeof(c1), 1000);
			}

			// once the new dollar received the old frame is now in the buffer to be decoded
			// end of extracting

			// HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
			// HAL_UART_Transmit(&huart6,(uint8_t *) "Before enter do while ", 23, 100);
			//  HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
			//HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
			//HAL_UART_Transmit(&huart6,(uint8_t *) "inside do while ", 17, 100);
			flagGGA = 1;

			if (Wait_for("GGA") == 1) {
				// HAL_UART_Transmit(&huart6,(uint8_t *) "found gga ", 11, 100);
				//HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
				//HAL_UART_Transmit(&huart6,(uint8_t *) "outside wait ", 13, 100);
				Copy_upto("*", GGA);
				if (decodeGGA(GGA, &GGAST.ggastruct) == 0) {
					//latitude_sum += GGAST.ggastruct.lcation.latitude;
					//longitude_sum += GGAST.ggastruct.lcation.longitude;
					//count++;
					//if (count == 1) {
					flagGGA = 0;
					//count = 0;
					//}
				}
			}

		} while (flagGGA == 1);

		// in using freertos
		// here is the delay ------
		taskENTER_CRITICAL();
		my_car_latitude = GGAST.ggastruct.lcation.latitude;	//latitude_sum ;/// 2;//* 100;
		my_car_longitude = GGAST.ggastruct.lcation.longitude;//longitude_sum;// / 2; //* 100;
		north_south = GGAST.ggastruct.lcation.NS;
		east_west = GGAST.ggastruct.lcation.EW;
		taskEXIT_CRITICAL();

		vTaskDelay(100 / portTICK_RATE_MS);

		// here for just showing data using uart3
		// HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
		//  HAL_UART_Transmit(&huart6,(uint8_t *) "outside do while ", 18, 100);
		// HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
		// ftoa(GGAST.ggastruct.lcation.longitude ,uartBuflong,6);
		//  ftoa(GGAST.ggastruct.lcation.latitude ,uartBuflat,6);
		//HAL_UART_Transmit(&huart6,(uint8_t *) "Longitude is : ", 16, 100);
		//HAL_UART_Transmit(&huart6, (uint8_t *)uartBuflong, sizeof(uartBuflong), 100);
		// HAL_UART_Transmit(&huart6, (uint8_t *)&(GGAST.ggastruct.lcation.EW), sizeof(GGAST.ggastruct.lcation.EW), 100);
		//HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);
		// HAL_UART_Transmit(&huart6, (uint8_t *)"Latitude is : ", 15, 100);
		// HAL_UART_Transmit(&huart6, (uint8_t *)uartBuflat, sizeof(uartBuflat), 100);
		// HAL_UART_Transmit(&huart6, (uint8_t *)&(GGAST.ggastruct.lcation.NS), sizeof(GGAST.ggastruct.lcation.NS), 100);

		//  HAL_UART_Transmit(&huart6,(uint8_t *)"\r\n", 2, 100);

		/* USER CODE BEGIN 3 */

	}
}

void Task_directionOfCar(void *parameters) {
	static volatile double car_lat1;
	static volatile double car_lon1;
	static volatile double car_lat2;
	static volatile double car_lon2;
	while (1) {
		//taskENTER_CRITICAL();
		car_lat1 = my_car_latitude;
		car_lon1 = my_car_longitude;
		//taskEXIT_CRITICAL();

		vTaskDelay(5000 / portTICK_RATE_MS);

		//taskENTER_CRITICAL();
		car_lat2 = my_car_latitude;
		car_lon2 = my_car_longitude;
		//taskEXIT_CRITICAL();

		taskENTER_CRITICAL();
		car_direction = bearing(car_lat1, car_lon1, car_lat2, car_lon2);
		taskEXIT_CRITICAL();
	}

}

/*
 * f b r l s
 * !CAR222!
 * */
void Task_controlCar(void *parameters) {

	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	while (1) {
		//car_control_character = 0;

		//HAL_UART_Receive_IT(&huart4, &Local_u8Received_data, 1);

		vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);

		if (car_control_character != 'f' && car_control_character != 'b'
				&& car_control_character != 'l' && car_control_character != 'r'
				&& car_control_character != 's') {
			/* Decrease The Speed Gradually */
			if ((TIM3->CCR1) >= 10) {
				TIM3->CCR1 -= 1;
				TIM12->CCR1 -= 1;
			}
			/* Stop The RC Car */
			else {
				TIM3->CCR1 = 0;
				TIM12->CCR1 = 0;
			}
		} else {

			/* Return To The Normal Speed */
			TIM3->CCR1 = 75;
			TIM12->CCR1 = 75;

			/* Direction Change According To The Received Direction */
			if (car_control_character == 'f')
				HAL_CAR_CTRL_voidForward();
			else if (car_control_character == 'b')
				HAL_CAR_CTRL_voidBackward();
			else if (car_control_character == 'l')
				HAL_CAR_CTRL_voidRight();
			else if (car_control_character == 'r')
				HAL_CAR_CTRL_voidLeft();
			else if (car_control_character == 's') {
				HAL_CAR_CTRL_voidStop();
				CLCD_voidDisplayClear();
			}
		}

	}

}

void Task_arrestMessageHandler(void *parameters) {
	xSemaphoreTake(bluetooth_message_semaphore, 0);

	/* Start receiving from UART4 which is connected to the blue-tooth */
	HAL_UART_Receive_IT(&huart4, (uint8_t *)&bluetooth_received_character, 1);

	while (1) {
		xSemaphoreTake(bluetooth_message_semaphore, portMAX_DELAY);
		arrest_message_buffer[0] = bluetooth_received_character;
		arrest_message_buffer[7] = 0;
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)(arrest_message_buffer + 1), 7);
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
		xSemaphoreTake(bluetooth_message_semaphore, (1000 / portTICK_RATE_MS));
		bluetooth_mode = BLTH_CAR_CTL_MODE;
		if(arrest_message_buffer[7] == '!'){
			arrest_message_buffer[8] = '\r';
			arrest_message_buffer[9] = '\n';
			arrest_message_buffer[10] = '\0';
			message_sending_mode = ARREST_MSG_MODE;
		}
		HAL_UART_Receive_IT(&huart4, (uint8_t *)&bluetooth_received_character, 1);
		vTaskResume(send_message_task_handle);
	}
}
