#include "gps.h"
#include "NMEA.h"
#include "CLCD_interface.h"
#include "uartRingBuffer.h"

extern GPSSTRUCT GGAST;

extern double my_car_latitude;
extern double my_car_longitude;
extern char north_south;
extern char east_west;

extern ring_buffer *_rx_buffer;

void GPS_init(void) {
	CLCD_voidDisplayClear();
	CLCD_voidGoToXY(0, 0);
	CLCD_voidSendString("Waiting for");
	CLCD_voidGoToXY(1, 0);
	CLCD_voidSendString("GPS ");
	uint8_t flagGGA = 1;
	uint8_t c1;
	int8_t GGA[100];
	//char count = 0;
	//double latitude_sum = 0.0;
	//double longitude_sum = 0.0;
	uint8_t GGA_loop_index;
	//char uartBuflong[100] ={0};
	//char uartBuflat[100] ={0};
	do {

		for (GGA_loop_index = 0; GGA_loop_index < 100; GGA_loop_index++) {
			GGA[(int) GGA_loop_index] = 0;
		}
		// extracting the message ($gga)
		//HAL_UART_Transmit(&huart3,(uint8_t *)"extracting the message", 23, 100);

		//MCAL_UART_u8ReceiveData(UART_3, (uint8_t *)&c);
		HAL_UART_Receive(&huart3, (uint8_t*) &c1, sizeof(c1), 1000);
		// start with the dollar sign if not then loop to find it
		while (c1 != '$') {
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

		// HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n", 2, 100);
		// HAL_UART_Transmit(&huart3,(uint8_t *) "Before enter do while ", 23, 100);
		//  HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n", 2, 100);
		//HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n", 2, 100);
		//HAL_UART_Transmit(&huart3,(uint8_t *) "inside do while ", 17, 100);
		flagGGA = 1;

		if (Wait_for("GGA") == 1) {
			// HAL_UART_Transmit(&huart3,(uint8_t *) "found gga ", 11, 100);
			//HAL_UART_Transmit(&huart3,(uint8_t *)"\r\n", 2, 100);
			//HAL_UART_Transmit(&huart3,(uint8_t *) "outside wait ", 13, 100);
			Copy_upto("*", GGA);
			if (decodeGGA(GGA, &GGAST.ggastruct) == 0) {

				//latitude_sum += GGAST.ggastruct.lcation.latitude;
				//longitude_sum += GGAST.ggastruct.lcation.longitude;
				// count++;
				//if (count == 1) {
				flagGGA = 0;
				//}

			}
		}

	} while (flagGGA == 1);

	CLCD_voidDisplayClear();       //clear LCD screen

	// in using freertos
	// here is the delay ------

	my_car_latitude = GGAST.ggastruct.lcation.latitude;	//latitude_sum ;/// 2;//* 100;
	my_car_longitude = GGAST.ggastruct.lcation.longitude;//longitude_sum;// / 2; //* 100;
	north_south = GGAST.ggastruct.lcation.NS;
	east_west = GGAST.ggastruct.lcation.EW;
// here we will write on lcd "now gps is working and system is activated "
}
