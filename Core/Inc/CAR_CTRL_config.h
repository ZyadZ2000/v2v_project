/***************************************************************************************/
/***************************************************************************************/
/********************			Author	:	GP TEAM				************************/
/********************			Layer	:	HAL					************************/
/********************			SWC		:	CAR_CTRL			************************/
/********************			Version	:	1.00				************************/
/***************************************************************************************/
/***************************************************************************************/


#ifndef CAR_CTRL_CONFIG_H_
#define CAR_CTRL_CONFIG_H_



#define SPEED_DECREASE_RATE					10


/****************************************************************/
/*					SELECT Bluetooth UART Number:				*/
/*																*/
/*		Options :			0-USART_1							*/
/*							1-UART_2							*/
/*							2-UART_3							*/
/*							3-UART_4							*/
/*							4-UART_5							*/
/*							5-USART_6							*/
/*																*/
/****************************************************************/
#define BLUETOOTH_UART_NUMBER				UART_4


/****************************************************************/
/*					SELECT GPIO PORT Nubmer:					*/
/*																*/
/*		Options :			0-GPIO_PORTA						*/
/*							1-GPIO_PORTB						*/
/*							2-GPIO_PORTC						*/
/*							3-GPIO_PORTD						*/
/*							4-GPIO_PORTH						*/
/*																*/
/****************************************************************/
#define IN_1_GPIO_PORT						GPIO_PORTC
#define IN_2_GPIO_PORT						GPIO_PORTC
#define IN_3_GPIO_PORT						GPIO_PORTC
#define IN_4_GPIO_PORT						GPIO_PORTC


/****************************************************************/
/*					SELECT GPIO PIN Nubmer:						*/
/*																*/
/*		Options :		GPIO_PIN0 -> GPIO_PIN15					*/
/*																*/
/****************************************************************/
#define IN_1_GPIO_PIN						GPIO_PIN2
#define IN_2_GPIO_PIN						GPIO_PIN3
#define IN_3_GPIO_PIN						GPIO_PIN14
#define IN_4_GPIO_PIN						GPIO_PIN15



#endif