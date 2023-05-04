#include "ultrasonic.h"
#include <stdint.h>
#include "main.h"

extern uint8_t icFlag;
extern uint32_t edge1Time;
extern uint32_t edge2Time;

double getUltrasonicDistance(void) {
	/*
	 note : don't use this code before initialize (any timer) to use it as input capture unit
	 configrations :
	 1- psc = frequency - 1 .. to make frequency 1 mega hz , and period 1 micro second
	 2- counter mode : up
	 3- counter period :1000000
	 4- polarity selection : both directions
	 5- input filter : 4 (to remove noise)
	 6- enable interrupt
	 */
	// configure any pin to be an output for trigger then modify the above #define and write the pin chosen
	// as initialization
	edge1Time = edge2Time = 0;
	float distance = 2.0;

	//Set TRIG to LOW for 3 uSec
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	//*** START Ultrasonic measure routine ***//
	//1. Output 10 usec TRIG
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

	//2. calculate ECHO signal pulse width using input capture unit in the timer
	//Start IC timer
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);

	// note : the logic to cakculate edge 1 and 2 is in HAL_TIM_IC_CaptureCallback

	//Wait for IC flag
	uint32_t startTick = HAL_GetTick();
	//int counter =0;
	do {
		//counter++;
		if (icFlag)
			break;
	} while ((HAL_GetTick() - startTick) < 500);  //500ms , time out condition
	//while (counter <1000000 );
	icFlag = 0;
	HAL_TIM_IC_Stop_IT(&htim4, TIM_CHANNEL_1);

	//Calculate distance in cm
	if (edge2Time > edge1Time) {
		return ((edge2Time - edge1Time) + 0.0f) * speedOfSound;
	} else {
		// error
		return 0.0f;

	}
}
