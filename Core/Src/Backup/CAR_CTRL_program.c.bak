/***************************************************************************************/
/***************************************************************************************/
/********************			Author	:	GP TEAM				************************/
/********************			Layer	:	HAL					************************/
/********************			SWC		:	CAR_CTRL			************************/
/********************			Version	:	1.00				************************/
/***************************************************************************************/
/***************************************************************************************/


#include "STD_TYPES.h"
#include "stm32f4xx_hal.h"

#include "CAR_CTRL_config.h"
#include "CAR_CTRL_interface.h"


/* Move RC Car To The Forward Direction */
void HAL_CAR_CTRL_voidForward(void)
{
	/* Activate Pins To Be High | Low To Match The Direction */
	//MCAL_GPIO_u8SetPinValue(IN_1_GPIO_PORT, IN_1_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	//MCAL_GPIO_u8SetPinValue(IN_2_GPIO_PORT, IN_2_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_3_GPIO_PORT, IN_3_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);

	//MCAL_GPIO_u8SetPinValue(IN_4_GPIO_PORT, IN_4_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}


/* Move RC Car To The Backward Direction */
void HAL_CAR_CTRL_voidBackward(void)
{
	/* Activate Pins To Be High | Low To Match The Direction */
	//MCAL_GPIO_u8SetPinValue(IN_1_GPIO_PORT, IN_1_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_2_GPIO_PORT, IN_2_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
	//MCAL_GPIO_u8SetPinValue(IN_3_GPIO_PORT, IN_3_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	//MCAL_GPIO_u8SetPinValue(IN_4_GPIO_PORT, IN_4_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

}


/* Move RC Car To The Right Direction */
void HAL_CAR_CTRL_voidRight(void)
{
	/* Activate Pins To Be High | Low To Match The Direction */
	//MCAL_GPIO_u8SetPinValue(IN_1_GPIO_PORT, IN_1_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_2_GPIO_PORT, IN_2_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
//	MCAL_GPIO_u8SetPinValue(IN_3_GPIO_PORT, IN_3_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_4_GPIO_PORT, IN_4_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);

}


/* Move RC Car To The Left Direction */
void HAL_CAR_CTRL_voidLeft(void)
{	
	/* Activate Pins To Be High | Low To Match The Direction */
	//MCAL_GPIO_u8SetPinValue(IN_1_GPIO_PORT, IN_1_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
	//MCAL_GPIO_u8SetPinValue(IN_2_GPIO_PORT, IN_2_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_3_GPIO_PORT, IN_3_GPIO_PIN, GPIO_PIN_HIGH);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
	//MCAL_GPIO_u8SetPinValue(IN_4_GPIO_PORT, IN_4_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);


}


/* Stop RC Car In The Current Position */
void HAL_CAR_CTRL_voidStop(void)
{
	/* Activate Pins To Be High | Low To Match The Direction */
	//MCAL_GPIO_u8SetPinValue(IN_1_GPIO_PORT, IN_1_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_2_GPIO_PORT, IN_2_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_3_GPIO_PORT, IN_3_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
	//MCAL_GPIO_u8SetPinValue(IN_4_GPIO_PORT, IN_4_GPIO_PIN, GPIO_PIN_LOW);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);

	
	
}





