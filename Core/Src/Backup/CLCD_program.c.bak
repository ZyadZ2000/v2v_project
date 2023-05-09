/**************************************************************************************/
/**************************************************************************************/
/********************      		Author  : Mazen gasser         ************************/
/********************     		Layer   : HAL                  ************************/
/********************      		SWC     : CLCD                 ************************/
/********************      		Version : 1.00                 ************************/
/**************************************************************************************/
/**************************************************************************************/




#include "STD_TYPES.h"

#include "stm32f4xx_hal.h"

#include "CLCD_interface.h"
#include "CLCD_private.h"
#include "CLCD_config.h"

#include "delay.h"


//SEND COMMAND TO LCD//
static void CLCD_voidSendCommand(LCD_Data Copy_u8Command)
{
	//ACTIVATE SENDING COMMAND(RS)//
	 HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_RS_PIN , GPIO_PIN_RESET);

	//ACTIVATE WRITING(RW)//
	 HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_RW_PIN , GPIO_PIN_RESET);

	//SEND COMMAND//
	 HAL_GPIO_WritePin(CLCD_D4_PORT , CLCD_D4_PIN , (u8)Copy_u8Command.Pin.D4);
	 HAL_GPIO_WritePin(CLCD_D5_PORT , CLCD_D5_PIN , (u8)Copy_u8Command.Pin.D5);
	 HAL_GPIO_WritePin(CLCD_D6_PORT , CLCD_D6_PIN , (u8)Copy_u8Command.Pin.D6);
	 HAL_GPIO_WritePin(CLCD_D7_PORT , CLCD_D7_PIN , (u8)Copy_u8Command.Pin.D7);
	//MGPIO_u8SetPortValue(CLCD_DATA_PORT , Copy_u8Command);

	//ENABLE PULSE WITH 2 MS DELAY//
	 HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_E_PIN , GPIO_PIN_SET);

	DELAY_MS(2);
	HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_E_PIN , GPIO_PIN_RESET);
}


//SEND DATA TO LCD//
static void CLCD_voidSendData(LCD_Data Copy_u8Data)
{

	//ACTIVATE SENDING COMMAND(RS)//
	HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_RS_PIN , GPIO_PIN_SET);

	//ACTIVATE WRITING(RW)//
	HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_RW_PIN , GPIO_PIN_RESET);

	//SEND COMMAND//
	HAL_GPIO_WritePin(CLCD_D4_PORT , CLCD_D4_PIN , (u8)Copy_u8Data.Pin.D4);
	HAL_GPIO_WritePin(CLCD_D5_PORT , CLCD_D5_PIN , (u8)Copy_u8Data.Pin.D5);
	HAL_GPIO_WritePin(CLCD_D6_PORT , CLCD_D6_PIN , (u8)Copy_u8Data.Pin.D6);
	HAL_GPIO_WritePin(CLCD_D7_PORT , CLCD_D7_PIN , (u8)Copy_u8Data.Pin.D7);
	//MGPIO_u8SetPortValue(CLCD_DATA_PORT , Copy_u8Data);

	//ENABLE PULSE WITH 2 MS DELAY//
	HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_E_PIN , GPIO_PIN_SET);

	DELAY_MS(2);

	HAL_GPIO_WritePin(CLCD_CTRL_PORT , CLCD_E_PIN , GPIO_PIN_RESET);
}


//INTIALIZATION OF LCD//
void CLCD_voidInit(void)
{
	LCD_Data DataPins;

	//WAIT FOR MORE THAN 40 MS //
	DELAY_MS(40);


	//FUNCTION SET : 1 LINE , 5*8 FONT SIZE//
	//CLCD_voidSendCommand(0b00110000);
	DataPins.Pins = 0b0010;
	CLCD_voidSendCommand(DataPins);
	CLCD_voidSendCommand(DataPins);
	CLCD_voidSendCommand(DataPins);

	//DISPLAY ON/OFF CONTROL : DISPLAY ENABLED , CURSOR NOT ENABLED , BLINK CURSOR NOT ENABLED//
	//CLCD_voidSendCommand(0b00001100);
	DataPins.Pins = 0b0000;
	CLCD_voidSendCommand(DataPins);

	DataPins.Pins = 0b1100;
	CLCD_voidSendCommand(DataPins);

	//DISPLAY CLEAR//
	//CLCD_voidSendCommand(0b00000001);
	CLCD_voidDisplayClear();

	//ACTIVATE WRITING ON 2 LINES//
	DataPins.Pins = 0b0010;
	CLCD_voidSendCommand(DataPins);

	DataPins.Pins = 0b1000;
	CLCD_voidSendCommand(DataPins);
	//CLCD_voidSendCommand(0X38);
}


void CLCD_voidDisplayClear(void)
{
	LCD_Data DataPins;

	//DISPLAY CLEAR//
	//CLCD_voidSendCommand(0b00000001);
	DataPins.Pins = 0b0000;
	CLCD_voidSendCommand(DataPins);

	DataPins.Pins = 0b0001;
	CLCD_voidSendCommand(DataPins);

}




//SEND STRING DATA TO LCD//
void CLCD_voidSendString(const char *Copy_pcString)
{
	u8 Local_u8Counter = 0;
	LCD_Data DataPins;

	//PRINT EACH CHAR UNTIL IT REACH NULL CHAR//
	while(Copy_pcString[Local_u8Counter] != '\0')
	{
		//SEND CHAR BY CHAR TO THE END OF THE STRING//
		DataPins.Pins = Copy_pcString[Local_u8Counter] >> 4;
		CLCD_voidSendData(DataPins);

		DataPins.Pins = Copy_pcString[Local_u8Counter];
		CLCD_voidSendData(DataPins);

		Local_u8Counter++;
	}
}




//GO TO SPECIFIC POSITION//
void CLCD_voidGoToXY(u8 Copy_u8Xpos , u8 Copy_u8Ypos)
{
	u8 Local_u8Address;
	LCD_Data DataPins;

	//LOCATION IN THE FRIST LINE //
	if (Copy_u8Xpos == 0)
	{
		Local_u8Address = Copy_u8Ypos;
	}

	//LOCATION IN THE SECOND LINE //
	else if(Copy_u8Xpos == 1)
	{
		Local_u8Address = Copy_u8Ypos + 0X40;
	}

	Local_u8Address |= 0X80;

	DataPins.Pins = Local_u8Address >> 4;
	CLCD_voidSendCommand(DataPins);

	DataPins.Pins = Local_u8Address;
	CLCD_voidSendCommand(DataPins);

	//SET DDRAM ADDRESS COMMAND //
	//CLCD_voidSendCommand(Local_u8Address|0X80);

}




//WRITE SPECIAL CHAR//
void CLCD_voidWriteSpeacialChar(u8 *Copy_u8Arr , u8 Copy_u8BlockNum , u8 Copy_u8Xpos , u8 Copy_u8Ypos)
{
	u8 Local_u8CGRAMAddress , Local_u8Iter;
	LCD_Data DataPins;


	//SPECIFY CGRAM ADDRESS//
	Local_u8CGRAMAddress = Copy_u8BlockNum*8;


	//SET DDRAM ADDRESS COMMAND //
	Local_u8CGRAMAddress |= 0X40;

	DataPins.Pins = Local_u8CGRAMAddress >> 4;
	CLCD_voidSendCommand(DataPins);

	DataPins.Pins = Local_u8CGRAMAddress;
	CLCD_voidSendCommand(DataPins);
	//CLCD_voidSendCommand(Local_u8CGRAMAddress|0X40);


	//SAVING PATTERN IN CGRAM BY SENDING EACH BYTE//
	for(Local_u8Iter = 0 ; Local_u8Iter<8 ; Local_u8Iter++)
	{
		DataPins.Pins = Copy_u8Arr[Local_u8Iter] >> 4;
		CLCD_voidSendData(DataPins);

		DataPins.Pins = Copy_u8Arr[Local_u8Iter];
		CLCD_voidSendData(DataPins);
	}


	//GO TO SPECIFIC POSITION//
	CLCD_voidGoToXY(Copy_u8Xpos , Copy_u8Ypos);



	//SEND DATA TO LCD//
	DataPins.Pins = Copy_u8BlockNum >> 4;
	CLCD_voidSendData(DataPins);

	DataPins.Pins = Copy_u8BlockNum;
	CLCD_voidSendData(DataPins);
	//CLCD_voidSendData(Copy_u8BlockNum);

}




//WRITE NUMBER ON LCD//
void CLCD_voidWriteNumber(u32 Copy_u32Number)
{

	u32 Local_u32ReversedNum = 0 , Local_u32Temp = Copy_u32Number;
	u8 Local_u8mod , Local_u8Digits = 0 , Local_u8DigitsPrint = 0;
	LCD_Data DataPins;

	//special case if entered number is 0//
	if (Copy_u32Number == 0)
	{
		DataPins.Pins = (u8) '0' >> 4;
		CLCD_voidSendData(DataPins);

		DataPins.Pins = (u8) '0';
		CLCD_voidSendData(DataPins);
	}
	else
	{
		//count number of digits for entered number//
		while (Local_u32Temp > 0)
		{
			Local_u32Temp /= 10;
			Local_u8Digits++;
		}

		Local_u32Temp = Copy_u32Number;

		//reverse the entered number to print each number in its right order//
		while(Local_u32Temp > 0)
		{
			Local_u8mod = Local_u32Temp%10;

			Local_u32ReversedNum = Local_u32ReversedNum*10 + Local_u8mod;

			Local_u32Temp/=10;
		}

		//print each number in its right order//
		while(Local_u32ReversedNum > 0)
		{
			Local_u8mod = Local_u32ReversedNum%10;

			DataPins.Pins = (u8)((Local_u8mod+'0') >> 4);
			CLCD_voidSendData(DataPins);

			DataPins.Pins = (u8) (Local_u8mod+'0');
			CLCD_voidSendData(DataPins);

			Local_u32ReversedNum/=10;

			Local_u8DigitsPrint++;
		}

		//check if the printed digits equal number digits
		while(Local_u8DigitsPrint != Local_u8Digits)
		{
			DataPins.Pins = (u8) ('0' >> 4);
			CLCD_voidSendData(DataPins);

			DataPins.Pins = (u8) '0';
			CLCD_voidSendData(DataPins);

			Local_u8DigitsPrint++;
		}

	}
}
