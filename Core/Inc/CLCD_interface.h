/**************************************************************************************/
/**************************************************************************************/
/********************      		Author  : Mazen gasser         ************************/
/********************     		Layer   : HAL                  ************************/
/********************      		SWC     : CLCD                 ************************/
/********************      		Version : 1.00                 ************************/
/**************************************************************************************/
/**************************************************************************************/



#ifndef CLCD_INTERFACE_H_
#define CLCD_INTERFACE_H_

#include "STD_TYPES.h"

//INTIALIZATION OF LCD//
void CLCD_voidInit(void);


/* Clear Display */
void CLCD_voidDisplayClear(void);


//SEND STRING DATA TO LCD//
void CLCD_voidSendString(const char *Copy_pcString);


//GO TO SPECIFIC POSITION//
void CLCD_voidGoToXY(u8 Copy_u8Xpos , u8 Copy_u8Ypos);


//WRITE SPECIAL CHAR//
void CLCD_voidWriteSpeacialChar(u8 *Copy_u8Arr , u8 Copy_u8BlockNum , u8 Copy_u8Xpos , u8 Copy_u8Ypos);


//WRITE NUMBER ON LCD//
void CLCD_voidWriteNumber(u32 Copy_u32Number);

#endif


