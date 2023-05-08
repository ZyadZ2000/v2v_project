#ifndef DELAY_H_
#define DELAY_H_

#include "FreeRTOS.h"

#define DELAY_MS(X)	vTaskDelay((X) / portTICK_RATE_MS)

#endif
