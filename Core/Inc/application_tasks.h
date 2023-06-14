#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define  SS_VELOCITY_THRESHOLD  -1500.0

void Task_speedCalculation(void *parameters);
void Task_sendMessage(void *parameters);
void Task_handleReceivedMessage(void * parameters);
void Task_readingGPS(void *parameters);
void Task_directionOfCar(void * parameters);
void Task_controlCar(void *parameters);
void Task_touchScreen(void *parameters);
void Task_arrestMessageHandler(void *parameters);

#endif
