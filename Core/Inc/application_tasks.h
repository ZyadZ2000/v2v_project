#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define  SS_VELOCITY_THRESHOLD  -10

void Task_speedCalculation(void *parameters);
void Task_sendMessage(void *parameters);
void Task_handleReceivedMessage(void * parameters);

#endif
