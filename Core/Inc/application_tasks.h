#ifndef APP_TASKS_H_
#define APP_TASKS_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define  SS_VELOCITY_THRESHOLD  -10

extern volatile uint16_t Global_u16SlitCount;
extern xSemaphoreHandle send_message_semaphore;
extern TaskHandle_t send_message_task_handle;

void Task_speedCalculation(void *parameters);
void Task_sendMessage(void *parameters);

#endif
