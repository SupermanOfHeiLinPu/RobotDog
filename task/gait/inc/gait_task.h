#ifndef __GAIT_TASK_H__
#define __GAIT_TASK_H__

#include "gait.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"

#define GAIT_QUEUE_LEN (8)

extern TaskHandle_t GaitTaskHandle;
extern QueueHandle_t GaitQueue;

extern BaseType_t xGaitTaskCreat();
extern void GaitQueueCreat();
#endif
