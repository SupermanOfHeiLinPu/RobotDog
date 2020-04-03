#ifndef __ATTITUDE_TASK_H_
#define __ATTITUDE_TASK_H_

#include <stdio.h>
#include "attitude.h"
#include "task.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"

#define ATTITUDE_QUEUE_LEN 2

//姿态任务句柄
extern TaskHandle_t AttitudeTaskHandle;
//姿态消息队列
extern QueueHandle_t AttitudeQueue;

BaseType_t xAttitudeTsakCreat();
void AttitudeQueueCreat();

#endif
