#ifndef __MOTOR_TASK_H__
#define __MOTOR_TASK_H__

#include <stdint.h>
#include "stm32f7xx.h"
#include "tim.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "task.h"

#define MOTOR1_TIM_HANDLE (&htim2)
#define MOTOR2_TIM_HANDLE (&htim2)

#define MOTOR1_PWM_CH TIM_CHANNEL_1
#define MOTOR2_PWM_CH TIM_CHANNEL_2

extern BaseType_t xMotorTaskCreat();

#endif