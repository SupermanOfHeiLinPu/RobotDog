#include "motor_task.h"

extern QueueHandle_t AttitudeQueue;

TaskHandle_t MotorTaskHandle;

static void MotorTask()
{
    float euler_angles[3];
    float rate;
    while (1)
    {
        xQueueReceive(AttitudeQueue, euler_angles, portMAX_DELAY);
        rate = euler_angles[0] / 90.0;
        __HAL_TIM_SET_COMPARE(MOTOR1_TIM_HANDLE, MOTOR1_PWM_CH, (uint32_t)(rate * 1000));
        __HAL_TIM_SET_COMPARE(MOTOR2_TIM_HANDLE, MOTOR2_PWM_CH, (uint32_t)(rate * 1000));
    }
}

BaseType_t xMotorTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)MotorTask,
                          (const char *)"MotorTask",
                          (uint16_t)32,
                          (void *)NULL,
                          (UBaseType_t)5,
                          (TaskHandle_t *)MotorTaskHandle);

    return xReturn;
}
