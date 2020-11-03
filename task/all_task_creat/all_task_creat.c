#include "all_task_creat.h"

TaskHandle_t AllTaskCreatHandle;

void all_task_creat()
{
    //进入临界区
    taskENTER_CRITICAL();
    //创建任务
    AttitudeQueueCreat();
    //GaitQueueCreat();
    xAttitudeTsakCreat();
    xMotorTaskCreat();
    //xGaitTaskCreat();
    //xServoDriverTaskCreat();
    //删除自己
    vTaskDelete(AllTaskCreatHandle);
    //退出临界区
    taskEXIT_CRITICAL();
}

int xAllTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)all_task_creat,
                          (const char *)"AllTask",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)5,
                          (TaskHandle_t *)AllTaskCreatHandle);
    return xReturn;
}
