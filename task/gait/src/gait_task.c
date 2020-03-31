#include "gait_task.h"
//步态任务句柄
TaskHandle_t GaitTaskHandle;
//步态队列（驱动空间）
QueueHandle_t GaitQueue;

/*
*功能：步态任务
*描述：调用步态函数（步态函数输出驱动空间），将驱动空间推送至消息队列
*/
void xGaitTask()
{
    double theta[12];
    double t = 0;
    while (1)
    {
        t++;
        if (t > 25)
            t = 0;

        tort_cycloid(t, 25, 10, 50, theta);
        //stand(theta);
        xQueueOverwrite(GaitQueue, theta);
    }
}

BaseType_t xGaitTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)xGaitTask,
                          (const char *)"GaitTask",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)9,
                          (TaskHandle_t *)GaitTaskHandle);
    return xReturn;
}

void GaitQueueCreat()
{
    GaitQueue = xQueueCreate((UBaseType_t)GAIT_QUEUE_LEN, 12 * sizeof(double));
}
