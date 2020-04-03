#include "gait_task.h"
//步态任务句柄
TaskHandle_t GaitTaskHandle;
//步态队列（驱动空间）
QueueHandle_t GaitQueue;

/*
 
 */
int go_straight(uint32_t current_time, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > 250)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, 250, 10, 40, theta, 0);
    return 0;
}

/*
*功能：步态任务
*描述：调用步态函数（步态函数输出驱动空间），将驱动空间推送至消息队列
*/
void xGaitTask()
{
    double theta[12];
    uint32_t t;
    float euler_angles[3];
    unsigned char ps2_data[9];
    BaseType_t queue_statue;
    while (1)
    {
        queue_statue = xQueueReceive(AttitudeQueue, euler_angles, 0);
        if (queue_statue != pdTRUE)
            continue;
        printf("pitch:%f\n", euler_angles[0]);
        printf("yaw:%f\n", euler_angles[2]);
        printf("roll:%f\n", euler_angles[1]);
        _PS2_Read_Data(ps2_data);
        //printf("%d\n", ps2_data[3]);
        t = xTaskGetTickCount();
        if (ps2_data[3] == 255)
        {
            stand(theta);
        }
        else
        {
            go_straight(t, theta);
        }

        xQueueSend(GaitQueue, theta, portMAX_DELAY);
    }
}

BaseType_t xGaitTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)xGaitTask,
                          (const char *)"GaitTask",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)5,
                          (TaskHandle_t *)GaitTaskHandle);
    return xReturn;
}

void GaitQueueCreat()
{
    GaitQueue = xQueueCreate((UBaseType_t)GAIT_QUEUE_LEN, 12 * sizeof(double));
}
