#include "gait_task.h"
//步态任务句柄
TaskHandle_t GaitTaskHandle;
//步态队列（驱动空间）
QueueHandle_t GaitQueue;

/*
 
 */
int go_straight(uint32_t current_time, double T, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 10, 50, theta, 0);
    return 0;
}

/*

*/
int go_right(uint32_t current_time, double T, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 10, 20, theta, 1);
    return 0;
}

int go_left(uint32_t current_time, double T, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 10, 20, theta, 2);
    return 0;
}

int go_back(uint32_t current_time, double T, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 10, 50, theta, 3);
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
    while (1)
    {
        xQueueReceive(AttitudeQueue, euler_angles, 0);
        /*printf("pitch:%f\n", euler_angles[0]);
        printf("yaw:%f\n", euler_angles[2]);
        printf("roll:%f\n", euler_angles[1]);*/
        _PS2_Read_Data(ps2_data);
        //printf("%d\n", ps2_data[3]);
        t = xTaskGetTickCount();
        if (ps2_data[3] == 255)
        {
            stand(theta);
        }
        else if (ps2_data[3] == 239)
        {
            go_straight(t, 250, theta);
        }
        else if (ps2_data[3] == 127)
        {
            go_left(t, 250, theta);
        }
        else if (ps2_data[3] == 223)
        {
            go_right(t, 250, theta);
        }
        else if (ps2_data[3] == 191)
        {
            go_back(t, 250, theta);
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
