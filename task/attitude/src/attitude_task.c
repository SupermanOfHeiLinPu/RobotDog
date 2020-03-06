#include "attitude_task.h"
//姿态任务句柄
TaskHandle_t AttitudeTaskHandle;
//姿态消息队列
QueueHandle_t AttitudeQueue = NULL;

//姿态任务
static void AttitudeTsak()
{
    int isReady;
    unsigned char dmp_status;
    attitude *attitude_self;
    attitude_self = attitude_self_Creat();
    attitude_self->mpu6050self->mpuFunList->Init(attitude_self->mpu6050self);
    while (1)
    {
        //判断mpu6050dmp开了没
        mpu_get_dmp_state(&dmp_status);
        if (dmp_status == 0)
        {
            attitude_self->mpu6050self->mpuFunList->Init(attitude_self->mpu6050self);
            continue;
        }
        //读取四元数
        isReady = attitude_self->mpu6050self->mpuFunList->ReadData(attitude_self->mpu6050self);
        attitude_self->quat[0] = attitude_self->mpu6050self->quat[0];
        attitude_self->quat[1] = attitude_self->mpu6050self->quat[1];
        attitude_self->quat[2] = attitude_self->mpu6050self->quat[2];
        attitude_self->quat[3] = attitude_self->mpu6050self->quat[3];
        //转化为欧拉角
        attitude_self->FunList->bulid_euler_angles(attitude_self);
        if (!isReady)
        {
            xQueueOverwrite(AttitudeQueue, &attitude_self);
            printf("pitch:%lf\n", attitude_self->pitch);
            printf("yaw:%lf\n", attitude_self->yaw);
            printf("roll:%lf\n", attitude_self->roll);
        }
    }
    attitude_self_delete(attitude_self);
}

BaseType_t xAttitudeTsakCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)AttitudeTsak,
                          (const char *)"AttitudeTsak",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)9,
                          (TaskHandle_t *)AttitudeTaskHandle);

    return xReturn;
}

void AttitudeQueueCreat()
{
    AttitudeQueue = xQueueCreate((UBaseType_t)GESTURE_QUEUE_LEN, sizeof(attitude));
}
