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
    float euler_angles[3];
    //创建姿态对象
    attitude_self = attitude_self_Creat();
    //进入临界区
    taskENTER_CRITICAL();
    //初始化mpu6050
    attitude_self->mpu6050self->mpuFunList->Init(attitude_self->mpu6050self);
    //退出临界区
    taskEXIT_CRITICAL();
    while (1)
    {
        //判断mpu6050dmp开了没
        mpu_get_dmp_state(&dmp_status);
        if (dmp_status == 0)
        {
            //进入临界区
            taskENTER_CRITICAL();
            //如果没打开dmp则重新初始化
            attitude_self->mpu6050self->mpuFunList->Init(attitude_self->mpu6050self);
            //退出临界区
            taskEXIT_CRITICAL();
            //延迟一会
            attitude_self->mpu6050self->mpuFunList->mpu_delay(5);
            continue;
        }
        //进入临界区
        taskENTER_CRITICAL();
        //读取四元数
        isReady = attitude_self->mpu6050self->mpuFunList->ReadData(attitude_self->mpu6050self);
        //退出临界区
        taskEXIT_CRITICAL();
        if (!isReady)
        {
            //将mpu6050 dmp的四元数拷贝过来
            attitude_self->quat[0] = attitude_self->mpu6050self->quat[0];
            attitude_self->quat[1] = attitude_self->mpu6050self->quat[1];
            attitude_self->quat[2] = attitude_self->mpu6050self->quat[2];
            attitude_self->quat[3] = attitude_self->mpu6050self->quat[3];
            //转化为欧拉角
            attitude_self->FunList->bulid_euler_angles(attitude_self);
            euler_angles[0] = attitude_self->pitch;
            euler_angles[1] = attitude_self->roll;
            euler_angles[2] = attitude_self->yaw;
#if (ATTITUDE_QUEUE_LEN < 1)
#error "GESTURE_QUEUE_LEN must more than 1"
#endif
#if (ATTITUDE_QUEUE_LEN == 1)
            //推送至消息队列
            xQueueOverwrite(AttitudeQueue, attitude_self);
#else
            //推送至消息队列
            xQueueSend(AttitudeQueue, euler_angles, portMAX_DELAY);
#endif
            //打印至串口
            /*printf("q0:%lf\n", attitude_self->quat[0]);
            printf("q1:%lf\n", attitude_self->quat[1]);
            printf("q2:%lf\n", attitude_self->quat[2]);
            printf("q3:%lf\n", attitude_self->quat[3]);*/
            printf("p:%d\n", (int)attitude_self->pitch);
            printf("y:%d\n", (int)attitude_self->yaw);
            printf("r:%d\n", (int)attitude_self->roll);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
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
                          (UBaseType_t)5,
                          (TaskHandle_t *)AttitudeTaskHandle);

    return xReturn;
}

void AttitudeQueueCreat()
{
    AttitudeQueue = xQueueCreate((UBaseType_t)ATTITUDE_QUEUE_LEN, 3 * sizeof(float));
}
