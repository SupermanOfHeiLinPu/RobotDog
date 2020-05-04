#include "gait_task.h"
//步态任务句柄
TaskHandle_t GaitTaskHandle;
//步态队列（驱动空间）
QueueHandle_t GaitQueue;

/*
 *站立姿态PID结构体定义，并初始化
 */
static struct _stand_PID
{
    double Kp;
    double Ki;
    double Kd;
    double pek;
    double pek_1;
    double rek;
    double rek_1;
    double perr_integral;
    double rerr_integral;
} stand_PID = {
    .Kp = 0.5,
    .Ki = 0.00001,
    .Kd = 1,
    .pek_1 = 0,
    .rek_1 = 0,
    .perr_integral = 0,
    .rerr_integral = 0};

/*
 *功能：走直线
 *输入：
    current_time：当前时间
    T：周期
    theta：驱动空间地址
*输出：
    theta：驱动空间
 */
static int go_straight(uint32_t current_time, double T, double pitch, double roll, double theta[12], int is_walk)
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    if (is_walk)
    {
        walk_cycloid(deltaT, T, 25, 45, pitch, roll, theta, 0);
    }
    else
    {
        tort_cycloid(deltaT, T, 25, 45, pitch, roll, theta, 0);
    }
    return 0;
}

/*
*右转
*输入：
    current_time：当前时间
    T：周期
    theta：驱动空间地址
*输出：
    theta：驱动空间
*/
static int go_right(uint32_t current_time, double T, double pitch, double roll, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 25, 45, pitch, roll, theta, 1);
    return 0;
}

/*
 *左转
 *输入：
    current_time：当前时间
    T：周期
    theta：驱动空间地址
 *输出：
    theta：驱动空间
 */
static int go_left(uint32_t current_time, double T, double pitch, double roll, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 25, 45, pitch, roll, theta, 2);
    return 0;
}

/*
*后退
*输入：
    current_time：当前时间
    T：周期
    theta：驱动空间地址
*输出：
    theta：驱动空间
*/
static int go_back(uint32_t current_time, double T, double pitch, double roll, double theta[12])
{
    uint32_t deltaT;
    static uint32_t lastT;
    deltaT = current_time - lastT;
    if (deltaT > T)
    {
        lastT = current_time;
        deltaT = 0;
    }
    tort_cycloid(deltaT, T, 25, 45, pitch, roll, theta, 3);
    return 0;
}
/*
*站立姿态控制
*输入：
    current_pitch，current_roll：当前俯仰角和横滚角
    aimp，aimr：目标俯仰角和横滚角
    theta：驱动空间地址
*输出：
    theta：驱动空间
*/
static int stand_control(double current_pitch, double current_roll, double aimp, double aimr, double theta[12])
{
    double pitch_out, roll_out;
    stand_PID.pek = aimp - current_pitch;
    stand_PID.rek = aimr - current_roll;
    stand_PID.perr_integral += stand_PID.pek;
    stand_PID.rerr_integral += stand_PID.rek;
    pitch_out = stand_PID.Kp * stand_PID.pek + stand_PID.Kd * (stand_PID.pek - stand_PID.pek_1) + stand_PID.Ki * stand_PID.perr_integral;
    roll_out = stand_PID.Kp * stand_PID.rek + stand_PID.Kd * (stand_PID.rek - stand_PID.rek_1) + stand_PID.Ki * stand_PID.rerr_integral;
    stand_PID.pek_1 = stand_PID.pek;
    stand_PID.rek_1 = stand_PID.rek;
    return stand((pitch_out / 180.0) * PI, (roll_out / 180.0) * PI, 0, theta);
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
    unsigned char dog_cmd;
    while (1)
    {
        //接收姿态队列的欧拉角
        xQueueReceive(AttitudeQueue, euler_angles, portMAX_DELAY);
        /*
        PH4为低位,PH5为高位,则
        00:45低
        01:5低4高
        10:5高4低
        11:45高
        */
        dog_cmd = HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_5);
        dog_cmd = dog_cmd << 1;
        dog_cmd = dog_cmd + HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_4);
        //获取当前系统节拍数
        t = xTaskGetTickCount();
        //根据控制指令做动作
        if (dog_cmd == 0)
        {
            go_straight(t, 300, euler_angles[1], euler_angles[0], theta, 0);
        }
        else if (dog_cmd == 1)
        {
            //go_left(t, 700, theta);
            go_straight(t, 300, euler_angles[1], euler_angles[0], theta, 0);
        }
        else if (dog_cmd == 2)
        {
            go_right(t, 500, euler_angles[1], euler_angles[0], theta);
        }
        else if (dog_cmd == 4)
        {
            go_back(t, 700, euler_angles[1], euler_angles[0], theta);
        }
        else if (dog_cmd == 3)
        {
            //由于陀螺仪安装的位置，俯仰角和横滚角反过来
            stand_control(euler_angles[1], euler_angles[0], 0, 0, theta);
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
                          (UBaseType_t)6,
                          (TaskHandle_t *)GaitTaskHandle);
    return xReturn;
}

void GaitQueueCreat()
{
    GaitQueue = xQueueCreate((UBaseType_t)GAIT_QUEUE_LEN, 12 * sizeof(double));
}
