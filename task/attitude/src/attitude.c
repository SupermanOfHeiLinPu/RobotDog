#include "attitude.h"
/**
*姿态解算模块
*定义了attitude变量类型,该类型继承了mpu6050,使用示例：
    attitude name;
    attitude *name_ptr;
*提供了创建、删除attitude对象的函数：
    *attitude_self_Creat();返回attitude对象指针（默认打开了mpu的dmp）
    attitude_self_delete(attitude *self);将对象指针传入
*使用了FreeRTOS提供的pvPortMalloc和vPortFree。
*attitude的属性：
    quat[4];四元数
    pitch;俯仰角
    roll;滚筒角
    yaw;偏航角
*attitude提供的方法：
    bulid_euler_angles(attitude *self);将四元数更新为欧拉角
    get_quaternion(attitude *self);从原始数据更新四元数（P.S.未完成）
使用示例：
    attitude *attitude_self;
    attitude_self = attitude_self_Creat();
    attitude_self->mpu6050self->mpuFunList->ReadData(attitude_self->mpu6050self);//从mpu6050的dmp更新四元数
    attitude_self->quat[0] = attitude_self->mpu6050self->quat[0];
    attitude_self->quat[1] = attitude_self->mpu6050self->quat[1];
    attitude_self->quat[2] = attitude_self->mpu6050self->quat[2];
    attitude_self->quat[3] = attitude_self->mpu6050self->quat[3];
    attitudeself->FunList->bulid_euler_angles();//将四元数转为欧拉角  
*
**/
/*
 *功能：将四元数转化为欧拉角
 *输入：attitude对象指针
 *输出：欧拉角
 */
static int _attitudeBulidEulerAngles(attitude *self)
{
    double q0, q1, q2, q3;

    q0 = self->quat[0];
    q1 = self->quat[1];
    q2 = self->quat[2];
    q3 = self->quat[3];

    self->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; //俯仰角

    self->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; //横滚角

    self->yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; //航向角
    return 0;
}

/**
* 功能：将接收到的角速度数据转化为实际值
* 输入：
* 输出：
* 引用：_attitudeRungeKutta1()函数
TODO:
**/

static int _attitudeGetTrueAngularVelocity(attitude *self, double *wx_ptr, double *wy_ptr, double *wz_ptr)
{
    return 0;
}

/**
* 功能：一阶段龙格库塔解算四元数
* 输入：
* 输出：
* 注意使用前必须获取一次时间戳，即self->mpu6050self->timestamp
TODO:将采集到的数据转化为真实数据（wx,wy,wz）；四元数初始化；误差补偿
**/
static int _attitudeRungeKutta1(attitude *self)
{
    static double q0, q1, q2, q3;
    double wx = 0, wy = 0, wz = 0;
    double deltaT;
    static unsigned long last_time;

    _attitudeGetTrueAngularVelocity(self, &wx, &wy, &wz);
    deltaT = self->mpu6050self->timestamp - last_time;

    q0 += 0.5 * (-wx * q1 - wy * q2 - wz * q3) * deltaT;
    q1 += 0.5 * (wx * q0 - wy * q3 + wz * q2) * deltaT;
    q2 += 0.5 * (wx * q3 + wy * q0 - wz * q1) * deltaT;
    q3 += 0.5 * (-wx * q0 + wy * q3 + wz * q2) * deltaT;

    self->quat[0] = q0;
    self->quat[1] = q1;
    self->quat[2] = q2;
    self->quat[3] = q3;

    last_time = self->mpu6050self->timestamp;
    return 0;
}

//姿态的虚函数表
static struct attitudeVtbl _attitudeFunList = {
    .bulid_euler_angles = _attitudeBulidEulerAngles,
    .get_quaternion = _attitudeRungeKutta1};

//接口函数
attitude *attitude_self_Creat()
{
    attitude *self;
    self = pvPortMalloc(sizeof(attitude));

    self->mpu6050self = MPU6050_Self_Creat(0x68, 1);
    self->FunList = &_attitudeFunList;
    return self;
}

void attitude_self_delete(attitude *self)
{
    vPortFree(self);
}
