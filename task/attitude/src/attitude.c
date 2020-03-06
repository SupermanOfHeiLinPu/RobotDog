#include "attitude.h"

static int Bulid_euler_angles(attitude *self)
{
    int status;
    double q0, q1, q2, q3;
    status = self->mpu6050self->mpuFunList->ReadData(self->mpu6050self);
    if (status)
        return -1;

    q0 = self->mpu6050self->quat[0];
    q1 = self->mpu6050self->quat[1];
    q2 = self->mpu6050self->quat[2];
    q3 = self->mpu6050self->quat[3];

    self->pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; //俯仰角

    self->roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; //横滚角

    self->yaw = atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3; //航向角
    return status;
}

//姿态的虚函数表
static struct attitudeVtbl _attitudeFunList = {
    .bulid_euler_angles = Bulid_euler_angles};

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
