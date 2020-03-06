#ifndef __ATTITUDE_H__
#define __ATTITUDE_H__

#include "mpu6050.h"
#include <math.h>
struct attitudeVtbl;

typedef struct _attitude
{
    mpu6050 *mpu6050self;
    struct attitudeVtbl *FunList;
    double quat[4];
    double pitch;
    double roll;
    double yaw;
} attitude;

struct attitudeVtbl
{
    int (*bulid_euler_angles)(attitude *self);
    int (*get_quaternion)(attitude *self);
};

//接口函数
attitude *attitude_self_Creat();
void attitude_self_delete(attitude *self);

#endif
