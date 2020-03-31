#include "theta2dev.h"
/*
 *关节空间到驱动空间的映射模块
 *主要功能：提供了将关节空间映射到驱动空间的函数，每个关节一个函数
double _theta_0_2driver(double theta);//关节0
double _theta_1_2driver(double theta);//关节1
double _theta_2_2driver(double theta);//关节2
...
以上函数输入值为关节角，
输出为调用_pca9685_set_angle(unsigned char channel, double angle)时候的的形参angle的值
返回值同上，若返回值为-1则是非法角度

注意：该驱动因狗而异
 */

double _theta_0_2driver(double theta)
{
    double result;
    //因为狗腿子的机械特性，ZERO_JOINT_0为机械零位的值，即默认theta最小为30度
    result = theta + ZERO_JOINT_0 - 30;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_1_2driver(double theta)
{
    double result;
    //因为狗腿子的机械特性，ZERO_JOINT_0为机械零位的值，即默认theta最小为30度
    result = ZERO_JOINT_1 - theta + 35;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_2_2driver(double theta)
{
    double result;
    //因为狗腿子的机械特性，ZERO_JOINT_0为机械零位的值，即默认theta最小为30度
    result = theta + ZERO_JOINT_2 - 40;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_3_2driver(double theta)
{
    double result;
    //因为狗腿子的机械特性，ZERO_JOINT_0为机械零位的值，即默认theta最小为30度
    result = ZERO_JOINT_3 - theta + 35;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_4_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_4 - theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_5_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_5 + theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_6_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_6 - theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_7_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_7 + theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_8_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_8 - theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_9_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_9 + theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_10_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_10 + theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}

double _theta_11_2driver(double theta)
{
    double result;
    result = ZERO_JOINT_11 - theta;
    if ((result > MAX_SERVO_ANGLE) || result < MIN_SERVO_ANGLE)
        return -720;
    return result;
}
