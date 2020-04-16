#include "legInverse_Kinematics.h"

/*
 *功能：单条狗腿子逆运动学第1个解
 *输入：
    px，py，pz：末端空间坐标
    theta：关节空间的地址
 *输出：
    theta：关节空间
 *返回值：0
 */
int _one_leg_invkinematics_1(double px, double py, double pz, double theta[3])
{
    double rxyz, rxy;
    double d2, a2, a3;
    double c3, s3, c1, s1;
    d2 = DOG_LEG_D2;
    a2 = DOG_LEG_A2;
    a3 = DOG_LEG_A3;

    rxy = px * px + py * py;
    rxyz = px * px + py * py + pz * pz;
    c3 = (rxyz - a2 * a2 - a3 * a3 - d2 * d2) / (2 * a2 * a3);
    if (c3 > 1)
    {
        return -720;
    }
    s3 = sqrt(1 - c3 * c3);
    if (rxy < (d2 * d2))
    {
        return -720;
    }
    theta[0] = atan2(py, px) - atan2(-d2, sqrt(rxy - d2 * d2));
    theta[2] = atan2(s3, c3);

    s1 = sin(theta[0]);
    c1 = cos(theta[0]);
    s3 = sin(theta[2]);
    c3 = cos(theta[2]);

    theta[1] = atan2(pz * (a3 + a2 * c3) + a2 * s3 * (px * c1 + py * s1), a3 * (px * c1 + py * s1) + a2 * c3 * (px * c1 + py * s1) - pz * a2 * s3);
    theta[1] -= theta[2];
    return 0;
}

/*
 *功能：单条狗腿子逆运动学第2个解
 *输入：
    px，py，pz：末端空间坐标
    theta：关节空间的地址
 *输出：
    theta：关节空间
 *返回值：0
 */
int _one_leg_invkinematics_2(double px, double py, double pz, double theta[3])
{
    double rxyz, rxy;
    double d2, a2, a3;
    double c3, s3, c1, s1;
    d2 = DOG_LEG_D2;
    a2 = DOG_LEG_A2;
    a3 = DOG_LEG_A3;

    rxy = px * px + py * py;
    rxyz = px * px + py * py + pz * pz;
    c3 = (rxyz - a2 * a2 - a3 * a3 - d2 * d2) / (2 * a2 * a3);
    if (c3 > 1)
    {
        return -720;
    }
    s3 = -sqrt(1 - c3 * c3);
    if (rxy < (d2 * d2))
    {
        return -720;
    }
    theta[0] = atan2(py, px) - atan2(-d2, sqrt(rxy - d2 * d2));
    theta[2] = atan2(s3, c3);

    s1 = sin(theta[0]);
    c1 = cos(theta[0]);
    s3 = sin(theta[2]);
    c3 = cos(theta[2]);

    theta[1] = atan2(pz * (a3 + a2 * c3) + a2 * s3 * (px * c1 + py * s1), a3 * (px * c1 + py * s1) + a2 * c3 * (px * c1 + py * s1) - pz * a2 * s3);
    theta[1] -= theta[2];
    return 0;
}
/*
 *功能：单条狗腿子逆运动学第3个解
 *输入：
    px，py，pz：末端空间坐标
    theta：关节空间的地址
 *输出：
    theta：关节空间
 *返回值：0
 */
int _one_leg_invkinematics_3(double px, double py, double pz, double theta[3])
{
    double rxyz, rxy;
    double d2, a2, a3;
    double c3, s3, c1, s1;
    d2 = DOG_LEG_D2;
    a2 = DOG_LEG_A2;
    a3 = DOG_LEG_A3;

    rxy = px * px + py * py;
    rxyz = px * px + py * py + pz * pz;
    c3 = (rxyz - a2 * a2 - a3 * a3 - d2 * d2) / (2 * a2 * a3);
    if (c3 > 1)
    {
        return -720;
    }
    s3 = sqrt(1 - c3 * c3);
    if (rxy < (d2 * d2))
    {
        return -720;
    }
    theta[0] = atan2(py, px) - atan2(-d2, -sqrt(rxy - d2 * d2));
    theta[2] = atan2(s3, c3);

    s1 = sin(theta[0]);
    c1 = cos(theta[0]);
    s3 = sin(theta[2]);
    c3 = cos(theta[2]);

    theta[1] = atan2(pz * (a3 + a2 * c3) + a2 * s3 * (px * c1 + py * s1), a3 * (px * c1 + py * s1) + a2 * c3 * (px * c1 + py * s1) - pz * a2 * s3);
    theta[1] -= theta[2];
    return 0;
}
/*
 *功能：单条狗腿子逆运动学第4个解
 *输入：
    px，py，pz：末端空间坐标
    theta：关节空间的地址
 *输出：
    theta：关节空间
 *返回值：0
 */
int _one_leg_invkinematics_4(double px, double py, double pz, double theta[3])
{
    double rxyz, rxy;
    double d2, a2, a3;
    double c3, s3, c1, s1;
    d2 = DOG_LEG_D2;
    a2 = DOG_LEG_A2;
    a3 = DOG_LEG_A3;

    rxy = px * px + py * py;
    rxyz = px * px + py * py + pz * pz;
    c3 = (rxyz - a2 * a2 - a3 * a3 - d2 * d2) / (2 * a2 * a3);
    if (c3 > 1)
    {
        return -720;
    }
    s3 = -sqrt(1 - c3 * c3);
    if (rxy < (d2 * d2))
    {
        return -720;
    }
    theta[0] = atan2(py, px) - atan2(-d2, -sqrt(rxy - d2 * d2));
    theta[2] = atan2(s3, c3);

    s1 = sin(theta[0]);
    c1 = cos(theta[0]);
    s3 = sin(theta[2]);
    c3 = cos(theta[2]);

    theta[1] = atan2(pz * (a3 + a2 * c3) + a2 * s3 * (px * c1 + py * s1), a3 * (px * c1 + py * s1) + a2 * c3 * (px * c1 + py * s1) - pz * a2 * s3);
    theta[1] -= theta[2];
    return 0;
}

/*
 *功能：身体姿态的运动学逆解
 *输入：
    theta1~4：四条腿的关节空间地址
    pitch，roll，yaw：欧拉角
    H0：高度
 *输出：
    theta1~4：四条腿的关节空间
*返回值：0
 */
int _body_attitude_invkinematics(double thteta1[3], double thteta2[3],
                                 double thteta3[3], double thteta4[3],
                                 double pitch, double roll, double yaw,
                                 double H0)
{
    double x, y, z;
    //腿1的坐标
    x = DOG_BODY_LEN * sin(pitch) + H0 * cos(pitch) * cos(roll) + DOG_BODY_WEIGHT * cos(pitch) * sin(roll);
    y = DOG_BODY_WEIGHT - DOG_BODY_WEIGHT * cos(roll) * cos(yaw) + H0 * cos(yaw) * sin(roll) + DOG_BODY_LEN * cos(pitch) * sin(yaw) - DOG_BODY_WEIGHT * sin(pitch) * sin(roll) * sin(yaw) - H0 * cos(roll) * sin(pitch) * sin(yaw);
    y = y - DOG_LEG_D2;
    z = DOG_BODY_LEN - DOG_BODY_LEN * cos(pitch) * cos(yaw) - DOG_BODY_WEIGHT * cos(roll) * sin(yaw) + H0 * sin(roll) * sin(yaw) + H0 * cos(roll) * cos(yaw) * sin(pitch) + DOG_BODY_WEIGHT * cos(yaw) * sin(pitch) * sin(roll);
    if (_one_leg_invkinematics_1(x, y, z, thteta1) == -720)
        return -1;
    //腿2的坐标
    x = DOG_BODY_LEN * sin(pitch) + H0 * cos(pitch) * cos(roll) - DOG_BODY_WEIGHT * cos(pitch) * sin(roll);
    y = DOG_BODY_WEIGHT * cos(roll) * cos(yaw) - DOG_BODY_WEIGHT + H0 * cos(yaw) * sin(roll) + DOG_BODY_LEN * cos(pitch) * sin(yaw) + DOG_BODY_WEIGHT * sin(pitch) * sin(roll) * sin(yaw) - H0 * cos(roll) * sin(pitch) * sin(yaw);
    y = y - DOG_LEG_D2;
    z = DOG_BODY_LEN - DOG_BODY_LEN * cos(pitch) * cos(yaw) + DOG_BODY_WEIGHT * cos(roll) * sin(yaw) + H0 * sin(roll) * sin(yaw) + H0 * cos(roll) * cos(yaw) * sin(pitch) - DOG_BODY_WEIGHT * cos(yaw) * sin(pitch) * sin(roll);
    if (_one_leg_invkinematics_1(x, y, z, thteta2) == -720)
        return -1;
    //腿3的坐标
    x = H0 * cos(pitch) * cos(roll) - DOG_BODY_LEN * sin(pitch) + DOG_BODY_WEIGHT * cos(pitch) * sin(roll);
    y = DOG_BODY_WEIGHT - DOG_BODY_WEIGHT * cos(roll) * cos(yaw) + H0 * cos(yaw) * sin(roll) - DOG_BODY_LEN * cos(pitch) * sin(yaw) - DOG_BODY_WEIGHT * sin(pitch) * sin(roll) * sin(yaw) - H0 * cos(roll) * sin(pitch) * sin(yaw);
    y = y - DOG_LEG_D2;
    z = DOG_BODY_LEN * cos(pitch) * cos(yaw) - DOG_BODY_LEN - DOG_BODY_WEIGHT * cos(roll) * sin(yaw) + H0 * sin(roll) * sin(yaw) + H0 * cos(roll) * cos(yaw) * sin(pitch) + DOG_BODY_WEIGHT * cos(yaw) * sin(pitch) * sin(roll);
    if (_one_leg_invkinematics_1(x, y, z, thteta3) == -720)
        return -1;

    x = H0 * cos(pitch) * cos(roll) - DOG_BODY_LEN * sin(pitch) - DOG_BODY_WEIGHT * cos(pitch) * sin(roll);
    y = DOG_BODY_WEIGHT * cos(roll) * cos(yaw) - DOG_BODY_WEIGHT + H0 * cos(yaw) * sin(roll) - DOG_BODY_LEN * cos(pitch) * sin(yaw) + DOG_BODY_WEIGHT * sin(pitch) * sin(roll) * sin(yaw) - H0 * cos(roll) * sin(pitch) * sin(yaw);
    y = y - DOG_LEG_D2;
    z = DOG_BODY_LEN * cos(pitch) * cos(yaw) - DOG_BODY_LEN + DOG_BODY_WEIGHT * cos(roll) * sin(yaw) + H0 * sin(roll) * sin(yaw) + H0 * cos(roll) * cos(yaw) * sin(pitch) - DOG_BODY_WEIGHT * cos(yaw) * sin(pitch) * sin(roll);
    if (_one_leg_invkinematics_1(x, y, z, thteta4) == -720)
        return -1;

    return 0;
}
