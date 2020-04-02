#include "gait.h"

/*
 *功能:椭圆轨迹-对角小跑步态
 *描述：输入alpha（0~2PI），抬腿高度H，步长E，输出驱动空间theta[12]
 *输入：
    alpha：0~2PI，周期为2PI
    H：抬腿高度
    E：步长
    theta：输出驱动空间地址
 *输出：
    theta[12]:驱动空间
 *备注：
    测试值为：tort_cycloid(t, 25, 20, 50, theta, 1);
 */
int tort_cycloid(double t, double T, double H, double E, double theta[12], int dir)
{
    double theta1[3], theta2[3], theta3[3], theta4[3];
    double x, y, z;
    double alpha;
    double max_x;
    //将周期换算到2PI
    alpha = (t / T) * 2 * PI;
    if (alpha > 2 * PI)
        return -1;
    //如果直行x轴最大坐标为98，转弯为90（每条腿的基座标）
    if (dir != 0)
    {
        max_x = 90;
        y = -27;
    }
    else
    {
        max_x = 98;
        y = -27.0;
    }
    //椭圆方程
    z = -E * cos(alpha);
    x = max_x - H * sin(alpha);
    if (_one_leg_invkinematics_1(x, y, z, theta1) == -720)
        return -1;
    //腿四和腿一动作一样
    theta4[0] = theta1[0];
    theta4[1] = theta1[1];
    theta4[2] = theta1[2];
    //腿1,4和腿2,3有半个周期差
    alpha = alpha + PI;
    z = -E * cos(alpha);
    x = max_x - H * sin(alpha);
    if (_one_leg_invkinematics_1(x, y, z, theta2) == -720)
        return -1;
    //腿三和腿一动作一样
    theta3[0] = theta2[0];
    theta3[1] = theta2[1];
    theta3[2] = theta2[2];
    //将关节空间转化到驱动空间
    theta[0] = _theta_0_2driver(180.0 * (theta1[2] / PI));
    theta[1] = _theta_1_2driver(180.0 * (theta2[2] / PI));
    theta[2] = _theta_2_2driver(180.0 * (theta3[2] / PI));
    theta[3] = _theta_3_2driver(180.0 * (theta4[2] / PI));
    theta[4] = _theta_4_2driver(180.0 * (theta1[1] / PI) - 15);
    theta[5] = _theta_5_2driver(180.0 * (theta2[1] / PI) - 15);
    theta[6] = _theta_6_2driver(180.0 * (theta3[1] / PI) - 25);
    theta[7] = _theta_7_2driver(180.0 * (theta4[1] / PI) - 25);
    theta[8] = _theta_8_2driver(180.0 * (theta1[0] / PI));
    theta[9] = _theta_9_2driver(180.0 * (theta2[0] / PI));
    theta[10] = _theta_10_2driver(180.0 * (theta3[0] / PI));
    theta[11] = _theta_11_2driver(180.0 * (theta4[0] / PI));
    if (theta[0] == -720 || theta[1] == -720 || theta[2] == -720 || theta[3] == -720 || theta[4] == -720 || theta[5] == -720 || theta[6] == -720 || theta[7] == -720 || theta[8] == -720 || theta[9] == -720 || theta[10] == -720 || theta[11] == -720)
        return -2;
    return 0;
}

/*
 *功能：顾名思义
 */
int fuck_earth(double t, double T, double H, double E, double theta[12])
{
    double theta1[3], theta2[3], theta3[3], theta4[3];
    double x, z;
    double alpha;

    alpha = (t / T) * 2 * PI;

    if (alpha > 2 * PI)
        return -1;

    z = E * cos(alpha);
    x = 98 - H * sin(alpha);

    if (_one_leg_invkinematics_1(x, -27, z, theta3) == -720)
        return -1;
    theta4[0] = theta3[0];
    theta4[1] = theta3[1];
    theta4[2] = theta3[2];

    //alpha = alpha + PI;
    z = E * cos(alpha);
    x = 98 - H * sin(alpha);
    if (_one_leg_invkinematics_1(x, -27, z, theta1) == -720)
        return -1;
    theta2[0] = theta1[0];
    theta2[1] = theta1[1];
    theta2[2] = theta1[2];

    theta[0] = _theta_0_2driver(180.0 * (theta1[2] / PI));
    theta[1] = _theta_1_2driver(180.0 * (theta2[2] / PI));
    theta[2] = _theta_2_2driver(180.0 * (theta3[2] / PI));
    theta[3] = _theta_3_2driver(180.0 * (theta4[2] / PI));
    theta[4] = _theta_4_2driver(180.0 * (theta1[1] / PI) - 15);
    theta[5] = _theta_5_2driver(180.0 * (theta2[1] / PI) - 15);
    theta[6] = _theta_6_2driver(180.0 * (theta3[1] / PI) - 15);
    theta[7] = _theta_7_2driver(180.0 * (theta4[1] / PI) - 15);
    theta[8] = _theta_8_2driver(180.0 * (theta1[0] / PI));
    theta[9] = _theta_9_2driver(180.0 * (theta2[0] / PI));
    theta[10] = _theta_10_2driver(180.0 * (theta3[0] / PI));
    theta[11] = _theta_11_2driver(180.0 * (theta4[0] / PI));
    if (theta[0] == -720 || theta[1] == -720 || theta[2] == -720 || theta[3] == -720 || theta[4] == -720 || theta[5] == -720 || theta[6] == -720 || theta[7] == -720 || theta[8] == -720 || theta[9] == -720 || theta[10] == -720 || theta[11] == -720)
        return -2;
    return 0;
}

/*
 功能:踏步步态
 输入:
    t:
    T:周期
    H0:抬腿高度
    theta[12]:12个舵机的角度数据存放地址
输出:
    theta[12]:12个舵机的角度
 备注:会往后退,写了玩的,并不可用
 */
int mark_time(double t, double T, double H0, double theta[12])
{
    double theta1[3], theta2[3], theta3[3], theta4[3];
    double x;
    if (t <= (T / 4))
    {
        x = 98.891 - (t / (T / 4)) * H0;
        if (_one_leg_invkinematics_1(x, -27, -38.891, theta1) == -720)
            return -1;
        theta4[0] = theta1[0];
        theta4[1] = theta1[1];
        theta4[2] = theta1[2];
        if (_one_leg_invkinematics_1(79.462, -27, -77.631, theta2) == -720)
            return -1;
        theta3[0] = theta2[0];
        theta3[1] = theta2[1];
        theta3[2] = theta2[2];
    }
    else if (t > (T / 4) && t <= (T / 2))
    {
        x = (98.891 - H0) + H0 * (t - T / 4) / (T / 4);
        if (_one_leg_invkinematics_1(x, -27, -38.891, theta1) == -720)
            return -1;
        theta4[0] = theta1[0];
        theta4[1] = theta1[1];
        theta4[2] = theta1[2];
        if (_one_leg_invkinematics_1(79.462, -27, -77.631, theta2) == -720)
            return -1;
        theta3[0] = theta2[0];
        theta3[1] = theta2[1];
        theta3[2] = theta2[2];
    }
    else if (t > (T / 2) && t <= (3 * T / 4))
    {
        x = 98.891 - ((t - T / 2) / (T / 4)) * H0;
        if (_one_leg_invkinematics_1(x, -27, -38.891, theta2) == -720)
            return -1;
        theta3[0] = theta2[0];
        theta3[1] = theta2[1];
        theta3[2] = theta2[2];
        if (_one_leg_invkinematics_1(79.462, -27, -77.631, theta1) == -720)
            return -1;
        theta4[0] = theta1[0];
        theta4[1] = theta1[1];
        theta4[2] = theta1[2];
    }
    else if (t > (3 * T / 4) && t <= T)
    {
        x = (98.891 - H0) + H0 * ((t - (3 * T / 4)) / (T / 4));
        if (_one_leg_invkinematics_1(x, -27, -38.891, theta2) == -720)
            return -1;
        theta3[0] = theta2[0];
        theta3[1] = theta2[1];
        theta3[2] = theta2[2];
        if (_one_leg_invkinematics_1(79.462, -27, -77.631, theta1) == -720)
            return -1;
        theta4[0] = theta1[0];
        theta4[1] = theta1[1];
        theta4[2] = theta1[2];
    }
    else
    {
        return -3;
    }

    theta[0] = _theta_0_2driver(180.0 * (theta1[2] / PI));
    theta[1] = _theta_1_2driver(180.0 * (theta2[2] / PI));
    theta[2] = _theta_2_2driver(180.0 * (theta3[2] / PI));
    theta[3] = _theta_3_2driver(180.0 * (theta4[2] / PI));
    theta[4] = _theta_4_2driver(180.0 * (theta1[1] / PI));
    theta[5] = _theta_5_2driver(180.0 * (theta2[1] / PI));
    theta[6] = _theta_6_2driver(180.0 * (theta3[1] / PI));
    theta[7] = _theta_7_2driver(180.0 * (theta4[1] / PI));
    theta[8] = _theta_8_2driver(180.0 * (theta1[0] / PI));
    theta[9] = _theta_9_2driver(180.0 * (theta2[0] / PI));
    theta[10] = _theta_10_2driver(180.0 * (theta3[0] / PI));
    theta[11] = _theta_11_2driver(180.0 * (theta4[0] / PI));
    if (theta[0] == -720 || theta[1] == -720 || theta[2] == -720 || theta[3] == -720 || theta[4] == -720 || theta[5] == -720 || theta[6] == -720 || theta[7] == -720 || theta[8] == -720 || theta[9] == -720 || theta[10] == -720 || theta[11] == -720)
        return -2;
    return 0;
}
/*
 *功能：站立
 *输入：
    theta：驱动空间数据的地址
 *输出：
    theta：驱动空间
 *描述：
    每个关节设为固定值
 */
int stand(double theta[12])
{
    theta[0] = _theta_0_2driver(45);
    theta[1] = _theta_1_2driver(45);
    theta[2] = _theta_2_2driver(45);
    theta[3] = _theta_3_2driver(45);
    theta[4] = _theta_4_2driver(-45);
    theta[5] = _theta_5_2driver(-45);
    theta[6] = _theta_6_2driver(-45);
    theta[7] = _theta_7_2driver(-45);
    theta[8] = _theta_8_2driver(0);
    theta[9] = _theta_9_2driver(0);
    theta[10] = _theta_10_2driver(0);
    theta[11] = _theta_11_2driver(0);
    return 0;
}
