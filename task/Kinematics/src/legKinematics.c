#include "legKinematics.h"

/*
 *功能：单条狗腿子末端空间矩阵（初始情况下胯关节坐标系与基坐标系重合）
 *输入：
    theta1（胯），theta2（肘），theta3（腕）三个关节的关节角
    matrix：4x4目标矩阵地址
 *输出：
    matrix：末端位姿矩阵
 *返回值：0
 */
int _one_leg_kinematics(double theta1, double theta2, double theta3, double matrix[4][4])
{
    double a1, a2, a3, d2;
    a1 = DOG_LEG_A1;
    a2 = DOG_LEG_A2;
    a3 = DOG_LEG_A3;
    d2 = DOG_LEG_D2;
    matrix[0][0] = cos(theta2 + theta3) * cos(theta1);
    matrix[0][1] = -sin(theta2 + theta3) * cos(theta1);
    matrix[0][2] = sin(theta1);
    matrix[0][3] = d2 * sin(theta1) + a3 * cos(theta1) * cos(theta2 + theta3) + a2 * cos(theta1) * cos(theta2);
    matrix[1][0] = cos(theta2 + theta3) * sin(theta1);
    matrix[1][1] = -sin(theta2 + theta3) * sin(theta1);
    matrix[1][2] = -cos(theta1);
    matrix[1][3] = a2 * cos(theta2) * sin(theta1) - d2 * cos(theta1) + a3 * sin(theta1) * cos(theta2 + theta3);
    matrix[2][1] = cos(theta2 + theta3);
    matrix[2][2] = 0;
    matrix[2][3] = a3 * sin(theta2 + theta3) + a2 * sin(theta2);
    return 0;
}