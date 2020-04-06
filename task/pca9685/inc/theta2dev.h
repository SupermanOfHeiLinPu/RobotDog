#ifndef __THETA2DEV_H__
#define __THETA2DEV_H__

#include "pca9685.h"

/*
*调用_pca9685_set_angle时每个关节的零位角度
*/
#define ZERO_JOINT_0 (95)
#define ZERO_JOINT_1 (190)
#define ZERO_JOINT_2 (85)
#define ZERO_JOINT_3 (180)
#define ZERO_JOINT_4 (51)
#define ZERO_JOINT_5 (193)
#define ZERO_JOINT_6 (133)
#define ZERO_JOINT_7 (120)
#define ZERO_JOINT_8 (100)
#define ZERO_JOINT_9 (95)
#define ZERO_JOINT_10 (113)
#define ZERO_JOINT_11 (80)

/*
 *舵机的最大与最小角度
 */
#define MAX_SERVO_ANGLE (270)
#define MIN_SERVO_ANGLE (0)

extern double _theta_0_2driver(double theta);
extern double _theta_1_2driver(double theta);
extern double _theta_2_2driver(double theta);
extern double _theta_3_2driver(double theta);
extern double _theta_4_2driver(double theta);
extern double _theta_5_2driver(double theta);
extern double _theta_6_2driver(double theta);
extern double _theta_7_2driver(double theta);
extern double _theta_8_2driver(double theta);
extern double _theta_9_2driver(double theta);
extern double _theta_10_2driver(double theta);
extern double _theta_11_2driver(double theta);

#endif
