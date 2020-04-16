#ifndef __LEGINVERSE_KINEMATICS__H__
#define __LEGINVERSE_KINEMATICS__H__

#include <math.h>
#include <stdio.h>
#include "dh_model.h"

extern int _one_leg_invkinematics_1(double px, double py, double pz, double theta[3]);
extern int _one_leg_invkinematics_2(double px, double py, double pz, double theta[3]);
extern int _one_leg_invkinematics_3(double px, double py, double pz, double theta[3]);
extern int _one_leg_invkinematics_4(double px, double py, double pz, double theta[3]);

extern int _body_attitude_invkinematics(double thteta1[3], double thteta2[3],
                                        double thteta3[3], double thteta4[3],
                                        double pitch, double roll, double yaw,
                                        double H0);

#endif
