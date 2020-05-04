#ifndef __GAIT_H__
#define __GAIT_H__

#include <math.h>
#include "legKinematics.h"
#include "legInverse_Kinematics.h"
#include "theta2dev.h"

#ifndef PI
#define PI 3.14159265358979f
#endif
//extern int tort_cycloid(double t, double T, double H, double E, double theta[12], int dir);
extern int tort_cycloid(double t, double T, double H, double E, double pitch, double roll, double theta[12], int dir);
extern int walk_cycloid(double t, double T, double H, double E, double pitch, double roll, double theta[12], int dir);
extern int stand(double pitch, double roll, double yaw, double theta[12]);
extern int fuck_earth(double t, double T, double H, double E, double theta[12]);
#endif
