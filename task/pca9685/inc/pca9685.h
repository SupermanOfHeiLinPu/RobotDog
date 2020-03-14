#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <math.h>
#include <stdio.h>
#include "bsp_i2c.h"
//freeRTOS头文件
#include "cmsis_os2.h"
#include "cmsis_os.h"

#define PCA9685_DEV_ADDR (0X40)
#define PCA9685_PRE_SCALE (0XFE)
#define PCA9685_CHANNEL0 (0x06)
#define PCA9685_MODE1 (0x00)

extern BaseType_t xpca_testTsakCreat();
extern void pca_test();
#endif
