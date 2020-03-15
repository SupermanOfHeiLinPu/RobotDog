#ifndef __PCA9685_H__
#define __PCA9685_H__

#include <math.h>
#include <stdio.h>
#include "bsp_i2c.h"
#include "stm32f7xx.h"
//freeRTOS头文件
#include "cmsis_os2.h"
#include "cmsis_os.h"

#define PCA9685_DEV_ADDR (unsigned char)(0X40)
#define PCA9685_PRE_SCALE (0XFE)
#define PCA9685_CHANNEL0 (0x06)
#define PCA9685_MODE1 (0x00)

//api
extern unsigned int _pca9685_set_angle(unsigned char channel, double angle);
extern unsigned int _pca9685_set_duty(unsigned char channel, unsigned int duty);
extern int _pca9685_set_off_on(unsigned char channel, unsigned int on, unsigned int off);
extern int _pca9685_init(float freq);

#endif
