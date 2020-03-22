/*
 *pca9685模块，使用它驱动舵机
 *_pca9685_set_angle：设置舵机角度（0-175）
 *_pca9685_set_duty：设置占空比
 *_pca9685_set_off_on：设置高低电平起止时间
 *_pca9685_init：初始化（一定要完整执行）
 */

#include "pca9685.h"

#define PCA_Delay vTaskDelay

/*
 *功能：向pca9685写一字节
 *输入：
    mem_addr：寄存器地址
    data：待写入数据的地址
 *输出：无
 *返回值：读写成功的状态
 */
int _pca9685_write_byte(char mem_addr, unsigned char *data)
{
    return Sensors_I2C_WriteRegister(PCA9685_DEV_ADDR, mem_addr, 1, data);
}

/*
 *功能：向pca9685读一字节
 *输入：
    mem_addr：寄存器地址
 *输出：data：待读入数据的地址
 *返回值：读写成功的状态
 */
int _pca9685_read_byte(char mem_addr, unsigned char *data)
{
    return Sensors_I2C_ReadRegister(PCA9685_DEV_ADDR, mem_addr, 1, data);
}

/*
 *功能：pca9685初始化
 *输入：
    freq：舵机频率
 *输出：
 *返回值：
 *
 */
int _pca9685_init(float freq)
{
    unsigned int prescale;
    unsigned char oldmode, newmode;
    float prescaleval;
    unsigned char prescale_write;
    //进入临界区
    taskENTER_CRITICAL();
    //复位
    oldmode = 0x00;
    if (_pca9685_write_byte(PCA9685_MODE1, &oldmode))
    {
        //退出临界区
        taskEXIT_CRITICAL();
        return -1;
    }
    if (_pca9685_read_byte(PCA9685_MODE1, &oldmode))
    {
        //退出临界区
        taskEXIT_CRITICAL();
        return -2;
    }
    newmode = (oldmode & 0x7F) | 0x10; // sleep
    if (_pca9685_write_byte(PCA9685_MODE1, &newmode))
    {
        //退出临界区
        taskEXIT_CRITICAL();
        return -3;
    }
    prescaleval = 25000000.0 / (4096 * freq * 0.915);
    prescale = floor(prescaleval + 0.5) - 1;
    prescale_write = (unsigned char)prescale;
    if (_pca9685_write_byte(PCA9685_PRE_SCALE, &prescale_write))
    {
        //退出临界区
        taskEXIT_CRITICAL();
        return -4;
    }
    if (_pca9685_write_byte(PCA9685_MODE1, &oldmode))
    {
        //退出临界区
        taskEXIT_CRITICAL();
        return -5;
    }
    //退出临界区
    taskEXIT_CRITICAL();
    //延迟一会
    PCA_Delay(5);
    //进入临界区
    taskENTER_CRITICAL();
    newmode = oldmode | 0xa1;
    if (_pca9685_write_byte(PCA9685_MODE1, &newmode))
    { //退出临界区
        taskEXIT_CRITICAL();
        return -6;
    }
    //退出临界区
    taskEXIT_CRITICAL();
    return 0;
}

/*
 *功能：设置pca9685高低电平开始的时间
 *输入：
    channel：通道
    on：高电平开始时间（0-4096）
    off：低电平开始时间（0-4096）
 *输出：
 *返回值：
 */
int _pca9685_set_off_on(unsigned char channel, unsigned int on, unsigned int off)
{
    unsigned char temp;
    unsigned char channel_temp;
    //通道
    channel_temp = PCA9685_CHANNEL0 + 4 * channel;
    //on低八位
    temp = (unsigned char)((on << 8) >> 8) & 0xff;
    _pca9685_write_byte(channel_temp, &temp);
    //on高八位
    temp = (unsigned char)(on >> 8) & 0xff;
    channel_temp += 1;
    _pca9685_write_byte(channel_temp, &temp);
    //off低八位
    temp = (unsigned char)((off << 8) >> 8) & 0xff;
    channel_temp += 1;
    _pca9685_write_byte(channel_temp, &temp);
    //on高八位
    temp = (unsigned char)(off >> 8) & 0xff;
    channel_temp += 1;
    _pca9685_write_byte(channel_temp, &temp);
    return 0;
}

/*
 *功能：设置pca9685产生占空比
 *输入：
    channel：通道
    duty：占空比（0-4096）
 *输出：
 *返回值：
 *备注：
    520：270度
    100：0度
 */
unsigned int _pca9685_set_duty(unsigned char channel, unsigned int duty)
{
    _pca9685_set_off_on(channel, 0, duty);
    return 0;
}

/*
 *功能：设置pca9685让舵机转的角度
 *输入：
    channel：通道
    angle：角度
 *输出：
 *返回值：
 *备注：
    使用的是270度的舵机
    duty为100时舵机为0度，520时为270度
 */
unsigned int _pca9685_set_angle(unsigned char channel, double angle)
{
    unsigned int duty;
    duty = 100 + (420) / 270.0 * angle;
    _pca9685_set_duty(channel, duty);
    return duty;
}
