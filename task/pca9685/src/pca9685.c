#include "pca9685.h"

#define PCA_Delay vTaskDelay

int _pca9685_write_byte(char mem_addr, unsigned char *data)
{
    return Sensors_I2C_WriteRegister(PCA9685_DEV_ADDR, mem_addr, 1, data);
}

int _pca9685_read_byte(char mem_addr, unsigned char *data)
{
    return Sensors_I2C_ReadRegister(PCA9685_DEV_ADDR, mem_addr, 1, data);
}

int _pca9685_init(float freq)
{
    unsigned int prescale;
    unsigned char oldmode, newmode;
    float prescaleval;
    unsigned char prescale_write;
    oldmode = 0x00;
    _pca9685_write_byte(PCA9685_MODE1, &oldmode);
    prescaleval = 25000000.0 / (4096 * freq * 0.915);
    prescale = floor(prescaleval + 0.5) - 1;
    _pca9685_read_byte(PCA9685_MODE1, &oldmode);
    newmode = (oldmode & 0x7F) | 0x10;            // sleep
    _pca9685_write_byte(PCA9685_MODE1, &newmode); // go to sleep
    prescale_write = (unsigned char)prescale;
    _pca9685_write_byte(PCA9685_PRE_SCALE, &prescale_write); // set the prescaler
    _pca9685_write_byte(PCA9685_MODE1, &oldmode);
    PCA_Delay(5);
    newmode = oldmode | 0xa1;
    _pca9685_write_byte(PCA9685_MODE1, &newmode);
    PCA_Delay(5);
    return 0;
}

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

unsigned int _pca9685_set_duty(unsigned char channel, unsigned int duty)
{
    _pca9685_set_off_on(channel, 0, duty);
    return 0;
}

unsigned int _pca9685_set_angle(unsigned char channel, double angle)
{
    return 0;
}

//测试

TaskHandle_t pca_testTaskHandle;

void pca_test()
{
    unsigned int a;
    unsigned char b;
    unsigned char ch;
    int count = 120;
    ch = 0;
    _pca9685_init(50);
    while (1)
    {
        count++;
        if (count > 400)
        {
            count = 120;
        }

        _pca9685_set_duty(ch, count);
        _pca9685_read_byte(PCA9685_CHANNEL0 + ch * 4 + 2, &b);
        a = b;
        _pca9685_read_byte(PCA9685_CHANNEL0 + ch * 4 + 3, &b);
        a = ((b << 8) & 0xffff) + a;
        printf("%d\n", a);
        vTaskDelay(100);
    }
}

BaseType_t xpca_testTsakCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)pca_test,
                          (const char *)"pca_testTsak",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)9,
                          (TaskHandle_t *)pca_testTaskHandle);

    return xReturn;
}
