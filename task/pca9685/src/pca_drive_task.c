#include "pca_drive_task.h"
#include "theta2dev.h"
//舵机驱动任务句柄
TaskHandle_t ServoDriverTaskHandle;
void xServoDriverTask()
{
    _pca9685_init(50);
    while (1)
    {
        _pca9685_set_angle(0, _theta_0_2driver(35));
        _pca9685_set_angle(1, _theta_1_2driver(35));
        _pca9685_set_angle(2, _theta_2_2driver(35));
        _pca9685_set_angle(3, _theta_3_2driver(35));
        _pca9685_set_angle(4, _theta_4_2driver(-30));
        _pca9685_set_angle(5, _theta_5_2driver(-30));
        _pca9685_set_angle(6, _theta_6_2driver(-30));
        _pca9685_set_angle(7, _theta_7_2driver(-30));
        _pca9685_set_angle(8, _theta_8_2driver(0));
        _pca9685_set_angle(9, _theta_9_2driver(0));
        _pca9685_set_angle(10, _theta_10_2driver(0));
        _pca9685_set_angle(11, _theta_11_2driver(0));
    }
}

int xServoDriverTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)xServoDriverTask,
                          (const char *)"ServoTsak",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)9,
                          (TaskHandle_t *)ServoDriverTaskHandle);

    return xReturn;
}
