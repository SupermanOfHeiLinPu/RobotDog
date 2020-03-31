#include "pca_drive_task.h"

double a = 45, flag = 0;

//舵机驱动任务句柄
TaskHandle_t ServoDriverTaskHandle;
void xServoDriverTask()
{
    double theta[12];
    BaseType_t statue;
    _pca9685_init(50);

    while (1)
    {
        statue = xQueueReceive(GaitQueue, theta, portMAX_DELAY);
        if (statue != pdTRUE)
            continue;
        taskENTER_CRITICAL();
        _pca9685_set_angle(0, theta[0]);
        _pca9685_set_angle(1, theta[1]);
        _pca9685_set_angle(2, theta[2]);
        _pca9685_set_angle(3, theta[3]);
        _pca9685_set_angle(4, theta[4]);
        _pca9685_set_angle(5, theta[5]);
        _pca9685_set_angle(6, theta[6]);
        _pca9685_set_angle(7, theta[7]);
        _pca9685_set_angle(8, theta[8]);
        _pca9685_set_angle(9, theta[9]);
        _pca9685_set_angle(10, theta[10]);
        _pca9685_set_angle(11, theta[11]);
        //退出临界区
        taskEXIT_CRITICAL();
    }
}

int xServoDriverTaskCreat()
{
    BaseType_t xReturn = pdPASS;
    xReturn = xTaskCreate((TaskFunction_t)xServoDriverTask,
                          (const char *)"ServoTsak",
                          (uint16_t)256,
                          (void *)NULL,
                          (UBaseType_t)10,
                          (TaskHandle_t *)ServoDriverTaskHandle);

    return xReturn;
}
