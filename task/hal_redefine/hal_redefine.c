#include "stm32f7xx.h"
#include "cmsis_os.h"
#include "cmsis_os2.h"

uint32_t HAL_GetTick(void)
{
    return xTaskGetTickCount();
}

void HAL_Delay(uint32_t Delay)
{
    vTaskDelay(Delay);
}
