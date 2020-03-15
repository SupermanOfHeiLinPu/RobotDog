#include "bsp_i2c.h"

/**********************************************************
以下部分用于MPU6050官方库的移植
***********************************************************/
/*
 *写数据
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr,
                              unsigned char reg_addr,
                              unsigned short len,
                              unsigned char *data_ptr)
{
    int iic_status = HAL_ERROR;
    while (iic_status != HAL_OK)
    {
        iic_status = HAL_I2C_Mem_Write(&hi2c4,
                                       slave_addr << 1,
                                       reg_addr,
                                       I2C_MEMADD_SIZE_8BIT,
                                       data_ptr,
                                       len,
                                       1000);
        //检查通讯状态
        if (iic_status != HAL_OK)
        {
            //出错处理
            HAL_I2C_DeInit(&hi2c4);
            MX_I2C4_Init();
            bsp_i2c_Delay(1);
        }
    }

    return iic_status;
}

/*
 *读数据
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr,
                             unsigned char reg_addr,
                             unsigned short len,
                             unsigned char *data_ptr)
{
    int iic_status = HAL_ERROR;
    while (iic_status != HAL_OK)
    {
        iic_status = HAL_I2C_Mem_Read(&hi2c4,
                                      (slave_addr << 1) + 1,
                                      reg_addr,
                                      I2C_MEMADD_SIZE_8BIT,
                                      data_ptr,
                                      len,
                                      1000);
        //检查通讯状态
        if (iic_status != HAL_OK)
        {
            //出错处理
            HAL_I2C_DeInit(&hi2c4);
            MX_I2C4_Init();
            bsp_i2c_Delay(1);
        }
    }
    return iic_status;
}

/**
  * @brief  获取当前毫秒值
  * @param  存储最新毫秒值的变量
  * @retval 无
  */
int get_tick_count(unsigned long *count)
{
    count[0] = xTaskGetTickCount();
    //count[0] = HAL_GetTick();
    return count[0];
}
