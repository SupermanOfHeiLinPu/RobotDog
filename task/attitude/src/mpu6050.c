#include "mpu6050.h"
/**
* mpu6050模块
* 定义了mpu6050变量类型，使用示例：
    mpu6050 name; mpu6050对象变量
    mpu6050 *ptr_name; mpu6050对象指针
* 提供了创建、删除mup6050对象的函数：
    *MPU6050_Self_Creat(char devaddr, _Bool isDMP);返回mpu6050对象指针
    MPU6050_Self_Delete(mpu6050 *self);将对象指针传入
* 使用了FreeRTOS提供的pvPortMalloc和vPortFree。
* mpu6050的属性： 
    devaddr;设备地址，默认为0x68
    isDMPon;是否打开dmp
    sample_rate;采样速率
    dmp_rate;dmp的速率
    gory[3];陀螺仪的值
    accel[3];加速度计的值
    quat[4];dmp读取的四元数（已除q30）
    timestamp;时间戳
    temperature;温度（已除q16）
    *mpuFunList;mpu6050函数虚表
* 提供的方法：
    mpuFunList->Init(mpu6050 *self);mpu6050初始化
    mpuFunList->ReadData(mpu6050 *self);读取mpu6050数据
    mpuFunList->mpu_delay(unsigned int num);mpu6050延迟（使用了vTaskDelay）
    mpuFunList->WriteByte(mpu6050 *self, unsigned char reg, unsigned char *dataptr);写一个字节数据
    mpuFunList->ReadByte(mpu6050 *self, unsigned char reg, unsigned char *dataptr);读一个字节数据
    mpuFunList->get_timestamp(unsigned long *timestamp);获取时间戳（使用了xTaskGetTickCount）
*使用示例：
    mpu6050 *mpu6050self;
    mpu6050self = MPU6050_Self_Creat(0x68, 1);
    mpu6050->mpuFunList->ReadData(mpu6050self);
    printf("temperature:%lf",mpu6050self->temperature);
**/

struct platform_data_s
{
    signed char orientation[9];
};

static struct platform_data_s gyro_pdata = {
    .orientation = {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1}};

static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7; // error
    return b;
}

/** 将由0、+1和-1组成的方向矩阵转换为标量表示形式。
* 输入：mtx：要转换成标量的方向矩阵。
* @return 方向矩阵的描述。 最低的2位(0和1)表示第一行的第1位所在的列，第2位是符号。
* 接下来的2位(3和4)表示第1列在第二行，第5位是符号。
* 接下来的2位(6和7)表示第三行的第1列，第8位是符号。
* 在二进制中，单位矩阵应该是:010_001_000或0x88(十六进制)。
*/
unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 单位矩阵
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

/*
 *功能:mpu6050初始化
 self->isDMPon:1打开dmp；0关闭dmp
 self->sample_rate:采样率
 self->dmp_rate：dmp速度
 *TODO:dmp初始化
 *成功返回0
 */
static int _mpuInit(mpu6050 *self)
{
    unsigned char data;

    unsigned long timestamp;
    struct int_param_s int_param;
    unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
                                  DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
                                  DMP_FEATURE_GYRO_CAL;
    //如果使用dmp
    if (self->isDMPon)
    {
        if (mpu_init(&int_param))
        {
            return -1;
        }
        //唤醒所有传感器
        mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        //使能accel和gyro的fifo
        mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
        //设置采样率
        mpu_set_sample_rate(self->sample_rate);
        //获取时间戳
        get_tick_count(&timestamp);
        //加载dmp固件
        dmp_load_motion_driver_firmware();
        dmp_set_orientation(
            inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
        //使能dmp
        dmp_enable_feature(dmp_features);
        //设置dmp FIFO的速率
        dmp_set_fifo_rate(self->dmp_rate);
        //打开dmp
        mpu_set_dmp_state(1);
        return 0;
    }
    //不使用dmp
    //唤醒
    data = 0;
    if ((self->mpuFunList->WriteByte)(self, MPU6050_RA_PWR_MGMT_1, &data))
        return -1;
    //设置陀螺仪量程为2000
    data = 3 << 3;
    if ((self->mpuFunList->WriteByte)(self, MPU6050_RA_GYRO_CONFIG, &data))
        return -2;
    //设置加速度计的量程为16g
    data = 3 << 3;
    if ((self->mpuFunList->WriteByte)(self, MPU6050_RA_ACCEL_CONFIG, &data))
        return -3;
    //数字低通滤波器1:188hz ;2:98hz; 3:42hz; 4:20hz; 5:10hz ;6:5hz
    data = 1;
    if ((self->mpuFunList->WriteByte)(self, MPU6050_RA_LPF_CONFIG, &data))
        return -4;
    //设置采样率rate data = 1000 / rate - 1;
    data = 1000 / self->sample_rate - 1;
    if ((self->mpuFunList->WriteByte)(self, MPU6050_RA_SMPLRT_DIV, &data))
        return -5;
    (self->mpuFunList->mpu_delay)(10);
    return 0;
}

/*
 *功能：从mpu读取传感器原始数据
 *读取的数据:
 self->gory[3](xyz),
 self->accel[3](xyz),
 self->temperature
 self->timestamp
 *备注:获取时间戳用的freeRTOS的API
 *TODO:异常返回,目前只返回0
 */
static unsigned char _mpuReadData(mpu6050 *self)
{
    int status = 0;
    unsigned char data;
    long temp;
    static short sensors;
    unsigned char more;
    static long tmp_quat[4];
    //如果打开了dmp
    if (self->isDMPon)
    {
        get_tick_count(&(self->timestamp));
        //进入临界区
        taskENTER_CRITICAL();
        //从dmp fifo中读取数据
        status = dmp_read_fifo(self->gory, self->accel, tmp_quat, &(self->timestamp), &sensors, &more);
        //退出临界区
        taskEXIT_CRITICAL();
        //如果读取失败
        if (status)
            return -1;
        self->quat[0] = (double)tmp_quat[0] / MPU6050_Q30;
        self->quat[1] = (double)tmp_quat[1] / MPU6050_Q30;
        self->quat[2] = (double)tmp_quat[2] / MPU6050_Q30;
        self->quat[3] = (double)tmp_quat[3] / MPU6050_Q30;

        //温度
        (self->mpuFunList->ReadByte)(self, MPU6050_RA_TEMP_OUT_H, &data);
        temp = data << 8;
        (self->mpuFunList->ReadByte)(self, MPU6050_RA_TEMP_OUT_L, &data);
        temp |= data;
        self->temperature = (double)temp / MPU6050_Q16;

        return status;
    }
    //读取原始数据
    //陀螺仪x轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_XOUT_H, &data);
    self->gory[0] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_XOUT_L, &data);
    self->gory[0] |= data;
    //陀螺仪y轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_YOUT_H, &data);
    self->gory[1] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_YOUT_L, &data);
    self->gory[1] |= data;
    //陀螺仪z轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_ZOUT_H, &data);
    self->gory[2] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_GYRO_ZOUT_L, &data);
    self->gory[2] |= data;
    //加速度计X轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_XOUT_H, &data);
    self->accel[0] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_XOUT_L, &data);
    self->accel[0] |= data;
    //加速度计Y轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_YOUT_H, &data);
    self->accel[1] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_YOUT_L, &data);
    self->accel[1] |= data;
    //加速度计z轴
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_ZOUT_H, &data);
    self->accel[2] = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_ACCEL_ZOUT_L, &data);
    self->accel[2] |= data;
    //温度
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_TEMP_OUT_H, &data);
    temp = data << 8;
    (self->mpuFunList->ReadByte)(self, MPU6050_RA_TEMP_OUT_L, &data);
    temp |= data;
    self->temperature = (double)temp / MPU6050_Q16;
    //获取时间戳
    (self->mpuFunList->get_timestamp)(&(self->timestamp));
    return (unsigned char)status;
}

/*
 *功能：mpu用到的延迟
 */
static void _mpuDelay(unsigned int num)
{
    vTaskDelay(num);
}

/*
 *功能:读一个字节
 *返回0代表成功
 */
static int _mpuReadByte(mpu6050 *self, unsigned char reg, unsigned char *dataptr)
{
    return (Sensors_I2C_ReadRegister(self->devaddr, reg, 1, dataptr));
}

/*
 *功能:写一个字节
 *返回0代表成功
 */
static int _mpuWriteByte(mpu6050 *self, unsigned char reg, unsigned char *dataptr)
{
    return (Sensors_I2C_WriteRegister(self->devaddr, reg, 1, dataptr));
}

/*
 *功能:获取时间戳
 *返回0代表成功
 */
static int _mpu_get_timestamp(unsigned long *timestamp)
{
    timestamp[0] = xTaskGetTickCount();
    return 0;
}
//mpu6050虚函数
static struct mpu6050Vtbl _mpu1FunList = {
    .Init = _mpuInit,
    .ReadData = _mpuReadData,
    .mpu_delay = _mpuDelay,
    .ReadByte = _mpuReadByte,
    .WriteByte = _mpuWriteByte,
    .get_timestamp = _mpu_get_timestamp};

//接口函数
/*
 *功能:创建一个mpu6050对象
 *输入:
    devaddr:mpu6050设备地址
    isDMP:是否打开dmp(暂时没有)
 *输出:
    创建的mpu6050对象指针
 *
 */
mpu6050 *MPU6050_Self_Creat(char devaddr, _Bool isDMP)
{
    mpu6050 *self;
    self = (mpu6050 *)pvPortMalloc(sizeof(mpu6050));
    self->devaddr = devaddr;
    self->isDMPon = isDMP;
    self->mpuFunList = &_mpu1FunList;
    self->sample_rate = 1000;
    self->dmp_rate = 200;

    return self;
}

/*
 *功能:删除一个mpu6050对象
 *输入：要删除的对象
 */
void MPU6050_Self_Delete(mpu6050 *self)
{
    vPortFree(self);
}
