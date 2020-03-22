# RobotDog
机器狗的代码  
主控：STM32F767IGT6  
陀螺仪：MPU6050  
文件结构：  
    Src/：CUBEMX生成的  
    Inc/：CUBEMX生成的  
    task/attitude:机器人姿态  
    task/printf_redefine：重定向printf到uart5
    task/hal_redefine:重定义了hal库的几个函数
    task/DH_Model：定义了一条狗腿子的D-H参数
    task/Kinematics：一条狗腿子的正运动学
    task/Inverse_Kinematics：一条狗腿子的逆运动学
    task/pca9685：舵机驱动，包含了关节空间到驱动空间的映射与舵机驱动任务

操作系统：FreeRTOS  
备注：使用CUBEMX生成代码；尚未完成  
