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
    task/gait：步态相关

任务与通讯：
    xGaitTask：步态任务，生成笛卡尔空间坐标然后映射到关节空间接着映射到驱动空间，将驱动空间推送至消息队列
    xServoDriverTask：舵机驱动任务，从消息队列将驱动空间读出并给pca9685发送对应数值
    xAttitudeTsak：姿态解算任务，将解算好的姿态推送至消息队列

    xGaitTask(推送驱动空间)===>GaitQueue(步态队列)===>(从队列读取)xServoDriverTask
    xAttitudeTsak(推送姿态对象)===>AttitudeQueue(姿态队列)
消息队列：
    GaitQueue：步态队列，存放驱动空间
    AttitudeQueue：姿态队列，存放姿态数据

操作系统：FreeRTOS  
备注：使用CUBEMX生成代码；尚未完成  
