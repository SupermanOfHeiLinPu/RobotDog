# RobotDog
## 机器狗的代码  
## 项目背景  
    我的毕设  
## 主控:  
    STM32F767IGT6  
## 陀螺仪:  
    MPU6050  
## 文件结构：  
    Src/：CUBEMX生成的  
    Inc/：CUBEMX生成的  
    task/attitude:机器人姿态  
    task/printf_redefine：重定向printf到uart5
    task/hal_redefine:重定义了hal库的几个函数
    task/DH_Model：定义了一条狗腿子的D-H参数
    task/Kinematics:一条狗腿子的正运动学
    task/Inverse_Kinematics:一条狗腿子的逆运动学
    task/pca9685：舵机驱动，包含了关节空间到驱动空间的映射与舵机驱动任务
    task/gait:步态相关
    task/PS2:PS2手柄控制相关
    matlab:matlab相关仿真，计算等等

## 任务与通讯：
    xGaitTask：步态任务，生成笛卡尔空间坐标然后映射到关节空间接着映射到驱动空间，将驱动空间推送至消息队列
    xServoDriverTask：舵机驱动任务，从消息队列将驱动空间读出并给pca9685发送对应数值
    xAttitudeTsak：姿态解算任务，将解算好的姿态推送至消息队列

    xGaitTask(推送驱动空间)===>GaitQueue(步态队列)===>(从队列读取)xServoDriverTask
    xAttitudeTsak(推送姿态对象)===>AttitudeQueue(姿态队列)===>(从队列读取)xGaitTask推送驱动空间  
## 消息队列:  
    GaitQueue:步态队列，存放驱动空间  
    AttitudeQueue:姿态队列，存放姿态数据  

## 操作系统:  
    FreeRTOS  

## matlab文件说明:  
    matlab版本：2019a  
    ATTITUDE.mat：姿态控制相关计算(最终计算结果为变量AnBn__)  
    k.m：狗腿子在空间中的轨迹动画  
    DOG.mat,matlab.mat：记不得干嘛用的了（大概是早期建了个错误的模型），不关键  
    逆运动学.mat,正运动学.mat:顾名思义  

## 编辑器vscode配置工具:  
    https://github.com/damogranlabs/VS-Code-STM32-IDE  

## 所使用编译器相关信息:  
    arm-none-eabi-gcc.exe (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 9.2.1 20191025 (release) [ARM/arm-9-branch revision 277599] Copyright (C) 2019 Free Software Foundation, Inc.  
    This is free software; see the source for copying conditions.  There is NO warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  

## 备注：
    使用CUBEMX生成代码；大致完成，预计把ps2手柄换成其他控制器(ps2手柄控制有点问题)  
