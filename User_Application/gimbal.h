#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"
#include "../User_Algorithm/User_Controller/user_pid.h"

/* 云台 yaw 配置 -----------------------------------------------------------*/
/* 控制方式：★ 锁世界方向（依赖电机 Rotor_speed 模式，见 main.c）
     - 推杆   -> 云台在【世界系】里以恒定角速度转
     - 松杆   -> 锁在当前世界方向，被外力推动会自动回位
     - 底盘自转时用前馈抵消，云台【不跟着底盘转】
       => 云台相对底盘角 theta 才真实反映底盘自转
       => 小陀螺才能走直线
   反馈用板载 IMU 的 yaw（A 板装在云台 yaw 上，测的就是云台世界朝向）*/
#define GIMBAL_LOCK_WORLD       (1)      /* 0=退回纯速度控制（IMU 有问题时用）*/

#define GIMBAL_YAW_MOTOR_ID     (1)          /* GM6020 的 CAN ID (1~7) */
#define GIMBAL_YAW_ZERO_DEG     (0.0f)       /* 云台正对底盘前方时的电机多圈角度 */
#define GIMBAL_YAW_DIR          (1.0f)       /* 角度增大方向：+1 或 -1 现场核对 */

#define GIMBAL_YAW_POWER_LIMIT  (90.0f)      /* 90W 对应 GM6020 电流满量程 */

/* 速度环 PID（Rotor_speed 模式：目标/反馈都是转子转速 RPM，输出是电流刻度）
   Kp=25 -> 1 RPM 误差给 25 输出；满速 57 RPM 误差约 1400（量程 3000）✓
   Ki 用来消除摩擦引起的稳态误差（原角度环的 0 在速度环下不够）
   Kd 保持 0：转速反馈是整数 RPM，差分噪声大，Kd 会放大抖动
   都能在线调：gimbal_kp / gimbal_ki / gimbal_kd */
#define GIMBAL_YAW_KP           (25.0f)
#define GIMBAL_YAW_KI           (0.5f)
#define GIMBAL_YAW_KD           (0.0f)
#define GIMBAL_YAW_MAX_OUT      (3000.0f)    /* GM6020 电流帧量程 */
#define GIMBAL_YAW_MAX_IOUT     (1000.0f)

/* 云台 yaw 遥控：1=开启。DBUS 通道号：0=右手水平 1=右手垂直 2=左手水平 3=左手垂直 */
#define GIMBAL_RC_ENABLE        (1)          /* 现在用右摇杆【垂直】测云台 yaw */
#define GIMBAL_RC_CH            (1)          /* ★ 用哪个通道：1 = 右摇杆上下 */
#define GIMBAL_YAW_MAX_W        (6.0f)       /* 满杆角速度 rad/s */
#define GIMBAL_WORLD_KP         (2.0f)       /* 锁世界位置环增益 RPM/deg */
/* 底盘角速度前馈系数 0~1：前馈是开环的，底盘响应有延迟/打滑时给满会过补偿，
   表现为云台来回追 -> theta 抖 -> 车头摇。抖就往下调，迟钝就往上调 */
#define GIMBAL_FF_GAIN          (0.7f)
#define GIMBAL_RC_DEAD          (10)

/* 掉线保护：连续多少 ms 收不到 GM6020 反馈就判定掉线 ----------------------*/
#define GIMBAL_OFFLINE_MS       (100)

/* 类型定义 ---------------------------------------------------------------*/

typedef struct {
    DJI_MOTOR_DRIVES* yaw_motor;   /* GM6020，Rotor_speed（转速）模式 */
    PID_Controller    yaw_pid;
    float zero_deg;                /* 云台正对底盘前方时的多圈角度 */
    float yaw;                     /* 云台相对底盘角度 deg，±180 */
    float target_yaw;              /* 世界系目标角 deg，±180（锁世界用）*/
    float angle_raw;               /* 电机多圈角度 deg（观察用）*/
    float speed;                   /* 电机转速 RPM（观察用）*/
    float rc_ch;                   /* 遥控通道 0~3，可在线改 */
    float max_w;                   /* 满杆角速度 rad/s，可在线改 */
    float world_kp;                /* 锁世界位置环增益 RPM/deg，可在线改 */
    float ff_gain;                 /* 底盘角速度前馈系数 0~1，可在线改 */
    uint32_t last_dbus_count;      /* 上一周期见到的遥控帧计数 */
    uint16_t dbus_offline_ms;      /* 遥控断线累计 ms */
    float imu_yaw;                 /* IMU 实测的云台世界朝向 deg（观察用）*/
    float target_speed;            /* 目标转子转速 RPM（观察用）*/
    SysTick_Task task;

    uint32_t last_rx_count;        /* 上一周期收到的反馈累计次数 */
    uint16_t offline_ms;           /* 连续未收到反馈的时间 ms */
    uint8_t  is_online;            /* 1=反馈正常 */
    uint8_t  is_aligned;           /* 1=已完成首次对齐（以当前位置为零点） */
    uint8_t  has_ever_online;      /* 1=本次上电曾经收到过反馈 */
} GIMBAL_DRIVES;

/* 函数声明 ---------------------------------------------------------------*/
void  Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor);

/* 把当前位置记成云台正前方（零点对齐用）
   上电时由 Gimbal_Update 自动调用一次；云台掉线重连后也会重新对齐 */
void  Gimbal_Set_Zero(GIMBAL_DRIVES* gimbal);

/* 云台相对底盘的角度 deg，逆时针为正，范围 ±180 */
float Gimbal_Get_Yaw(const GIMBAL_DRIVES* gimbal);

/* GM6020 反馈是否正常，掉线返回 0 */
uint8_t Gimbal_Is_Online(const GIMBAL_DRIVES* gimbal);

/* 是否已完成首次对齐（对齐前不会下发任何指令，避免上电猛转） */
uint8_t Gimbal_Is_Aligned(const GIMBAL_DRIVES* gimbal);

/* 云台角度是否【可信】给底盘当航向源用：
     从没连上过   -> 可信（裸车调试，角度恒为 0，不会乱跑）
     连上过又掉线 -> 不可信（角度是冻结值，会让车跑偏）
     在线但没对齐 -> 不可信
     在线且已对齐 -> 可信 */
uint8_t Gimbal_Is_Reliable(const GIMBAL_DRIVES* gimbal);

#endif // GIMBAL_H
