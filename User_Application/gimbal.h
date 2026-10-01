#ifndef GIMBAL_H
#define GIMBAL_H

#include "main.h"
#include "../User_Drives/User_Motor/user_dji_motor.h"
#include "../User_Algorithm/User_Controller/user_pid.h"

/* 云台 yaw 配置 -----------------------------------------------------------*/

#define GIMBAL_YAW_MOTOR_ID     (1)          /* GM6020 的 CAN ID (1~7) */
#define GIMBAL_YAW_ZERO_DEG     (0.0f)       /* 云台正对底盘前方时的电机多圈角度 */
#define GIMBAL_YAW_DIR          (1.0f)       /* 角度增大方向：+1 或 -1 现场核对 */

/* GM6020 电流帧量程 ±3000，框架会用 power_limit 再夹一次，
   90W 刚好对应满量程，改小会限制云台出力 */
#define GIMBAL_YAW_POWER_LIMIT  (90.0f)

#define GIMBAL_YAW_KP           (25.0f)
#define GIMBAL_YAW_KI           (0.0f)
#define GIMBAL_YAW_KD           (0.8f)
#define GIMBAL_YAW_MAX_OUT      (3000.0f)    /* GM6020 电流帧量程 */
#define GIMBAL_YAW_MAX_IOUT     (1000.0f)

/* 云台 yaw 是否由遥控器 ch0（右手水平）控制。开了以后底盘就不再用 ch0 平移 */
#define GIMBAL_RC_ENABLE        (1)
#define GIMBAL_RC_RANGE         (180.0f)     /* 满杆对应的角度 deg */
#define GIMBAL_RC_DEAD          (10)

/* 类型定义 ---------------------------------------------------------------*/

typedef struct {
    DJI_MOTOR_DRIVES* yaw_motor;   /* GM6020，Rotor_angle 模式 */
    PID_Controller    yaw_pid;
    float zero_deg;                /* 云台正对底盘前方时的多圈角度 */
    float yaw;                     /* 云台相对底盘角度 deg，±180 */
    float target_yaw;              /* 目标角度 deg，±180 */
    SysTick_Task task;
} GIMBAL_DRIVES;

/* 函数声明 ---------------------------------------------------------------*/
void  Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor);

/* 把当前位置记成云台正前方（零点对齐用） */
void  Gimbal_Set_Zero(GIMBAL_DRIVES* gimbal);

/* 云台相对底盘的角度 deg，逆时针为正，范围 ±180 */
float Gimbal_Get_Yaw(const GIMBAL_DRIVES* gimbal);

#endif // GIMBAL_H
