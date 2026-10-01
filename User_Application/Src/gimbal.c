#include "../../Core/Inc/bsp.h"
#include "../gimbal.h"

#define YAW_STICK_MAX   (660.0f)

static GIMBAL_DRIVES* s_gimbal = NULL;

/* 私有函数声明 */
static void  Gimbal_Update(void* arg);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Stick(int16_t value);

/* 函数体 */

void Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor) {
    gimbal->yaw_motor = yaw_motor;
    gimbal->zero_deg = GIMBAL_YAW_ZERO_DEG;
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;

    PID_Init(&gimbal->yaw_pid, GIMBAL_YAW_KP, GIMBAL_YAW_KI, GIMBAL_YAW_KD,
             GIMBAL_YAW_MAX_OUT, GIMBAL_YAW_MAX_IOUT);

    s_gimbal = gimbal;

    SysTick_InitTask(&gimbal->task, (void*)gimbal, 100, 1, Task_REPEAT, Gimbal_Update);
    SysTick_StartTask(&gimbal->task);
}

void Gimbal_Set_Zero(GIMBAL_DRIVES* gimbal) {
    gimbal->zero_deg = DJI_Motor_Get_Angle(gimbal->yaw_motor);
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;
}

float Gimbal_Get_Yaw(const GIMBAL_DRIVES* gimbal) {
    const float rel = (DJI_Motor_Get_Angle(gimbal->yaw_motor) - gimbal->zero_deg)
                    * GIMBAL_YAW_DIR;
    return Gimbal_Wrap180(rel);
}

/* 1kHz：取角度 + 遥控给目标 + 角度环下发电流 */
static void Gimbal_Update(void* arg) {
    GIMBAL_DRIVES* gimbal = (GIMBAL_DRIVES*)arg;

    gimbal->yaw = Gimbal_Get_Yaw(gimbal);

#if GIMBAL_RC_ENABLE
    if (user_dbus.is_update) {
        gimbal->target_yaw = Gimbal_Stick(user_dbus.ch0) * GIMBAL_RC_RANGE;
    }
#endif

    /* 目标转成最短路径的多圈角度：A_new = A_cur + (target - yaw) * DIR */
    const float error = Gimbal_Wrap180(gimbal->target_yaw - gimbal->yaw);
    const float cur_angle = DJI_Motor_Get_Angle(gimbal->yaw_motor);

    gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor,
                                       cur_angle + error * GIMBAL_YAW_DIR);
}

static float Gimbal_Wrap180(float deg) {
    while (deg >= 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

static float Gimbal_Stick(const int16_t value) {
    if (value > -GIMBAL_RC_DEAD && value < GIMBAL_RC_DEAD) {
        return 0.0f;
    }
    return (float)value / YAW_STICK_MAX;
}
