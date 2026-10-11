#include "../../Core/Inc/bsp.h"
#include "../gimbal.h"

#define YAW_STICK_MAX   (660.0f)
#define RAD_S_TO_RPM    (9.54929659f)   /* 60 / 2pi */
#define RAD_TO_DEG      (57.29578f)
#define GIMBAL_DT_S     (0.001f)        /* Gimbal_Update 跑在 1kHz */

static GIMBAL_DRIVES* s_gimbal = NULL;

/* 私有函数声明 */
static void  Gimbal_Update(void* arg);
static float Gimbal_Wrap180(float deg);
static float Gimbal_Stick(int16_t value);
static int16_t Gimbal_RC_Channel(uint8_t ch);

/* 函数体 */

void Gimbal_Init(GIMBAL_DRIVES* gimbal, DJI_MOTOR_DRIVES* yaw_motor) {
    gimbal->yaw_motor = yaw_motor;
    gimbal->zero_deg = GIMBAL_YAW_ZERO_DEG;
    gimbal->yaw = 0.0f;
    gimbal->target_yaw = 0.0f;
    gimbal->last_rx_count = 0;
    gimbal->offline_ms = 0;
    gimbal->is_online = 0;
    gimbal->is_aligned = 0;
    gimbal->has_ever_online = 0;
    gimbal->rc_ch = (float)GIMBAL_RC_CH;
    gimbal->max_w = GIMBAL_YAW_MAX_W;
    gimbal->world_kp = GIMBAL_WORLD_KP;
    gimbal->ff_gain = GIMBAL_FF_GAIN;
    gimbal->last_dbus_count = 0;
    gimbal->dbus_offline_ms = 0;
    gimbal->imu_yaw = 0.0f;
    gimbal->target_speed = 0.0f;

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

uint8_t Gimbal_Is_Online(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->is_online : 0;
}

uint8_t Gimbal_Is_Aligned(const GIMBAL_DRIVES* gimbal) {
    return (gimbal != NULL) ? gimbal->is_aligned : 0;
}

uint8_t Gimbal_Is_Reliable(const GIMBAL_DRIVES* gimbal) {
    if (gimbal == NULL) {
        return 0;
    }
    if (gimbal->is_online) {
        return gimbal->is_aligned;                        /* 在线且已对齐 */
    }
    /* 掉线状态：从没连上过算"裸车调试"，连上过又掉才算真掉线 */
    return (gimbal->has_ever_online != 0) ? 0 : 1;
}

/* 1kHz：取角度 + 遥控给目标 + 角度环下发电流 */
static void Gimbal_Update(void* arg) {
    GIMBAL_DRIVES* gimbal = (GIMBAL_DRIVES*)arg;

    /* 掉线检测：反馈计数不涨就累计超时 */
    if (gimbal->yaw_motor->rx_count != gimbal->last_rx_count) {
        gimbal->last_rx_count = gimbal->yaw_motor->rx_count;
        gimbal->offline_ms = 0;
        gimbal->is_online = 1;
        gimbal->has_ever_online = 1;
    } else if (gimbal->offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->offline_ms++;
        if (gimbal->offline_ms >= GIMBAL_OFFLINE_MS) {
            gimbal->is_online = 0;
            gimbal->is_aligned = 0;      /* 掉线就丢掉零点，恢复后重新对齐 */
        }
    }

    /* 观察用镜像 + 遥控目标：故意放在对齐判断【之前】。
       这样云台没接通时，也能从串口看出：
         graw/gspd 有没有值  -> GM6020 反馈通没通
         gtgt 跟不跟杆动     -> 遥控通道对不对 */
    gimbal->angle_raw = DJI_Motor_Get_Angle(gimbal->yaw_motor);
    gimbal->speed = DJI_Motor_Get_Speed(gimbal->yaw_motor);

#if GIMBAL_LOCK_WORLD
    /* A 板在云台 yaw 上，所以 IMU 的 yaw 就是【云台的世界朝向】 */
    gimbal->imu_yaw = Gimbal_Wrap180(Attitude_Get_Yaw(&user_attitude));
#endif

#if GIMBAL_RC_ENABLE
    /* 遥控在线检测：用 rx_count 而不是 is_update。
       ① is_update 会被底盘任务清零，两个模块抢同一个标志会丢数据
       ② DT7 遥控刷新率只有 ~71Hz，而本任务跑 1kHz —— 拿它当积分门控
          会让积分速度只剩 1/14（这是"云台转不动"的根因）*/
    if (user_dbus.rx_count != gimbal->last_dbus_count) {
        gimbal->last_dbus_count = user_dbus.rx_count;
        gimbal->dbus_offline_ms = 0;
    } else if (gimbal->dbus_offline_ms < GIMBAL_OFFLINE_MS) {
        gimbal->dbus_offline_ms++;
    }

    /* 遥控断线就把杆量当 0：锁世界模式会停住不动，速度模式会主动刹车 */
    const float stick = (gimbal->dbus_offline_ms < GIMBAL_OFFLINE_MS)
                      ? Gimbal_Stick(Gimbal_RC_Channel((uint8_t)gimbal->rc_ch))
                      : 0.0f;

#if GIMBAL_LOCK_WORLD
    /* ★ 每个周期都积分（本任务固定 1kHz，DT = 1ms）
       摇杆不动时每周期加 stick*max_w*0.001*57.3 度，1 秒正好累到目标角速度 */
    gimbal->target_yaw += stick * gimbal->max_w * GIMBAL_DT_S
                        * RAD_TO_DEG * GIMBAL_YAW_DIR;
    gimbal->target_yaw = Gimbal_Wrap180(gimbal->target_yaw);
#else
    /* 纯速度控制：推着匀速转、松杆主动刹车停在当前位置 */
    gimbal->target_speed = stick * gimbal->max_w * RAD_S_TO_RPM * GIMBAL_YAW_DIR;
#endif
#else
#if !GIMBAL_LOCK_WORLD
    gimbal->target_speed = 0.0f;
#endif
#endif

    /* 首次对齐：绝对编码器只知道"转子在哪"，上电时 zero_deg 还是旧值，
       直接下发会把云台猛拉向编码器零位。所以等第一帧反馈到了，
       以【当前位置】为零点，对齐完成前不发任何指令 */
    if (!gimbal->is_aligned) {
        if (!gimbal->is_online) {
            return;
        }
        Gimbal_Set_Zero(gimbal);
#if GIMBAL_LOCK_WORLD
        gimbal->target_yaw = gimbal->imu_yaw;   /* 世界目标 = 当前世界朝向 */
#endif
        gimbal->is_aligned = 1;
    }

    gimbal->yaw = Gimbal_Get_Yaw(gimbal);

#if GIMBAL_LOCK_WORLD
    /* ★ 锁世界 = 位置环 + 底盘角速度前馈
       位置环：让云台世界朝向追上 target_yaw
       前馈：  底盘以 omega 自转时，云台必须相对底盘反向转同样角速度，
               才能在世界系里不动 —— 这样云台才不会跟着底盘跑
       两项都要乘 GIMBAL_YAW_DIR（它是"编码器角度增大"的转向系数）*/
    const float error = Gimbal_Wrap180(gimbal->target_yaw - gimbal->imu_yaw);

    gimbal->target_speed = GIMBAL_YAW_DIR *
                           (gimbal->world_kp * error
                            - user_chassis.omega * RAD_S_TO_RPM * gimbal->ff_gain);

    gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor, gimbal->target_speed);
#else
    /* 纯速度控制：直接把转速目标交给电机内置的速度环 */
    gimbal->yaw_motor->Set_Motor_State(gimbal->yaw_motor, gimbal->target_speed);
#endif
}

/* 取 DBUS 指定通道 */
static int16_t Gimbal_RC_Channel(const uint8_t ch) {
    switch (ch) {
        case 0:  return user_dbus.ch0;
        case 1:  return user_dbus.ch1;
        case 2:  return user_dbus.ch2;
        case 3:  return user_dbus.ch3;
        default: return 0;
    }
}

static float Gimbal_Wrap180(float deg) {
    /* 先用整数除法一次折叠到 ±360 以内再微调。
       云台会一直转，total_angle 累积后 deg 可能几万度，
       纯 while 循环要迭代上百次 —— 1kHz 任务里会越来越慢 */
    deg -= 360.0f * (float)((int32_t)(deg / 360.0f));

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
