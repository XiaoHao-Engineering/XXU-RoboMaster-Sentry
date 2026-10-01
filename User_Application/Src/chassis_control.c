#include "../../Core/Inc/bsp.h"
#include "../chassis_control.h"
#include <math.h>

#define DBUS_STICK_MAX   (660.0f)        /* DBUS 摇杆满量程 */
#define DBUS_OFFLINE_MS  (100)           /* 断线停车阈值 ms */
#define DEG2RAD          (0.0174532925f)

/* ---- 在线可调参数，初值取自 chassis_config.h ---- */
static float s_spin_w   = CHASSIS_SPIN_W;
static float s_dead     = (float)CHASSIS_RC_DEAD;
static float s_head_kp  = CHASSIS_FOLLOW_KP;
static float s_head_ki  = CHASSIS_FOLLOW_KI;
static float s_head_kd  = CHASSIS_FOLLOW_KD;
static float s_wheel_kp = CHASSIS_SPEED_KP;
static float s_wheel_ki = CHASSIS_SPEED_KI;
static float s_wheel_kd = CHASSIS_SPEED_KD;

static Chassis*       s_chassis = NULL;
static DBUS_DRIVES*   s_dbus = NULL;
static PID_Controller s_heading_pid = {0};
static Chassis_Mode   s_mode = CHASSIS_MODE_MANUAL;
static uint8_t        s_offline_ms = 0;
static uint8_t        s_prev_sw1 = 0;
static Chassis_HeadingSource s_heading_src = ChassisControl_NoHeading;

/* 私有函数声明 */
static void  Chassis_Apply_Wheel_Gain(void);
static float Stick(int16_t value);
static float Heading(void);
static void  Heading_Pid_Reset(void);

/* 函数体 */

/* 不接航向源时用这个，功能退化为车体系直控 */
float ChassisControl_NoHeading(void) {
    return 0.0f;
}

static void Chassis_Apply_Wheel_Gain(void) {
    PID_Controller* pids[4] = {&user_wheel_pid_fl, &user_wheel_pid_fr,
                               &user_wheel_pid_rl, &user_wheel_pid_rr};
    for (uint8_t i = 0; i < 4; i++) {
        pids[i]->kp = s_wheel_kp;
        pids[i]->ki = s_wheel_ki;
        pids[i]->kd = s_wheel_kd;
    }
}

static float Stick(const int16_t value) {
    if (value > -(int16_t)s_dead && value < (int16_t)s_dead) {
        return 0.0f;
    }
    return (float)value / DBUS_STICK_MAX;
}

static float Heading(void) {
    return (s_heading_src != NULL) ? s_heading_src() : 0.0f;
}

static void Heading_Pid_Reset(void) {
    PID_Init(&s_heading_pid, s_head_kp, s_head_ki, s_head_kd,
             CHASSIS_FOLLOW_MAX, CHASSIS_FOLLOW_MAX);
}

void ChassisControl_Init(Chassis* chassis, DBUS_DRIVES* dbus) {
    s_chassis = chassis;
    s_dbus = dbus;
    s_mode = CHASSIS_MODE_MANUAL;
    s_offline_ms = 0;
    s_prev_sw1 = 0;
    s_heading_src = ChassisControl_NoHeading;

    Heading_Pid_Reset();
    Chassis_Apply_Wheel_Gain();
}

void ChassisControl_Set_HeadingSource(Chassis_HeadingSource source) {
    s_heading_src = (source != NULL) ? source : ChassisControl_NoHeading;
    Heading_Pid_Reset();
}

void ChassisControl_Update(void) {
    if (s_chassis == NULL || s_dbus == NULL) {
        return;
    }

    /* 遥控器断线检测 */
    if (s_dbus->is_update) {
        s_dbus->is_update = 0;
        s_offline_ms = 0;
    } else if (s_offline_ms < 255) {
        s_offline_ms++;
    }

    if (s_offline_ms > DBUS_OFFLINE_MS) {
        Chassis_Set_Velocity(s_chassis, 0.0f, 0.0f, 0.0f);
        Chassis_Update(s_chassis);
        return;
    }

    /* 拨杆切模式：上=跟随 中=手动 下=小陀螺 */
    if (s_dbus->sw1 != s_prev_sw1) {
        if (s_dbus->sw1 == 1) {
            s_mode = CHASSIS_MODE_FOLLOW;
        } else if (s_dbus->sw1 == 2) {
            s_mode = CHASSIS_MODE_SPIN;
        } else {
            s_mode = CHASSIS_MODE_MANUAL;
        }
        Heading_Pid_Reset();
        s_prev_sw1 = s_dbus->sw1;
    }

    /* 摇杆 -> 参考系目标速度 */
    const float vx =  Stick(s_dbus->ch3) * CHASSIS_MAX_VX;
#if GIMBAL_RC_ENABLE
    const float vy = 0.0f;                                  /* ch0 让给云台 yaw */
#else
    const float vy = -Stick(s_dbus->ch0) * CHASSIS_MAX_VY;
#endif

    float vx_body = vx;
    float vy_body = vy;

#if CHASSIS_HEADING_COMP
    /* 参考系 -> 车体系：绕 z 轴转 theta（逆时针为正） */
    const float theta_rad = Heading() * DEG2RAD;
    const float cos_t = cosf(theta_rad);
    const float sin_t = sinf(theta_rad);
    vx_body = vx * cos_t - vy * sin_t;
    vy_body = vx * sin_t + vy * cos_t;
#endif

    float omega = 0.0f;

    switch (s_mode) {
        case CHASSIS_MODE_SPIN:
            /* 自转方向由 ch2 决定，速度固定 */
            omega = (s_dbus->ch2 < 0) ? -s_spin_w : s_spin_w;
            break;

        case CHASSIS_MODE_FOLLOW: {
            /* 目标：夹角 theta -> 0。误差取 theta 本身，输出即为 +Kp*theta */
            s_heading_pid.kp = s_head_kp;
            s_heading_pid.ki = s_head_ki;
            s_heading_pid.kd = s_head_kd;
            PID_Set_Target(&s_heading_pid, Heading());
            omega = PID_Calculate(&s_heading_pid, 0.0f, 0.0f);
            break;
        }

        case CHASSIS_MODE_MANUAL:
        default:
            omega = Stick(s_dbus->ch2) * CHASSIS_MAX_W;
            break;
    }

    Chassis_Set_Velocity(s_chassis, vx_body, vy_body, omega);
    Chassis_Update(s_chassis);
}

Chassis_Mode ChassisControl_Get_Mode(void) {
    return s_mode;
}

void ChassisControl_Task(void* arg) {
    (void)arg;
    ChassisControl_Update();
}

/* TIM2 10kHz 中断里把底盘速度喂给 JScope */
void ChassisControl_JScopeCallback(void* arg) {
    (void)arg;
    if (s_chassis == NULL) {
        return;
    }
    jscope_transmit.val_1 = s_chassis->vx;
    jscope_transmit.val_2 = s_chassis->vy;
    jscope_transmit.val_3 = s_chassis->omega;
    JScope_Transmit(100);
}

/* ---- 在线调参变量表 ---- */

static const SERIALPLOT_VAR chassis_tunable[] = {
    {"spin_w",   &s_spin_w,   0.0f,  20.0f,  NULL},
    {"dead",     &s_dead,     0.0f, 100.0f,  NULL},
    {"head_kp",  &s_head_kp,  0.0f, 100.0f,  NULL},
    {"head_ki",  &s_head_ki,  0.0f,  10.0f,  NULL},
    {"head_kd",  &s_head_kd,  0.0f,  10.0f,  NULL},
    {"wheel_kp", &s_wheel_kp, 0.0f, 100.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_ki", &s_wheel_ki, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_kd", &s_wheel_kd, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
};

const SERIALPLOT_VAR* ChassisControl_Get_Tunable(uint8_t* count) {
    *count = (uint8_t)(sizeof(chassis_tunable) / sizeof(chassis_tunable[0]));
    return chassis_tunable;
}
