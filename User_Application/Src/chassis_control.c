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

/* 自转方向校正（在线可调） */
static float s_omega_dir = CHASSIS_OMEGA_DIR;

/* 单轮测试：0=正常，1~4=只驱动对应轮子，负值反转 */
static float s_test_wheel = 0.0f;
static float s_test_rpm    = 500.0f;

/* 轮子方向（在线可调，填 1 或 -1） */
static float s_rev_fl = CHASSIS_REV_FL;
static float s_rev_fr = CHASSIS_REV_FR;
static float s_rev_rl = CHASSIS_REV_RL;
static float s_rev_rr = CHASSIS_REV_RR;

static Chassis*       s_chassis = NULL;
static DBUS_DRIVES*   s_dbus = NULL;
static PID_Controller s_heading_pid = {0};
static Chassis_Mode   s_mode = CHASSIS_MODE_MANUAL;
static uint8_t        s_offline_ms = 0;
static uint8_t        s_prev_sw1 = 0;
static Chassis_HeadingSource s_heading_src = ChassisControl_NoHeading;

/* 私有函数声明 */
static void  Chassis_Apply_Wheel_Gain(void);
static void  Chassis_Apply_Reverse(void);
static int16_t RC_Channel(uint8_t ch);
static float Chassis_Spin_Omega(void);
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

/* 把在线调的轮子方向写进底盘对象 */
static void Chassis_Apply_Reverse(void) {
    if (s_chassis == NULL) {
        return;
    }
    /* 0=左前 1=右前 2=左后 3=右后 */
    s_chassis->reverse[0] = (s_rev_fl < 0.0f) ? -1 : 1;
    s_chassis->reverse[1] = (s_rev_fr < 0.0f) ? -1 : 1;
    s_chassis->reverse[2] = (s_rev_rl < 0.0f) ? -1 : 1;
    s_chassis->reverse[3] = (s_rev_rr < 0.0f) ? -1 : 1;
}

/* 取 DBUS 指定通道 */
static int16_t RC_Channel(const uint8_t ch) {
    switch (ch) {
        case 0:  return s_dbus->ch0;
        case 1:  return s_dbus->ch1;
        case 2:  return s_dbus->ch2;
        case 3:  return s_dbus->ch3;
        default: return 0;
    }
}

/* 原地旋转量：手动模式右摇杆比例给定，小陀螺模式恒定转速 */
static float Chassis_Spin_Omega(void) {
    if (s_mode == CHASSIS_MODE_SPIN) {
        /* 方向由左手水平决定，不推默认逆时针 */
        return (RC_Channel(CHASSIS_SPIN_DIR_CH) < 0) ? -s_spin_w : s_spin_w;
    }
    return Stick(RC_Channel(CHASSIS_RC_W_CH)) * CHASSIS_MAX_W;
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
    Chassis_Apply_Reverse();
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

    /* 单轮测试模式 */
    if (s_test_wheel != 0.0f) {
        const int idx = (int)s_test_wheel;
        const int wheel = ((idx > 0) ? idx : -idx) - 1;   /* 0=左前 1=右前 2=左后 3=右后 */
        const float dir = (idx > 0) ? 1.0f : -1.0f;

        for (int i = 0; i < 4; i++) {
            s_chassis->wheel[i]->Set_Power_Limit(s_chassis->wheel[i], s_chassis->power_limit);
            s_chassis->wheel[i]->Set_Motor_State(s_chassis->wheel[i],
                                                 (i == wheel) ? dir * s_test_rpm : 0.0f);
        }
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

    /* 左摇杆 -> 平移（右推右移） */
    const float vx =  Stick(RC_Channel(CHASSIS_RC_VX_CH)) * CHASSIS_MAX_VX;
    const float vy = -Stick(RC_Channel(CHASSIS_RC_VY_CH)) * CHASSIS_MAX_VY;

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

    if (s_mode == CHASSIS_MODE_FOLLOW) {
        /* 航向 PID：夹角 theta -> 0 */
        s_heading_pid.kp = s_head_kp;
        s_heading_pid.ki = s_head_ki;
        s_heading_pid.kd = s_head_kd;
        PID_Set_Target(&s_heading_pid, Heading());
        omega = PID_Calculate(&s_heading_pid, 0.0f, 0.0f);
    } else {
        /* 手动 = 右摇杆比例旋转；小陀螺 = 恒定转速 */
        omega = Chassis_Spin_Omega();
    }

    /* 自转方向校正 */
    omega *= s_omega_dir;

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
    /* 单轮测试 */
    {"test_wheel", &s_test_wheel, -4.0f, 4.0f,   NULL},
    {"test_rpm",   &s_test_rpm,    0.0f, 3000.0f, NULL},
    {"spin_w",   &s_spin_w,   0.0f,  20.0f,  NULL},
    {"omega_dir",&s_omega_dir,-1.0f,   1.0f,  NULL},
    {"dead",     &s_dead,     0.0f, 100.0f,  NULL},
    {"head_kp",  &s_head_kp,  0.0f, 100.0f,  NULL},
    {"head_ki",  &s_head_ki,  0.0f,  10.0f,  NULL},
    {"head_kd",  &s_head_kd,  0.0f,  10.0f,  NULL},
    {"wheel_kp", &s_wheel_kp, 0.0f, 100.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_ki", &s_wheel_ki, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
    {"wheel_kd", &s_wheel_kd, 0.0f,  10.0f,  Chassis_Apply_Wheel_Gain},
    /* 轮子方向标定 */
    {"rev_fl",   &s_rev_fl,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_fr",   &s_rev_fr,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_rl",   &s_rev_rl,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
    {"rev_rr",   &s_rev_rr,  -1.0f,   1.0f,  Chassis_Apply_Reverse},
};

const SERIALPLOT_VAR* ChassisControl_Get_Tunable(uint8_t* count) {
    *count = (uint8_t)(sizeof(chassis_tunable) / sizeof(chassis_tunable[0]));
    return chassis_tunable;
}
