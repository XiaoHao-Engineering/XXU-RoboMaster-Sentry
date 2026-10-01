#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 底盘参数集中配置 新乡学院 RoboMaster 哨兵 */

/* 底盘类型 ----------------------------------------------------------------*/
#define CHASSIS_MECANUM   (0)   /* 麦轮 */
#define CHASSIS_OMNI4     (1)   /* 四轮全向轮 */
#define CHASSIS_TYPE      CHASSIS_MECANUM

/* 机械参数 ----------------------------------------------------------------*/
#define CHASSIS_WHEELBASE (0.300f)         /* 轴距 m */
#define CHASSIS_TRACK     (0.300f)         /* 轮距 m */
#define CHASSIS_WHEEL_R   (0.076f)         /* 轮半径 m */
#define CHASSIS_RATIO     (3591.0f / 187.0f)   /* M3508 减速比 */

/* 轮序 0=左前 1=右前 2=左后 3=右后；装反了就把对应项改成 -1 */
#define CHASSIS_REV_FL    (1)
#define CHASSIS_REV_FR    (1)
#define CHASSIS_REV_RL    (1)
#define CHASSIS_REV_RR    (1)

/* 速度限幅 ----------------------------------------------------------------*/
#define CHASSIS_MAX_VX    (3.0f)    /* m/s */
#define CHASSIS_MAX_VY    (3.0f)    /* m/s */
#define CHASSIS_MAX_W     (6.0f)    /* rad/s */

/* 遥控器映射 --------------------------------------------------------------*/
#define CHASSIS_RC_DEAD   (10)      /* 摇杆死区 */

/* 小陀螺 ------------------------------------------------------------------*/
#define CHASSIS_SPIN_W    (6.0f)    /* 自转角速度 rad/s */

/* 跟随 --------------------------------------------------------------------*/
#define CHASSIS_FOLLOW_KP (30.0f)
#define CHASSIS_FOLLOW_KI (0.0f)
#define CHASSIS_FOLLOW_KD (0.0f)   /* D 对 ±180 跳变敏感，默认 0 */
#define CHASSIS_FOLLOW_MAX (6.0f)

/* 轮速 PID ----------------------------------------------------------------*/
#define CHASSIS_SPEED_KP       (15.0f)
#define CHASSIS_SPEED_KI       (0.15f)
#define CHASSIS_SPEED_KD       (0.0f)
#define CHASSIS_SPEED_MAX_OUT  (16384.0f)   /* 电流环满量程 */
#define CHASSIS_SPEED_MAX_IOUT (4000.0f)

/* 功率 --------------------------------------------------------------------*/
#define CHASSIS_WHEEL_POWER_MAX (72.0f) /* 单轮功率上限 W */

/* 控制周期 ----------------------------------------------------------------*/
#define CHASSIS_PERIOD_MS   (1)     /* 底盘控制任务周期 ms */

/* 串口绘图 / 在线调参 ------------------------------------------------------*/
#define CHASSIS_SERIALPLOT_PERIOD_MS (10)   /* 上报周期 ms，10=100Hz */

/* 航向补偿：1=按参考系方向行进（小陀螺走直线） ----------------------------*/
#define CHASSIS_HEADING_COMP  (1)

#endif /* CHASSIS_CONFIG_H */
