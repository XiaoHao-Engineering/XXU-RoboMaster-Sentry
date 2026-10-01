#ifndef USER_BSP_H
#define USER_BSP_H

#include "main.h"
#include "bsp_config.h"
#include "../../SEGGER_RTT/SEGGER_RTT.h"

/* 主循环事件表 */
#define MAX_LOOP_EVENT 32
typedef void (*LOOP_Event)(void);
extern LOOP_Event loop_event[MAX_LOOP_EVENT];
extern uint8_t loop_event_num;
void LOOP_EVENT_Handle(void);

/* 滴答任务 */
#include "../../User_Architect/user_systick.h"
extern SysTick_Task LED_Blink_Task;
void LED_Blink_Callback(void *arg);

/* JScope */
#include "../../SEGGER_RTT/user_JScope_Transmit.h"
extern JScope_Transmit_t jscope_transmit;
extern uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP];

/* 串口 */
#include "../../User_Drives/User_Peripheral/user_uart.h"
extern UART_DRIVES user_imu_uart;

/* 状态灯 */
#include "../../User_Drives/User_Basic/user_led.h"
extern LED_DRIVES user_red_led;
extern LED_DRIVES user_green_led;

/* CAN 总线 */
#include "../../User_Drives/User_Peripheral/user_can.h"
extern CAN_DRIVES user_can_1;
extern CAN_DRIVES user_can_2;

/* 定时器 */
#include "../../User_Drives/User_Peripheral/user_timer.h"
extern TIMER_DRIVES user_timer_2;

/* 蜂鸣器 */
#include "../../User_Drives/User_Basic/user_buzzer.h"
extern BUZZER_DRIVES user_buzzer_1;

/* 启动音乐 */
#include "../../User_Application/user_startup_music.h"
extern STARTUP_MUSIC_DRIVES user_startup_music;
extern SysTick_Task user_startup_music_task;

/* 遥控器 */
#include "../../User_Drives/user_dji_bus.h"
extern DBUS_DRIVES user_dbus;

/* 姿态传感器 */
#include "../../User_Drives/User_Sensor/user_hwt906.h"
extern HWT906_DRIVES user_imu;

/* 底盘驱动电机 */
#include "../../User_Drives/User_Motor/user_dji_motor.h"
extern DJI_MOTOR_DRIVES user_wheel_fl;
extern DJI_MOTOR_DRIVES user_wheel_fr;
extern DJI_MOTOR_DRIVES user_wheel_rl;
extern DJI_MOTOR_DRIVES user_wheel_rr;

/* 轮速 PID */
#include "../../User_Algorithm/User_Controller/user_pid.h"
extern PID_Controller user_wheel_pid_fl;
extern PID_Controller user_wheel_pid_fr;
extern PID_Controller user_wheel_pid_rl;
extern PID_Controller user_wheel_pid_rr;

/* 云台 yaw */
#include "../../User_Application/gimbal.h"
extern DJI_MOTOR_DRIVES user_gimbal_yaw_motor;
extern PID_Controller   user_gimbal_yaw_pid;
extern GIMBAL_DRIVES    user_gimbal;

/* 底盘 */
#include "../../User_Application/chassis.h"
#include "../../User_Application/chassis_control.h"
extern Chassis user_chassis;
extern SysTick_Task chassis_task;

/* 串口绘图 / 在线调参 */
#include "../../User_Drives/User_Peripheral/user_serialplot.h"
extern UART_DRIVES       user_serialplot_uart;
extern SERIALPLOT_DRIVES user_serialplot;

/* 板载 IMU 与姿态解算 */
#include "../../User_Drives/User_Peripheral/user_spi.h"
#include "../../User_Drives/User_Sensor/user_mpu6500.h"
#include "../../User_Algorithm/User_Attitude/attitude.h"
extern SPI_DRIVES      user_spi_5;
extern MPU6500_DRIVES  user_mpu6500;
extern ATTITUDE_DRIVES user_attitude;

#endif /* USER_BSP_H */
