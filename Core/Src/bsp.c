#include "bsp.h"

/* 主循环事件表 */
LOOP_Event loop_event[MAX_LOOP_EVENT] = {0};
uint8_t loop_event_num = 0;

void LOOP_EVENT_Handle(void) {
    for (uint8_t i = 0; i < loop_event_num; i++) {
        loop_event[i]();
    }
}

/* JScope */
JScope_Transmit_t jscope_transmit = {0};
uint8_t JScope_RTT_UpBuffer[BUFFER_SIZE_UP] = {0};

/* 串口 */
UART_DRIVES user_imu_uart = {0};

/* 状态灯 */
LED_DRIVES user_red_led = {0};
LED_DRIVES user_green_led = {0};

/* CAN 总线 */
CAN_DRIVES user_can_1 = {0};
CAN_DRIVES user_can_2 = {0};

/* 定时器 */
TIMER_DRIVES user_timer_2 = {0};

/* 蜂鸣器 */
BUZZER_DRIVES user_buzzer_1 = {0};

/* 启动音乐 */
STARTUP_MUSIC_DRIVES user_startup_music = {0};
SysTick_Task user_startup_music_task = {0};

/* 遥控器 */
DBUS_DRIVES user_dbus = {0};

/* 姿态传感器 */
HWT906_DRIVES user_imu = {0};

/* 底盘驱动电机 */
DJI_MOTOR_DRIVES user_wheel_fl = {0};
DJI_MOTOR_DRIVES user_wheel_fr = {0};
DJI_MOTOR_DRIVES user_wheel_rl = {0};
DJI_MOTOR_DRIVES user_wheel_rr = {0};

/* 轮速 PID */
PID_Controller user_wheel_pid_fl = {0};
PID_Controller user_wheel_pid_fr = {0};
PID_Controller user_wheel_pid_rl = {0};
PID_Controller user_wheel_pid_rr = {0};

/* 底盘 */
Chassis user_chassis = {0};
SysTick_Task chassis_task = {0};

/* 云台 yaw */
DJI_MOTOR_DRIVES user_gimbal_yaw_motor = {0};
PID_Controller   user_gimbal_yaw_pid = {0};
GIMBAL_DRIVES    user_gimbal = {0};

/* LED 闪烁 */
SysTick_Task LED_Blink_Task = {0};

/* 串口绘图 / 在线调参 */
UART_DRIVES       user_serialplot_uart = {0};
SERIALPLOT_DRIVES user_serialplot = {0};

/* 板载 IMU 与姿态解算 */
SPI_DRIVES      user_spi_5 = {0};
MPU6500_DRIVES  user_mpu6500 = {0};
ATTITUDE_DRIVES user_attitude = {0};

void LED_Blink_Callback(void *arg) {
    LED_Toggle((const LED_DRIVES*)arg);
}
