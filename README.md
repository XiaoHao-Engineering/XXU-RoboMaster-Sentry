# 新乡学院 RoboMaster 哨兵底盘框架

大疆 A 板（STM32F427IIH6）麦轮 / 全向轮底盘框架。CMake + Ninja + VSCode。
A 板装在**云台**上，板载 IMU 只解算 **Yaw + Pitch** 两轴；云台 yaw 用 **GM6020** 的绝对编码器。

## 架构

```
上层  User_Application/   chassis.c          底盘对象（逆解 / 正解 / 下发电机）
                          chassis_control.c  遥控映射、模式切换、航向补偿
                          gimbal.c           云台 yaw 角度环 + 云台相对底盘角度
                          chassis_config.h   所有可调参数集中在这里
中层  User_Algorithm/     User_Chassis/      麦轮 / 全向轮运动学
                          User_Attitude/     互补滤波姿态解算（Yaw / Pitch）
                          user_pid / user_cf / user_coord ...
底层  User_Drives/        CAN、DJI 电机（含 GM6020）、DBUS、UART、SPI、MPU6500
      Core/Src/bsp.c      全局实例表 + 主循环事件表
```

## 运行时任务

| 频率 | 在哪 | 干什么 |
|---|---|---|
| **1 kHz** | SysTick → `chassis_task` | 遥控 → 速度 → 航向补偿 → 逆解 → 下发电机 |
| **1 kHz** | SysTick → `gimbal_task` | 读 GM6020 绝对角度 → 云台 yaw 角度环 |
| **1 kHz** | SysTick → `MPU6500_Handle` | 读 IMU → 姿态解算 |
| **500 Hz** | SysTick → `DJI_Motor_Execute` | 组帧发 4 个 M3508 + GM6020 |
| 100 Hz | SysTick → `Serialplot_Output` | 串口绘图上报 |
| 10 kHz | TIM2 中断 | 往 JScope 喂 `vx / vy / omega` |
| 事件 | CAN1_RX0 中断 | 解析电机反馈（含云台编码器）+ 跑 PID |
| 事件 | USART1 中断 | DBUS 空闲中断判帧尾 |
| 事件 | USART2 中断 | 接收串口调参指令 |
| 事件 | `while(1)` | 排空 CAN 发送队列、UART 收发队列 |

## 控制链路

```
遥控器 ──DBUS──▶ USART1 ──▶ user_dbus
                              │
                    ┌─────────┴──────────┐
                    ▼                    ▼
        Gimbal_Update @1kHz      ChassisControl_Update @1kHz
          ├ ch0 → 目标 yaw          ├ 断线 >100ms → 速度清零
          └ GM6020 角度环           ├ 左拨杆 sw1 切模式
              │                     ├ ch3→vx  ch2→omega
        绝对编码器反馈              ├ 航向补偿：参考系 → 车体系
              │                     └ Chassis_Set_Velocity → Chassis_Update
        云台相对底盘角度 ────────────▶ (作为航向源 θ)
                                          │
                              Chassis_Update
                                ├ 逆解：(vx,vy,ω) → 4 轮线速度 m/s
                                └ × 减速比 → 转子 RPM → 下发电机目标
                                          │
                      DJI_Motor_Execute @500Hz 组帧
                                          │
                        CAN1 @1Mbps ──▶ C620 ×4 + GM6020 ──▶ 电机
                                          │
                      反馈 0x201~0x204 / 0x205~0x20B ──▶ 轮速环 / 云台角度环
```

## 云台 yaw（GM6020）

### 为什么用它

GM6020 自带 **13 位绝对编码器**（`rotor_angle` 0~8191 对应 0~360°），
正适合给出**「云台相对底盘」的角度** —— 这是小陀螺走直线和跟随的唯一输入。

| 数据 | 来源 | 用途 |
|---|---|---|
| 云台相对底盘角度 | GM6020 绝对编码器 | 航向补偿 + 跟随 |
| 云台世界姿态 yaw | 板载 MPU6500 | 云台自身控制 / 瞄准 |

### 接线

| 项 | 值 |
|---|---|
| CAN | **CAN1**（和 4 个 M3508 同一条总线） |
| 电机 ID | `GIMBAL_YAW_MOTOR_ID`，默认 **1** |
| 控制帧 | `0x1FE`（ID 1~4）/ `0x2FE`（ID 5~7），电流模式 |
| 反馈帧 | `0x205` ~ `0x20B` |
| 控制模式 | 框架的 `Rotor_angle`（多圈角度，单位度） |

### 三层文件

| 层 | 文件 | 职责 |
|---|---|---|
| 底层 | `User_Drives/User_Motor/user_dji_motor.c/h` | **框架已有**，直接用 `GM6020` 型号 + `Rotor_angle` 模式 |
| 上层 | `User_Application/gimbal.c/h` | 云台相对底盘角度换算 + yaw 角度环 |

### 对外接口

```c
float Gimbal_Get_Yaw(&user_gimbal);   /* 云台相对底盘 deg，逆时针为正，±180 */
Gimbal_Set_Zero(&user_gimbal);        /* 把当前位置记成云台正前方 */
```

### 上电必须标定的 3 个值（都在 `gimbal.h`）

| 宏 | 怎么定 |
|---|---|
| `GIMBAL_YAW_MOTOR_ID` | GM6020 拨码/上位机设的 ID，默认 1 |
| `GIMBAL_YAW_ZERO_DEG` | 手动把云台摆到**正对底盘前方**，读 `DJI_Motor_Get_Angle()` 的值填进来；或运行中调 `Gimbal_Set_Zero()` |
| `GIMBAL_YAW_DIR` | 逆时针转云台，若 `Gimbal_Get_Yaw()` **增大**就是 `+1`，否则 `-1` |

### 遥控器通道

`GIMBAL_RC_ENABLE = 1` 时，**ch0（右手水平）控制云台 yaw 绝对角度**（满杆 ±180°，杆回中云台回正前方）。
此时**底盘不再用 ch0 做左右平移**（`chassis_control.c` 里已处理）。
想要底盘平移就把 `GIMBAL_RC_ENABLE` 改成 0。

## 板载 IMU（MPU6500，装在云台上）

### 接线

| 功能 | 引脚 |
|---|---|
| SPI5 | `PF7` SCK / `PF8` MISO / `PF9` MOSI（5.625 Mbit/s，Mode 3） |
| 片选 CS | `PF6`（普通 GPIO，低有效） |
| MPU6500 中断 | `PB8`（未用，轮询读取） |
| 磁力计 Set/Reset / INT | `PE2` / `PE3`（未用） |
| 串口绘图 | `USART2`（`PD5` / `PD6`，115200） |

### 三层文件

| 层 | 文件 | 职责 |
|---|---|---|
| 底层 | `User_Drives/User_Peripheral/user_spi.c/h` | SPI 收发 + 片选 |
| 底层 | `User_Drives/User_Sensor/user_mpu6500.c/h` | 寄存器、量程换算、**零偏标定** |
| 中层 | `User_Algorithm/User_Attitude/attitude.c/h` | 互补滤波，输出 `pitch` / `yaw` |

### 对外接口

```c
float Attitude_Get_Yaw(&user_attitude);      /* deg，逆时针为正 */
float Attitude_Get_Pitch(&user_attitude);    /* deg，抬头为正 */
Attitude_Reset_Yaw(&user_attitude);          /* 把当前 yaw 置 0 */
MPU6500_Calibrate_Start(&user_mpu6500);      /* 重新标定零偏 */
```

### 零偏标定与漂移测试

1. 上电后**保持静止约 1 秒**（1000 次采样）自动完成陀螺零偏标定
2. **测漂移**：静止 1 分钟，看 `yaw` 漂了多少度 —— 这就是你这块板子的真实零漂
3. `yaw` 是**陀螺积分**，没有绝对参考（磁力计 IST8310 未接），**必然缓慢漂移**

> 大俯仰角下 yaw 用 `gz·cos(pitch) − gx·sin(pitch)` 投影回世界垂直轴。
> 如果大俯仰时 yaw 明显不对，把 `attitude.c` 里那个 `-` 改成 `+`。

### 上电必须核对的 2 件事

| 项 | 怎么核对 | 不对就改 |
|---|---|---|
| **轴向** | 云台放平，`pitch` 应 ≈ 0；抬头时 `pitch` 增大 | `attitude.h` 的 `ATTITUDE_SIGN_X/Y/Z` |
| **偏航正方向** | 手动**逆时针**转云台，`yaw` 应增大 | `ATTITUDE_SIGN_Z` 符号 |

## 串口绘图 / 在线调参

**板子 → 上位机**：`0xAB` + N×`float` + 校验和（1 字节累加和）
默认 4 通道：`yaw / pitch / omega / vx`（在 `main.c` 的 `Serialplot_Set_Data` 一行就能改）

**上位机 → 板子**：`变量名=值#`，例如 `wheel_kp=18.5#`

| 变量名 | 含义 | 范围 |
|---|---|---|
| `spin_w` | 小陀螺自转角速度 | 0 ~ 20 |
| `dead` | 摇杆死区 | 0 ~ 100 |
| `head_kp` / `head_ki` / `head_kd` | 航向 PID | — |
| `wheel_kp` / `wheel_ki` / `wheel_kd` | 4 个轮速 PID（一起改） | — |

上报周期 `CHASSIS_SERIALPLOT_PERIOD_MS`（默认 10ms = 100Hz）。
115200 下 4 通道每帧 18 字节，理论上限约 640Hz；要更快就把 USART2 提到 921600。

变量表在 `chassis_control.c` 的 `chassis_tunable[]`，加一行就能多调一个参数。

## 三种模式与航向补偿

遥控器**左拨杆** `sw1` 切换：

| sw1 | 模式 | 自转 ω | 行进方向 |
|---|---|---|---|
| 上（1） | **跟随** | 把云台夹角控到 0（底盘跟着云台转） | 航向补偿 |
| 中（3） | **手动** | `ch2` 直接给 | 航向补偿 |
| 下（2） | **小陀螺** | 恒定 `spin_w`，方向由 `ch2` 定 | 航向补偿 |

### 航向补偿 = 小陀螺能走直线的原因

遥控器的 vx/vy 是**云台系**的速度，底盘却一直在自转，所以下发前先转到**车体系**：

```
vx_body = vx·cos(θ) − vy·sin(θ)
vy_body = vx·sin(θ) + vy·cos(θ)
```

`θ` = `Gimbal_Get_Yaw()` = **云台相对底盘的角度**。

**⇒ 自转时 θ 每一圈都在变，车体系速度跟着反向旋转，底盘就沿直线走而不是画圈。**

`main.c` 里已经挂好：

```c
static float Chassis_Heading_Get(void) {
    return Gimbal_Get_Yaw(&user_gimbal);
}
/* ... */
ChassisControl_Set_HeadingSource(Chassis_Heading_Get);
```

想换成别的角度源（比如 IMU），只改这个函数即可。

## 底盘运动学与上层 API

麦轮（X 型）和四轮全向轮的**逆解 + 正解**都已实现，改一行切换：

```c
#define CHASSIS_TYPE      CHASSIS_OMNI4   /* chassis_config.h */
```

```c
Chassis_Set_Velocity(&user_chassis, vx, vy, omega);    /* m/s, m/s, rad/s */
Chassis_Update(&user_chassis);                         /* 逆解 + 下发电机 */

float vx, vy, omega;
Chassis_Get_Velocity(&user_chassis, &vx, &vy, &omega); /* 正解，里程计 */
```

坐标系：**x 向前，y 向左，omega 逆时针为正**。

**安全兜底**：遥控器断线 >100ms 自动停车；vx/vy 限 ±3 m/s、ω 限 ±6 rad/s；单轮功率上限 72W；摇杆死区 ±10。全在 `chassis_config.h`。

## 上车前必须标定

| 项 | 位置 | 说明 |
|---|---|---|
| 云台零点 / 方向 | `gimbal.h` 的 `GIMBAL_YAW_ZERO_DEG` / `_DIR` | 见上文 |
| IMU 轴向 | `attitude.h` 的 `ATTITUDE_SIGN_*` | 见上文 |
| 轮子转向 | `chassis_config.h` 的 `CHASSIS_REV_*` | 哪个轮子反了就改成 -1，不要改运动学 |
| 轮径 / 轴距 / 轮距 | `CHASSIS_WHEEL_R` / `WHEELBASE` / `TRACK` | 量实际值 |
| 轮速 PID | `CHASSIS_SPEED_KP / KI / KD` | 起始 15 / 0.15 / 0，可在线调 |
| 航向 PID | `CHASSIS_FOLLOW_*` | 可在线调 |

## 常用操作

| 想干什么 | 怎么做 | 输出看哪个面板 |
|---|---|---|
| **编译 + 烧录** | `Ctrl+Shift+B` | 终端 → `[OK] 烧录完成` |
| **编译 + 烧录 + 调试** | `F5` | 调试控制台 → 停在 `main` |
| 只编译 | `Ctrl+Shift+P` → 任务 → `编译` | 问题面板 |
| 检查 ST-Link | 任务 → `检查 ST-Link 连接` | 终端 → `Voltage: 3.xxV` |
| 清理重编 | 任务 → `清理` | — |

**烧录成功的标志是 `Download verified successfully`**（脚本带 `-v` 校验）。

## CubeMX 重新生成代码之后

CubeMX 会用模板**覆盖 `CMakeLists.txt`**，所以生成完必须跑一次：

```
Ctrl+Shift+P → 任务: 运行任务 → CubeMX 生成后修补
```

它会还原 `CMakeLists.txt`，并检查 6 个 USER CODE 钩子有没有被删掉。
**输出 `全部检查通过` 才算成功**，之后才能编译。

## 依赖说明

- 姿态解算用 `sqrtf` / `atan2f` / `cosf` / `sinf`，`CMakeLists.txt` 里已加 `target_link_libraries(... m)`。
- 加热（PB5）**没有启用**，CubeMX 里也没配。
- 磁力计 IST8310 **未接入**，所以 MPU6500 的 yaw 会漂；云台相对底盘角度靠 GM6020 编码器，不漂。
- 「没接线」的库还有很多（ADRC、LADRC、FIR、牛顿迭代、STP23、ADC、RNG 等），
  都编译好了但没人调用，被 `--gc-sections` 裁掉，所以**不占 Flash**。

---

## 致谢

本框架的**三层架构设计、底层驱动库的组织方式，以及串口绘图调参的思路**，
参考了 **昆明理工大学「昆蓬南冥」战队** 开源的 RoboMaster A 板工程。

在本项目复刻、精简与扩展的过程中，那份开源代码提供了关键的方向与参考。

**感谢昆蓬南冥战队的开源精神。**
