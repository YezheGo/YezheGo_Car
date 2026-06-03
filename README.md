# YezheGo_Car
# 江科大两轮平衡车项目
基于 STM32F103C8 的两轮自平衡小车工程。项目使用 MPU6050 获取姿态数据，通过编码器反馈车轮速度，结合角度环、速度环和转向环 PID 控制电机输出，并支持 OLED 实时显示和蓝牙串口调参/遥控。

## 项目功能

- STM32F103C8 主控，Keil uVision 工程
- MPU6050 姿态采集，使用加速度计与陀螺仪互补滤波计算车身倾角
- 双编码器测速，分别读取左右轮转速
- 三个 PID 控制环：
  - 角度环：维持车身平衡
  - 速度环：控制前进/后退速度，并输出角度目标
  - 转向环：控制左右轮差速
- OLED 显示角度、速度、转向 PID 参数与输出
- 蓝牙串口接收遥控和滑块调参命令
- 按键启动/停止控制，倾角过大时自动停止电机

## 目录结构

```text
.
├── DebugConfig/        # Keil 调试配置
├── Hardware/           # 外设驱动：OLED、MPU6050、电机、编码器、串口、按键等
├── Library/            # STM32F10x 标准外设库
├── Start/              # 启动文件、CMSIS 与系统初始化
├── System/             # 延时与定时器模块
├── User/               # 主程序、中断配置和 PID 控制
├── Project.uvprojx     # Keil uVision 工程文件
├── Project.uvoptx      # Keil 工程用户配置
├── keilkill.bat        # Keil 临时/编译文件清理脚本
└── .gitignore          # Git 忽略规则
```

## 硬件与引脚

| 模块 | 引脚/外设 | 说明 |
| --- | --- | --- |
| 电机 PWM | PA0 / TIM2_CH1, PA1 / TIM2_CH2 | 左右电机 PWM，占空比范围 0-100 |
| 电机方向 | PB12, PB13, PB14, PB15 | 两路电机方向控制 |
| 编码器 1 | PA6, PA7 / TIM3 | 左/右其中一路编码器，正交编码输入 |
| 编码器 2 | PB6, PB7 / TIM4 | 另一路编码器，正交编码输入 |
| MPU6050 | PB10(SCL), PB11(SDA) | 软件 I2C，设备地址 0xD0 |
| 蓝牙串口 | PA2(TX), PA3(RX) / USART2 | 9600 baud，用于遥控和调参 |
| 调试串口 | PA9(TX), PA10(RX) / USART1 | 9600 baud |
| OLED | 使用 `Hardware/OLED.*` | 显示 PID 参数、角度和速度状态 |
| 按键 | PB1, PB0, PA5, PA4 | Key1 用于启动/停止 |
| LED | PC13 | 运行状态指示 |

> 具体接线请以硬件实际设计为准；如果电机方向或编码器方向相反，可以在电机接线、方向引脚逻辑或编码器极性处调整。

## 控制流程

主程序初始化 OLED、MPU6050、蓝牙串口、TIM1 定时器、LED、按键、电机、编码器和调试串口。按下 Key1 后，系统初始化 PID 状态并进入运行模式。

TIM1 定时中断作为控制节拍：

- 每 10 次中断读取 MPU6050，计算车身倾角，并更新角度环 PID
- 倾角超过 +/-50 度时自动关闭运行标志，电机输出归零
- 每 50 次中断读取左右编码器，计算平均速度与差速
- 速度环输出作为角度环目标，转向环输出作为左右轮差速 PWM

当前主要 PID 初始参数位于 `User/main.c`：

| PID | Kp | Ki | Kd | 输出限制 |
| --- | ---: | ---: | ---: | --- |
| AnglePID | 3.0 | 0.22 | 2.61 | -100 到 100 |
| SpeedPID | 1.6 | 0.2 | 0 | -20 到 20 |
| TurnPID | 2.0 | 1.0 | 0 | -50 到 50 |

`User/PID.c` 中实现了目标斜坡、实际值一阶滤波、死区、积分限幅和输出限幅，便于降低速度环突变和积分饱和带来的抖动。

## 蓝牙通信协议

蓝牙串口使用 USART2，波特率 9600。接收帧以 `[` 开始、`]` 结束，帧内字段用英文逗号分隔。

### PID 滑块调参

```text
[slider,AngleKp,3.0]
[slider,AngleKi,0.22]
[slider,AngleKd,2.61]
[slider,SpeedKp,1.6]
[slider,SpeedKi,0.2]
[slider,SpeedKd,0]
[slider,TurnKp,2]
[slider,TurnKi,1]
[slider,TurnKd,0]
```

### 摇杆遥控

```text
[joystick,LH,LV,RH,RV]
```

- `LV` 控制速度目标：`SpeedPID.Target = LV / 25.0`
- `RH` 控制转向目标：`TurnPID.Target = RH / 25.0`

程序会通过蓝牙周期性发送绘图数据：

```text
[plot,SpeedTarget,AveSpeed,]
```

## 构建与烧录

1. 安装 Keil MDK-ARM。
2. 安装 STM32F1 设备支持包，工程中使用 `Keil.STM32F1xx_DFP.2.2.0`。
3. 用 Keil 打开 `Project.uvprojx`。
4. 选择 `Target 1`，确认芯片为 `STM32F103C8`。
5. 编译工程并通过 ST-Link/J-Link 等工具烧录到开发板。

编译输出目录 `Objects/`、`Listings/` 以及 `.hex`、`.axf`、`.o` 等产物已在 `.gitignore` 中忽略。

## 调试建议

- 上电前确认电机驱动电源、主控电源和地线连接可靠。
- 第一次调试时建议抬起车轮，避免 PID 参数不合适导致小车突然运动。
- 如果车身倾倒方向与电机补偿方向相反，优先检查电机方向、编码器方向和角度符号。
- 如果速度环引起明显震荡，先降低 `SpeedPID.Kp/Ki`，再逐步增加。
- 如果平衡点存在固定偏差，可以检查 `AngleAcc += 1.2` 这一角度零点补偿。

## 主要文件

- `User/main.c`：系统初始化、主循环、TIM1 控制中断、蓝牙命令处理
- `User/PID.c` / `User/PID.h`：PID 控制器实现
- `Hardware/MPU6050.c`：MPU6050 初始化与姿态数据读取
- `Hardware/Encoder.c`：TIM3/TIM4 编码器模式测速
- `Hardware/Motor.c` / `Hardware/PWM.c`：电机方向和 PWM 输出
- `Hardware/BlueSerial.c`：蓝牙串口协议收发
- `Hardware/OLED.c`：OLED 显示驱动
