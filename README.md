# Smart Clock — 智能门禁系统

基于 **TI MSPM0G3519** 微控制器的智能门禁系统，集成 RFID 刷卡识别与舵机自动开门功能。

通过 4×4 矩阵键盘操控，SSD1306 OLED 实时显示状态，支持手动控制舵机、读取 RFID 卡号、以及刷卡自动开门三种工作模式。

---

## 功能特性

| 模式 | 说明 |
| --- | --- |
| **SERVO（舵机控制）** | 手动调节舵机角度（0°–180°），支持步进设置 |
| **RFID（刷卡识别）** | 通过 UART 读取 RFID 卡号，OLED 显示卡号与原始数据 |
| **AUTO（自动门禁）** | 匹配指定卡号后自动开门：舵机从 0° 转到 180°，保持 3 秒后返回 |

---

## 硬件平台

- **MCU：** TI MSPM0G3519（ARM Cortex-M0+, LQFP-80）
- **主频：** 80 MHz（40 MHz HFXT → SYSPLL 倍频）
- **SDK：** `mspm0_sdk@2.08.00.03`
- **显示屏：** SSD1306 128×64 OLED（4 线 SPI 位模拟驱动）
- **输入：** 4×4 矩阵键盘
- **舵机：** MG946R（PWM 控制，1000–2000 μs 脉宽，20 ms 周期）
- **RFID 读卡器：** UART 通信，9600 波特率，STX/ETX 帧协议

### 引脚分配

| 外设 | 引脚 | 说明 |
| --- | --- | --- |
| 舵机 PWM (CCP0) | PA7 | TIMG8 PWM 输出 |
| 舵机 PWM (CCP1) | PA0 | TIMG8 PWM 输出（备用） |
| UART4 TX/RX | PB10 / PB11 | RFID 读卡器通信（9600 baud） |
| OLED SCL | PB3 | SPI 时钟（位模拟） |
| OLED SDA | PB2 | SPI 数据（位模拟） |
| OLED RES | PB23 | 复位 |
| OLED DC | PC8 | 数据/命令选择 |
| OLED CS | PC9 | 片选 |
| 键盘行 | PB6 – PB9 | 输出扫描（内部上拉） |
| 键盘列 | PB20, PB24, PB25, PB27 | 输入读取（内部下拉） |
| LED4 | PA14 | 状态指示灯 |
| LED8 | PC7 | 状态指示灯 |

---

## 工程结构

```text
Smart_Clock/
├── User/
│   ├── main.c                  # 主程序：模式状态机、舵机控制、RFID 解析、OLED UI
│   ├── ti_msp_dl_config.c/h    # SysConfig 自动生成的外设初始化
│   └── config.syscfg           # SysConfig 工程文件
├── Project/
│   ├── key.c / key.h           # 4×4 矩阵键盘驱动
│   ├── oled.c / oled.h         # SSD1306 OLED 驱动（SPI 位模拟）
│   ├── oledfont.h              # 字模数据（ASCII + 中文）
│   ├── DHT11.c / DHT11.h      # DHT11 温湿度驱动（已实现，未启用）
│   ├── empty.uvprojx           # Keil uVision 5 工程文件
│   ├── mspm0g3519.sct          # ARM 链接脚本
│   └── startup_mspm0g351x_uvision.s  # 启动文件
├── BSP/
│   ├── bsp.h                   # 公共头文件与类型定义
│   └── delay/                  # 延时函数（基于 80 MHz delay_cycles）
├── Source/
│   └── third_party/CMSIS/      # ARM CMSIS Core + CMSIS-DSP 库
├── Output/                     # 编译输出（.hex / .axf / .map）
├── .clangd                     # Clangd 配置（用于 VS Code 代码提示）
├── compile_commands.json       # 编译数据库（clangd 使用）
├── gen_compile_db.py           # 生成 compile_commands.json 的脚本
└── keilkill.bat                # 清理 Keil 编译产物
```

---

## 构建

### 环境要求

- **Keil MDK v5**（需要 ARMClang 编译器）
- **TI MSPM0 SDK 2.08.00.03**
- **TI SysConfig v1.27.0**（如需修改外设配置）

### 编译步骤

1. 安装 [TI MSPM0 SDK](https://www.ti.com/tool/MSPM0-SDK)，默认路径 `C:\ti\mspm0_sdk_2_08_00_03`
2. 用 Keil uVision 5 打开 `Project/empty.uvprojx`
3. 选择目标 `KEIL_M0G3519`，编译（F7）
4. 烧录输出文件 `Output/test.axf` 或 `Output/test.hex`

### VS Code 代码提示

运行以下命令生成编译数据库，配合 clangd 插件即可获得代码补全和跳转：

```bash
python gen_compile_db.py
```

---

## 使用说明

上电后 OLED 显示欢迎界面，3 秒后进入主界面。通过键盘切换模式和操作：

```text
按键 9      → 切换模式（SERVO → RFID → AUTO → SERVO ...）
按键 0-9    → 在 SERVO 模式下输入角度或步进值
按键 + / -  → SERVO 模式下增加/减少当前角度
按键 *      → SERVO 模式下确认输入
```

### SERVO 模式

- OLED 显示当前角度和步进值
- 输入数字后按 `*` 确认设置角度
- 按 `+` / `-` 以步进值增减角度（范围 0°–180°）

### RFID 模式

- OLED 显示读取到的 10 位卡号（ASCII-Hex）
- 支持查看原始数据的分页浏览

### AUTO 模式

- 自动检测刷卡事件
- 匹配预设卡号（`0692CDEE24`）后自动执行开门动作
- 舵机 0° → 180° → 等待 3 秒 → 返回 0°
- 未匹配卡号显示 "NO MATCH"

---

## RFID 通信协议

读卡器通过 UART4（9600 baud）发送数据帧：

```text
[STX 0x02] [12 字节数据] [ETX 0x03]
```

其中 12 字节数据包含 10 字节有效卡号（ASCII-Hex 格式）+ 2 字节校验。

---

## 许可证

本项目基于 TI MSPM0 SDK 开发，SDK 部分遵循 TI 的许可协议。
