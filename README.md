# DDS 信号发生器 — MCU 固件

基于 **TI MSPM0G3519** 微控制器与 **FPGA** 协同工作的 DDS（直接数字频率合成）信号发生器的 MCU 控制端固件。

MCU 负责人机交互（键盘输入 + OLED 显示）和 DDS 参数配置，通过 UART 将控制帧发送给 FPGA，由 FPGA 完成实际的波形合成与 DAC 输出。

---

## 功能特性

| 波形模式 | 按键 | 载波频率 | 调制频率 | 调制参数 |
|---------|------|---------|---------|---------|
| **CW（正弦波）** | `1` | 1 Hz – 10 MHz | — | — |
| **AM（调幅波）** | `2` | 1 Hz – 10 MHz | 1 Hz – 1 MHz | 调制深度 10% – 100%（步进 10%） |
| **FM（调频波）** | `3` | 100 kHz – 10 MHz | 1 Hz – 1 MHz | 频偏 1 – 10 kHz |
| **ASK（二进制幅移键控）** | `7` | 固定 100 kHz | 固定 10 kbps | FPGA 内部 PRBS 基带，OOK 输出 |
| **PSK（二进制相移键控）** | `8` | 固定 100 kHz | 固定 10 kbps | FPGA 内部 PRBS 基带，BPSK 输出 |

所有模式支持全局输出幅度调节：按 `取反键` 每次增加 5%，在 10% – 100% 之间循环。

---

## 硬件平台

- **MCU：** TI MSPM0G3519（ARM Cortex-M0+, LQFP-80）
- **主频：** 80 MHz（40 MHz HFXT → SYSPLL 倍频）
- **SDK：** `mspm0_sdk@2.08.00.03`
- **显示屏：** SSD1306 128×64 OLED（SPI 位模拟驱动）
- **输入：** 4×4 矩阵键盘
- **通信：** 双路 UART（UART0 + UART4）同步发送 DDS 控制帧至 FPGA

### 引脚分配

| 外务 | 引脚 | 说明 |
|------|------|------|
| UART0 TX/RX | PA10 / PA11 | DDS 命令通道 1 |
| UART4 TX/RX | PB10 / PB11 | DDS 命令通道 2 |
| OLED SCL | PB3 | SPI 时钟（位模拟） |
| OLED SDA | PB2 | SPI 数据（位模拟） |
| OLED RES | PC9 | 复位 |
| OLED DC | PC8 | 数据/命令选择 |
| OLED CS | PB23 | 片选 |
| 键盘行 | PB6 – PB9 | 输出扫描 |
| 键盘列 | PB20, PB24, PB25, PB27 | 输入读取 |
| LED | PA14 | 状态指示 |

---

## DDS 通信协议

MCU 向 FPGA 发送 **6 字节帧**：

```
[data_hi] [data_lo] [addr] [0xFF] [0xFF] [0xFF]
```

运行时波特率统一为 **921600 bps**。

### 寄存器映射

| 地址 | 名称 | 说明 |
|------|------|------|
| `0x01` | `DDS_ADDR_FREQ_L` | 载波频率，低 16 位 |
| `0x02` | `DDS_ADDR_FREQ_H` | 载波频率，高 16 位 |
| `0x03` | `DDS_ADDR_MOD_TYPE` | 调制类型（0=CW, 1=AM, 2=FM, 3=ASK, 4=PSK） |
| `0x04` | `DDS_ADDR_AM_DEPTH` | AM 调制深度（0 – 32768） |
| `0x05` | `DDS_ADDR_MOD_FREQ_L` | 调制频率，低 16 位 |
| `0x06` | `DDS_ADDR_MOD_FREQ_H` | 调制频率，高 16 位 |
| `0x07` | `DDS_ADDR_FM_DEV` | FM 频偏，单位 Hz |
| `0x08` | `DDS_ADDR_OUTPUT_AMP` | 全局输出幅度（0 – 32768，对应 0% – 100%） |

---

## 工程结构

```
DDS/
├── User/
│   ├── main.c                 # 主程序：键盘 UI、DDS 协议、UART 传输、OLED 显示
│   ├── ti_msp_dl_config.c/h   # SysConfig 自动生成的外设初始化
│   └── config.syscfg           # SysConfig 工程文件
├── Project/
│   ├── key.c / key.h           # 4×4 矩阵键盘驱动
│   ├── oled.c / oled.h         # SSD1306 OLED 驱动（SPI 位模拟）
│   ├── oledfont.h              # 字模数据（ASCII + 中文）
│   ├── DHT11.c / DHT11.h      # DHT11 温湿度驱动（已配置，未启用）
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

如需在 VS Code 中获得代码补全和跳转，运行：

```bash
python gen_compile_db.py
```

生成 `compile_commands.json`，配合 clangd 插件即可使用。

---

## 使用说明

上电后 OLED 显示默认界面，通过键盘操作：

```
数字键 0-9  → 输入频率/参数数值，或选择模式
+ / -       → 切换频率单位 / 调节参数步进
*           → 确认当前输入
/           → 切换调制参数设置项
=           → 切换载波频率 / 调制频率的设置目标
```

当前固件按键分配：

- `1`：CW 正弦波
- `2`：AM
- `3`：FM
- `4/5/6`：保持用于现有 SIN/AM/FM 参数输入
- `7`：ASK，固定 100 kHz 载波、10 kbps PRBS 基带
- `8`：PSK，固定 100 kHz 载波、10 kbps PRBS 基带
- `取反键`：全局输出幅度 `10% -> 15% -> ... -> 100% -> 10%` 循环

### 操作流程示例

**设置 1 MHz 正弦波：**
1. 按 `1` 选择 CW 模式
2. 输入 `1` → 按 `+` 切换单位到 MHz → 按 `*` 确认

**设置 AM 调幅波（载波 500 kHz，调制 1 kHz，深度 50%）：**
1. 按 `2` 选择 AM 模式
2. 输入载波频率 `500`，切换单位 kHz，按 `*` 确认
3. 按 `/` 切换到调制频率，输入 `1`，切换单位 kHz，按 `*` 确认
4. 按 `/` 切换到调制深度，输入 `50`，按 `*` 确认

---

## 开发日志

| 日期 | 里程碑 |
|------|--------|
| 2026-03-27 | 工程初始化，搭建基础框架 |
| 2026-05-03 | 完成 OLED、键盘、UART 基础功能 |
| 2026-05-23 | 首次对接 FPGA，实现 DDS 通信协议 |
| 2026-05-30 | 完成正弦波（CW）、调幅（AM）、调频（FM）三种波形模式 |

---

## 许可证

本项目基于 TI MSPM0 SDK 开发，SDK 部分遵循 TI 的许可协议。用户代码部分请参阅项目中的许可文件。
