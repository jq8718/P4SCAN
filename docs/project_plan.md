# P4SCAN 项目信息与开发计划

## 项目目标

P4SCAN 基于 ESP32-P4 和 OV01C1B 黑白摄像头实现本地条码扫码识别设备。

OV01C1B 是黑白（mono）图像传感器，不输出 RGB 彩色图像，也不需要按彩色 Bayer CFA 进行解码。当前目标输出为 10-bit 单色 RAW，经 ISP 或算法链路转换为灰度 / RGB565 预览格式。

目标工作方式：

```text
OV01C1B 摄像头
  -> ESP32-P4 MIPI-CSI / ISP
  -> DSI LCD 本地实时预览
  -> 触发抓拍
  -> ESP32-P4 本地条码解码
  -> USB CDC 返回最终解码字符
  -> USB MSC 虚拟 U 盘保存原图、过程图片和调试数据
```

PC 端主要承担：

- 发送控制命令。
- 接收最终解码字符。
- 从虚拟 U 盘读取抓拍图片、算法过程图和调试文件。

## 当前验证状态与硬件变更

旧 SC2336 方案已验证：

- ESP32-P4 + SC2336 MIPI-CSI 初始化正常。
- SC2336 PID 可识别：`0xcb3a`。
- `RAW10 640x480` 图像正常。
- UVC MJPEG `640x480 @2fps` 可在 Windows Camera 中显示。
- 已修复 Windows Camera 第二次、第三次打开 UVC 无图像的问题。

当前硬件方案已变更：

- 图像 sensor 改为 OV01C1B；当前确认 OV01C1B 使用 7-bit 地址 `0x10`，对应 8-bit write 地址 `0x20`。
- 新增 I2C 扩展 IO 芯片 PCA9538A，I2C 设备地址 `0x70`。
- OV01C1B 的电源、外部 24MHz 时钟、XSHUTDN、瞄准灯、补光灯均由 PCA9538A 控制。
- 后续开发需要优先实现 PCA9538A 初始化、OV01C1B 上电 / 下电时序和 OV01C1B sensor 驱动。

当前固件标记：

```c
raw10-640x480-uvc-abortreset-20260811
```

OV01C1B 的测试寄存器需要区分两种功能：

- ColorBar 使用 `P4:0xF3=0x03` 和 `P4:0x12=0x01`，图像由传感器内部电路生成，与像素阵列无关。
- 规格书说明 ColorBar 的完整输出尺寸为 `1032x1032`，不能直接用普通 `1024x1024` CSI 配置验证；RAW10 单帧应为 `1,331,280` 字节。
- 厂商代码中的 `P1:0xF0=0x04`、`P1:0xF3=0x03`、`P1:0x12=0x01` 属于 FSIN/EVSYNC 帧同步输出路径，不是 ColorBar 配置。

当前 MIPI bring-up 只启用 P4 ColorBar，不写入 P1 FSIN 配置。

RAW10 接收诊断结论（2026-08-15）：

- ESP32-P4 PSRAM 原配置为 `20MHz`，在 OV01C1B `1024x1024@50fps` RAW10 连续输出时会造成 CSI bridge 丢帧；改为 `200MHz` 后，1024x1024 帧可完整接收。
- 1032x1032 RAW10 帧长度不是 64 字节对齐，CSI 驱动和应用的 `esp_cache_msync` 必须对同步地址/长度向外按 64 字节对齐，否则 UVC 侧只能得到黑色占位帧。
- 1024x1024 测试帧的逐行比较显示 `1024/1024` 行均有数据，但不是每行都相同；示例为 `same_previous=1020/1023`，应以行转换日志判断测试图案的实际边界。
- 本次 ColorBar RAW10 复测得到四个水平区域：`y=0..216`、`217..472`、`473..728`、`729..1023`；区域内行完全相同，边界行是 `217/473/729`，且 CSI 状态无 `phy/pkt/boundary/seq/crc` 错误。

当前稳定提交：

```text
c68fdfb Fix UVC reopen streaming for SC2336 RAW10 VGA
```

远程仓库：

```text
git@github.com:jq8718/P4SCAN.git
```

## 硬件平台

| 项目 | 内容 |
| --- | --- |
| 主控芯片 | ESP32-P4, RISC-V |
| 开发板 | ESP32-P4-Function-EV-Board |
| 摄像头 | OV01C1B 黑白图像传感器（mono） |
| 摄像头接口 | MIPI-CSI |
| 摄像头输出 | 10-bit 单色 RAW（RAW10） |
| I2C 扩展 IO | PCA9538A |
| 本地显示 | ESP32-P4 DSI LCD |
| PC 通信接口 | USB HS, 后续规划为 CDC + MSC 复合设备 |
| 调试 / 烧录接口 | USB Serial/JTAG, COM5 |

## 器件资料

| 项目 | 路径 |
| --- | --- |
| OV01C1B 规格书 | `D:\XC\2026\P4SCAN\pdf\OV01C1B_COB_DS_1.0.pdf` |
| PCA9538A 规格书 | `D:\XC\2026\P4SCAN\pdf\PCA9538A.pdf` |
| SC2336 旧方案规格书 | `D:\XC\2026\P4SCAN\pdf\camera_datasheet.pdf` |
| ESP-IDF 参考例程 | `C:\Users\jq163\esp\esp-idf\examples\peripherals\camera\camera_dsi` |

## 板载存储配置

| 组件 | 型号 / 类型 | 规格 | 当前配置 |
| --- | --- | --- | --- |
| Flash | GD25Q128ES1G | 16MB, 128Mbit | `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` |
| Flash 模式 | SPI Flash | DIO, 80MHz | `CONFIG_ESPTOOLPY_FLASHMODE_DIO=y`, `CONFIG_ESPTOOLPY_FLASHFREQ_80M=y` |
| PSRAM | ESP32-P4NRW32 内置 Octal PSRAM | 4MB, 32Mbit | `CONFIG_SPIRAM=y`, `CONFIG_SPIRAM_MODE_HEX=y` |
| PSRAM 频率 | Octal PSRAM | 200MHz | `CONFIG_SPIRAM_SPEED_200M=y` |
| 堆分配策略 | PSRAM 优先 | 小于 16KB 优先内部 RAM | `CONFIG_SPIRAM_USE_MALLOC=y`, `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=16384` |

## 关键硬件连接

| 信号 | GPIO / 管脚 | 用途 |
| --- | --- | --- |
| USB_JTAG D+ | GPIO24 | USB Serial/JTAG 调试 / 烧录 |
| USB_JTAG D- | GPIO25 | USB Serial/JTAG 调试 / 烧录 |
| USB_HS D+ | USB_HS_DP | 规划用于 CDC + MSC 复合 USB 设备 |
| USB_HS D- | USB_HS_DM | 规划用于 CDC + MSC 复合 USB 设备 |
| I2C0 SDA | GPIO7 | OV01C1B SCCB / I2C 控制，PCA9538A 控制 |
| I2C0 SCL | GPIO8 | OV01C1B SCCB / I2C 控制，PCA9538A 控制 |
| OV01C1B I2C 地址 | 7-bit `0x10`, 8-bit write `0x20` | 图像 sensor 寄存器配置 |
| PCA9538A I2C 地址 | `0x70` | 电源、时钟、灯光、XSHUTDN 控制 |
| MIPI-CSI | 开发板摄像头接口 | OV01C1B 图像输入 |
| MIPI-DSI | 开发板 LCD 接口 | 本地 LCD 预览 |

## PCA9538A IO 分配

PCA9538A 用于控制 OV01C1B 相关电源、外部时钟、灯光和复位 / 关断信号。

PCA9538A 上电初始化流程：

```text
1. 向寄存器 0x01 写 0x00，使 P0-P7 输出低电平
2. 向寄存器 0x03 写 0x00，使 P0-P7 配置为输出
```

PCA9538A `0x01` 寄存器为输出寄存器，8 bit 对应 P0-P7：

- bit = 1：对应引脚输出高电平。
- bit = 0：对应引脚输出低电平。

| PCA9538A 引脚 | bit | 控制对象 | 高电平 | 低电平 |
| --- | --- | --- | --- | --- |
| P0 | bit0 | DOVDD | DOVDD 打开 | DOVDD 关闭 |
| P1 | bit1 | AVDD | AVDD 打开 | AVDD 关闭 |
| P2 | bit2 | DVDD | DVDD 打开 | DVDD 关闭 |
| P3 | bit3 | 瞄准灯 | 瞄准灯打开 | 瞄准灯关闭 |
| P4 | bit4 | 补光灯 | 补光灯打开 | 补光灯关闭 |
| P5 | bit5 | XSHUTDN | OV01C1B 打开 | OV01C1B 关闭 |
| P6 | bit6 | 24MHz 时钟 | 24MHz 时钟打开 | 24MHz 时钟关闭 |
| P7 | bit7 | 补光电源 3.3V | 3.3V 打开 | 3.3V 关闭 |

## OV01C1B 上电 / 下电流程

### 上电流程

OV01C1B 上电前需要先初始化 PCA9538A，使所有 P0-P7 输出低电平，然后按以下顺序拉高对应 IO：

```text
1. P7 输出高电平，打开补光电源 3.3V
2. P6 输出高电平，打开 24MHz 时钟
3. P0 输出高电平，打开 DOVDD
4. P1 输出高电平，打开 AVDD
5. P2 输出高电平，打开 DVDD
6. 延时 5ms
7. P5 输出高电平，打开 OV01C1B
8. 延时 5ms
9. 开始 I2C 通信，配置 OV01C1B 寄存器
```

对应 `0x01` 输出寄存器累计值：

| 步骤 | 动作 | `0x01` 建议值 |
| --- | --- | --- |
| 初始化 | 全部输出低 | `0x00` |
| 打开 3.3V | P7=1 | `0x80` |
| 打开 24MHz | P7=1, P6=1 | `0xC0` |
| 打开 DOVDD | P7=1, P6=1, P0=1 | `0xC1` |
| 打开 AVDD | P7=1, P6=1, P1=1, P0=1 | `0xC3` |
| 打开 DVDD | P7=1, P6=1, P2=1, P1=1, P0=1 | `0xC7` |
| 打开 OV01C1B | P7=1, P6=1, P5=1, P2=1, P1=1, P0=1 | `0xE7` |

注意：P3 瞄准灯、P4 补光灯不在基础上电流程中打开，应由扫码流程或调试命令单独控制。

### 下电流程

OV01C1B 下电时按以下顺序关闭：

```text
1. P4 输出低电平，关闭补光灯
2. P3 输出低电平，关闭瞄准灯
3. P6 输出低电平，关闭 24MHz 时钟
4. P5 输出低电平，关闭 OV01C1B
5. P2 输出低电平，关闭 DVDD
6. P1 输出低电平，关闭 AVDD
7. P0 输出低电平，关闭 DOVDD
8. P7 输出低电平，关闭补光电源 3.3V
```

下电流程需要保证最终 `0x01` 输出寄存器回到 `0x00`。

## OV01C1B 寄存器配置文件评估

已收到厂家 / 模组侧寄存器脚本：

```text
D:\XC\2026\P4SCAN\OV01C10_1024X1024_MIPI1LANE_RAW10_50FPS_V1.6.3_20250513_ori.txt
```

该文件对项目有帮助，原因如下：

- 提供了一个可用于 bring-up 的 `1024x1024 RAW10 50fps` 初始化配置。
- 文件中包含时钟和时序注释：
  - `mpll_clk=1434M`
  - 参考 Linux 驱动的 sensor `pclk=89M`；脚本中的 `timer_clk=91M` 是内部时钟参数，不能替代 sensor pixel clock。
  - `phy_clk=1434M`
  - `cnt_clk=546M`
  - `row_clk=45.5M`
  - `hts=364`
  - `frame length / vts=5000`
  - `fps=50`
- 文件中有 196 行形如 `20 xx yy` 的寄存器写入。
- 使用 `0xfd` 做 page select，涉及 page：
  - `0x00`
  - `0x01`
  - `0x02`
  - `0x03`
  - `0x04`
  - `0x05`
  - `0x06`
  - `0x07`
  - `0x0a`
- 文件中包含一次 `sl 5 5` 延时，移植到 ESP-IDF sensor driver 时需要转换为 `vTaskDelay()` 或 sensor register delay 项。
- 参考 Linux 驱动的开流顺序为：page 0 写 `0xa0=0x01`，page 1 写 `0x01=0x31`，回 page 0 后写 `0x20=0x1f`；关闭时写 page 0 的 `0xa0=0x00` 和 page 1 的 `0x01=0x31`。

当前确认 OV01C1B 的 ESP-IDF / Linux 风格 7-bit I2C 地址为 `0x10`，对应厂家寄存器文件中的 8-bit write 地址 `0x20`。寄存器表中 `20 xx yy` 的 `20` 是 8-bit write 地址前缀，后两个字节按 `8-bit register address + 8-bit register value` 导入。

2026-08-13 重新核对 OV01C1B 规格书后，SCCB 地址需要按以下规则调试：

- 规格书第 2.12 节写明 SID2=0、SID1=0 时，write address 为 `0x20`，read address 为 `0x21`；对应 7-bit 地址 `0x10`。
- ESP-IDF `i2c_master_dev_handle_t.device_address` 的注释说明该字段是不带 R/W bit 的 7/10-bit raw address。
- 当前工程把 OV01C1B 驱动地址配置为 7-bit `0x10`。
- 若后续 `0x10` ACK 但读 ID 不符合规格书预期，优先确认 OV01C1B ID 寄存器页/地址、SID1/SID2 实际电平、XSHUTDN 是否真正拉高、24MHz ECLK 是否到达 sensor、AVDD/DOVDD/DVDD 是否满足规格书要求。

### 仍需确认的问题

1. 文件名是 `MIPI1LANE`，但文件头写的是：

   ```text
   @@OV01C10_1024X1024_MIPI2LANE_RAW10_50FPS_V1.6
   ```

   当前已确认以文件名 `MIPI1LANE` 为准，项目按 `1-lane` 使用。后续 ESP32-P4 CSI 配置中的 `data_lane_num` 应设置为 `1`，并需要重点确认 1-lane 下的 lane bit rate 是否稳定。

2. 文件名和文件头使用 `OV01C10`，当前项目硬件描述是 `OV01C1B`。

   当前已确认 `OV01C10` 的寄存器配置可直接用于 `OV01C1B`。

3. 需要确认 `20 xx yy` 的脚本格式含义：

   - `20` 已按厂家寄存器表前缀处理，不作为 ESP-IDF I2C 设备地址。
   - `xx` 已按 8-bit register address 处理。
   - `yy` 已按 8-bit register value 处理。
   - page select `20 fd page` 是否等价于向寄存器 `0xfd` 写 page。

4. 文件中以下行不是标准寄存器写法，需要确认是否为工具命令、FPGA 命令或 sensor 配置参数：

   ```text
   100 99 1024 1024
   100 98 1 0
   64 70200100 104118d
   ```

5. 文件末尾最后一行是：

   ```text
   20 fd 01
   ```

   需要确认该文件是否完整，或者末尾是否缺少后续寄存器写入。

6. OV01C1B 为黑白 mono sensor，输出是单色 RAW10，不是 RGB Bayer 彩色传感器。因此后续应确认单色 RAW10 的有效位排列、黑电平和 ISP 灰度输入配置，不应继续以 BGGR / GRBG / RGGB / GBRG 作为彩色 Bayer 依据。

7. 需要确认 `1024x1024 RAW10 50fps` 对 ESP32-P4 当前硬件链路是否可稳定输入：

   - MIPI lane 数已确认为 `1-lane`。
   - 1-lane 下的 lane bit rate。
   - RAW10 packing 格式。
   - frame buffer 大小。
   - PSRAM / DMA 带宽。

8. 需要确认 24MHz 时钟打开后，到 sensor I2C 可访问前，当前 `5ms + 5ms` 延时是否足够覆盖所有温度和电源上升场景。

该寄存器文件可以作为 OV01C1B driver 第一版模式表。正式写入代码前，仍需确认脚本格式；运行出图后，通过示波器、ESP32-P4 捕获日志和抓帧分析继续确认 1-lane lane bit rate、单色 RAW10 packing、embedded data line 和黑电平。

## 1024x1024 RAW10 50fps 可行性评估

目标模式：

```text
1024 x 1024 RAW10 @ 50fps
```

### 初步结论

ESP32-P4 的 MIPI-CSI 接口从标称能力看，有机会接收该模式；但项目已确认使用 `1-lane`，链路余量比 2-lane 明显更紧。在当前工程配置下，不能直接认为完整系统可以稳定吃下该数据流。

原因是该问题分为三层：

1. CSI 接口是否能收。
2. ISP / DMA / PSRAM 是否能持续搬运和转换。
3. LCD 预览、扫码算法、USB CDC / MSC 调试输出是否能和图像链路同时稳定运行。

其中第 1 层需要重点实测 1-lane lane bit rate 的稳定性，第 2、3 层也需要实测验证，尤其当前工程仍配置为 `CONFIG_SPIRAM_SPEED_20M=y`。

### 数据量估算

1024x1024 RAW10 的有效像素数据量：

```text
1024 * 1024 * 10bit = 10,485,760bit/frame
                   = 1,310,720byte/frame
                   = 1.25MiB/frame

1.25MiB * 50fps = 62.5MiB/s
约等于 65.5MB/s
约等于 524Mbps 有效 RAW payload
```

如果经过 ISP 转成 RGB565：

```text
1024 * 1024 * 2byte = 2MiB/frame
2MiB * 50fps = 100MiB/s
```

这还没有包含：

- MIPI blanking / packet overhead。
- CSI DMA 写入。
- ISP 读写。
- LCD DSI 刷屏读取。
- 条码算法读图。
- USB MSC 保存调试图片。
- 额外 memcpy / cache sync。

因此，裸 RAW 输入数据量尚可接受，但完整应用链路在 50fps 下压力较大。

### MIPI lane 风险

当前寄存器文件存在明显矛盾：

```text
文件名: OV01C10_1024X1024_MIPI1LANE_RAW10_50FPS...
文件头: @@OV01C10_1024X1024_MIPI2LANE_RAW10_50FPS_V1.6
```

当前项目已确认使用 `1-lane`，因此应以 1-lane 作为唯一目标配置评估。

1-lane 下，有效 RAW payload 已约 `524Mbps`。再加上 MIPI 开销和寄存器注释中的 `phy_clk=1434M`，需要确认该 lane rate 是否在 ESP32-P4 当前 CSI PHY 配置、板级走线和 sensor 输出配置可稳定工作的范围内。

正式编码或 bring-up 后仍需确认：

- MIPI data lane bit rate。
- RAW10 packing 方式。
- 是否有 embedded data line。
- 单色 RAW10 的有效位排列和黑电平。

### 当前工程瓶颈

当前工程配置：

```text
CONFIG_SPIRAM_SPEED_20M=y
# CONFIG_SPIRAM_SPEED_200M is not set
```

ESP-IDF 官方 `camera_dsi` 例程默认使用：

```text
CONFIG_SPIRAM_SPEED_200M=y
```

这说明高分辨率 camera + DSI LCD 链路通常需要更高 PSRAM 频率。若继续使用 20MHz PSRAM，1024x1024 RAW10 50fps 不建议作为稳定目标。

另外，当前项目记录的板载 PSRAM 为 4MB。按 1024x1024 计算：

- RAW10 packed 单帧约 `1.25MiB`。
- RGB565 单帧约 `2MiB`。
- RGB565 双缓冲约 `4MiB`。

如果同时需要算法缓存、MSC RAM Disk、JPEG / PGM 输出缓存，4MB PSRAM 会非常紧张。后续需要确认实际可用 PSRAM 容量；此前启动日志曾显示 `32768K` PSRAM，这与项目表格中的 4MB 不一致。

### 建议验证路线

不要一开始就验证完整链路，建议按以下顺序逐层压测：

1. 确认 OV01C1B / OV01C10 配置在 1-lane 下的 lane bit rate、单色 RAW10 packing 和黑电平。
2. 将 PSRAM 配置提升到 200MHz，并确认板级硬件稳定。
3. 只测 MIPI-CSI RAW10 输入，不开 LCD、不跑算法、不写 USB 文件。
4. 统计 frame counter、drop counter、DMA error、CSI error、buffer overrun。
5. 再加入 ISP 输出 RGB565，确认 1024x1024 单帧图像正确。
6. 再加入 LCD 预览，但可以丢帧显示，例如 sensor 50fps、LCD 15-25fps。
7. 扫码算法只在触发时抓取一帧或少量帧，不对 50fps 每帧都做完整解码。
8. USB MSC 调试图片只保存触发帧和过程图，不在预览路径中连续保存。

### 当前 Bring-up 配置

已新增 OV01C1B 第一版 bring-up 配置：

- Sensor I2C 地址：7-bit `0x10`；8-bit write 地址 / 厂家寄存器表前缀为 `0x20`。
- MIPI data lane：`1-lane`。
- 分辨率 / 格式：`1024x1024 RAW10 @50fps`。
- 默认 MIPI lane bit rate：`1434Mbps/lane`，来自厂家寄存器文件注释 `phy_clk=1434M`，后续需要用示波器实测后校正。
- 图像类型：黑白 mono；RAW10 数据应按单色像素处理，不使用彩色 Bayer 解码。
- PCA9538A 地址：`0x70`。
- PCA9538A 上电流程已在 `esp_video_init()` 之前执行。
- 当前 UVC 预览临时配置为 `1024x1024 @2fps MJPEG`，用于快速判断是否有图；这不是最终产品预览架构。

上板后优先观察日志：

```text
p4scan_power: OV01C1B power on through PCA9538A addr=0x70
p4scan_power: PCA9538A output=0x80
p4scan_power: PCA9538A output=0xc0
p4scan_power: PCA9538A output=0xc1
p4scan_power: PCA9538A output=0xc3
p4scan_power: PCA9538A output=0xc7
p4scan_power: PCA9538A output=0xe7
ov01c1b: probing OV01C1B at 7-bit SCCB addr=0x10
ov01c1b: OV01C1B I2C probe ok
ov01c1b: set format: MIPI_1lane_24Minput_RAW10_1024x1024_50fps
```

示波器优先测量：

- PCA9538A P7 / P6 / P0 / P1 / P2 / P5 上电顺序是否符合预期。
- OV01C1B 24MHz 输入时钟是否稳定。
- OV01C1B SID1 / SID2 是否为预期电平；默认地址对应 SID2=0、SID1=0。
- XSHUTDN 拉高后到第一次 SCCB 访问需至少满足规格书 T5 要求，当前代码按 10ms 等待。
- MIPI clock lane 是否进入 HS。
- MIPI data lane 0 是否有 HS burst。
- 帧周期是否接近 `20ms`，即 `50fps`。
- 实测 lane bit rate 是否接近当前默认 `1434Mbps/lane`。

### 产品策略建议

推荐把 `1024x1024 RAW10 50fps` 定义为 sensor 采集能力和抓拍能力，而不是完整系统所有模块都以 50fps 处理。

更稳妥的实时策略：

- Sensor 可以运行 50fps。
- LCD 预览按 15-25fps 抽帧显示。
- 扫码触发时冻结或复制一帧进入算法。
- 算法在独立任务中处理 ROI / 灰度图。
- USB 只上传最终结果和触发帧调试数据。

这样更符合条码扫码设备的实际需求，也能避免为了全链路 50fps 消耗过多内存带宽。

## 软件与工具链

| 项目 | 内容 |
| --- | --- |
| ESP-IDF 版本 | v5.4.2 |
| ESP-IDF 路径 | `C:\Users\jq163\esp\esp-idf` |
| ESP-IDF Python 环境 | `C:\Users\jq163\.espressif\python_env\idf5.4_py3.11_env` |
| ESP-IDF 工具路径 | `C:\Users\jq163\esp\esp-idf\tools` |
| CMake | `C:\Users\jq163\.espressif\tools\cmake\3.30.2\bin` |
| Ninja | `C:\Users\jq163\.espressif\tools\ninja\1.12.1` |
| ccache | `C:\Users\jq163\.espressif\tools\ccache\4.10\ccache-4.10-windows-x86_64` |
| RISC-V 工具链 | `C:\Users\jq163\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin` |
| 烧录 / 调试串口 | COM5 |

## 常用构建命令

PowerShell 环境：

```powershell
$env:IDF_PATH='C:\Users\jq163\esp\esp-idf'
$env:IDF_PYTHON_ENV_PATH='C:\Users\jq163\.espressif\python_env\idf5.4_py3.11_env'
$env:PATH='C:\Users\jq163\.espressif\tools\cmake\3.30.2\bin;C:\Users\jq163\.espressif\tools\ninja\1.12.1;C:\Users\jq163\.espressif\tools\ccache\4.10\ccache-4.10-windows-x86_64;C:\Users\jq163\.espressif\tools\riscv32-esp-elf\esp-14.2.0_20241119\riscv32-esp-elf\bin;' + $env:PATH
```

编译：

```powershell
& 'C:\Users\jq163\.espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe' 'C:\Users\jq163\esp\esp-idf\tools\idf.py' build
```

烧录：

```powershell
& 'C:\Users\jq163\.espressif\python_env\idf5.4_py3.11_env\Scripts\python.exe' 'C:\Users\jq163\esp\esp-idf\tools\idf.py' -p COM5 flash
```

注意：烧录前需要释放 COM5，关闭串口监视器或其他占用 COM5 的程序。

## 目标 USB 架构

后续不再以 UVC 作为主要 PC 图像预览方式，改为：

```text
USB HS Composite Device
  Interface 0/1: CDC ACM
    - PC -> ESP32-P4: 命令控制
    - ESP32-P4 -> PC: 解码结果、状态、进度、错误信息

  Interface 2: MSC Mass Storage
    - 保存抓拍原图
    - 保存灰度图、ROI 图、二值化图、边缘图等过程图片
    - 保存 result.txt
    - 保存 debug.json
```

CDC 用于实时控制和最终结果上传，MSC 用于调试文件交换。

## 双核任务划分规划

ESP32-P4 是双核 RISC-V 芯片，后续应将实时预览、USB 通信和扫码算法解耦到不同任务中运行。

推荐核心分工：

```text
Core 0:
  Camera / LCD preview / USB CDC + MSC / 系统任务

Core 1:
  Barcode decode task
  ROI / 灰度化 / 二值化 / 条码解码算法
```

如果阶段性保留 UVC 预览，也可以采用：

```text
Core 0:
  TinyUSB / UVC 传输 / JPEG 编码调度

Core 1:
  扫码识别算法
```

ESP-IDF 中可通过 `xTaskCreatePinnedToCore()` 固定扫码任务到 Core 1：

```c
xTaskCreatePinnedToCore(
    barcode_decode_task,
    "barcode_decode",
    8192,
    ctx,
    6,
    &decode_task_handle,
    1
);
```

当前 UVC 组件已有任务核心配置：

```text
CONFIG_UVC_TINYUSB_TASK_CORE=-1
CONFIG_UVC_CAM1_TASK_CORE=-1
```

如果继续使用 UVC 预览，可改为：

```text
CONFIG_UVC_TINYUSB_TASK_CORE=0
CONFIG_UVC_CAM1_TASK_CORE=0
```

扫码任务固定到 Core 1。

注意：分核心可以减少任务互相抢占，提高调度并行度，但解码速度不一定线性提升。实际瓶颈还可能来自：

- PSRAM 带宽。
- 图像拷贝次数。
- ISP / JPEG / USB DMA 资源竞争。
- 算法处理全幅图像导致计算量过大。

真正提高扫码速度的重点：

- 只处理 ROI，不处理整幅图。
- 解码直接使用 RAW / 灰度数据，不从 JPEG 反解。
- LCD 预览走 ISP RGB565，解码算法走 RAW / 灰度路径。
- 使用双缓冲或三缓冲，避免预览和解码互相等待。
- `DEBUG OFF` 时不保存大量过程图片，只返回最终字符。

推荐数据流：

```text
Camera DMA
  -> preview_frame_queue -> LCD task, Core 0
  -> scan_frame_queue    -> Decode task, Core 1

Decode task:
  ROI -> gray -> binary -> barcode decode
  -> CDC 返回字符
  -> DEBUG ON 时写 MSC 文件
```

## CDC 命令协议规划

第一版命令：

```text
SCAN
SNAP
DEBUG ON
DEBUG OFF
STATUS
CLEAR
SET ROI x y w h
SET SYM CODE128
LIGHT AIM ON
LIGHT AIM OFF
LIGHT ILLUM ON
LIGHT ILLUM OFF
SENSOR POWER ON
SENSOR POWER OFF
IOEXP GET
IOEXP SET xx
```

第一版返回：

```text
OK
BUSY
ERROR reason
RESULT OK CODE128 1234567890
RESULT FAIL
IOEXP 0xE7
```

## MSC 文件结构规划

每次触发解码生成一个独立目录：

```text
/SCAN0001/
  original.jpg
  gray.pgm
  roi.pgm
  binary.pgm
  edge.pgm
  result.txt
  debug.json
  ready.txt
```

写入规则：

- ESP32-P4 先写所有图片和调试数据。
- 最后创建 `ready.txt`。
- PC 端看到 `ready.txt` 后再读取该目录，避免读到半写入文件。

`result.txt` 示例：

```text
status=OK
format=CODE128
text=ABC123456789
time_ms=37
confidence=92
```

`debug.json` 示例：

```json
{
  "id": 1,
  "status": "OK",
  "format": "CODE128",
  "text": "ABC123456789",
  "roi": [100, 80, 440, 260],
  "time_ms": 37,
  "threshold": 128
}
```

## 开发计划

### 阶段 1：保存当前稳定版本

目标：

- 保留当前已验证的 UVC 稳定版本作为回退点。
- 建议创建 git tag。

建议 tag：

```text
uvc-raw10-vga-reopen-ok-20260811
```

验收：

- Windows Camera 多次打开均有图像。
- 当前 commit 已推送远程仓库。

### 阶段 2：PCA9538A 与 OV01C1B 硬件 Bring-up

目标：

- 实现 PCA9538A I2C 驱动。
- 实现 OV01C1B 上电 / 下电控制流程。
- 验证 OV01C1B I2C 通信正常。

实现要点：

- I2C0 同时挂载 PCA9538A `0x70` 和 OV01C1B 7-bit `0x10`。
- 系统启动后先初始化 PCA9538A：
  - `0x01 <- 0x00`
  - `0x03 <- 0x00`
- 按 OV01C1B 上电流程依次打开 P7、P6、P0、P1、P2、P5。
- 上电完成后再访问 OV01C1B I2C 寄存器。
- 下电流程必须先关闭补光灯和瞄准灯，再关闭时钟、XSHUTDN 和各路电源。

验收：

- PCA9538A 寄存器读写正常。
- OV01C1B 上电时序可重复执行。
- OV01C1B I2C 7-bit 地址 `0x10` 可通信。
- 下电后 PCA9538A `0x01` 输出寄存器回到 `0x00`。

### 阶段 3：OV01C1B Sensor 驱动与 MIPI-CSI 输入

目标：

- 新增或移植 OV01C1B sensor 驱动。
- 使 ESP32-P4 能通过 MIPI-CSI 获取 OV01C1B 图像。

实现要点：

- 根据 `OV01C1B_COB_DS_1.0.pdf` 和 `OV01C10_1024X1024_MIPI1LANE_RAW10_50FPS_V1.6.3_20250513_ori.txt` 实现初始化寄存器配置。
- 明确 OV01C1B 输出格式、分辨率、MIPI lane 数、lane bit rate 和 24MHz 输入时钟要求。
- 先选择适合扫码的稳定分辨率和帧率，不急于追求最高分辨率。
- 参考当前 SC2336 sensor driver 结构实现 OV01C1B 格式表和 detect/init 接口。

验收：

- OV01C1B sensor probe 成功。
- MIPI-CSI 能输出稳定帧。
- 能从 frame buffer 中得到有效图像数据。

### 阶段 4：DSI LCD 本地预览

目标：

- 基于 `camera_dsi` 例程，把 OV01C1B 图像显示到开发板 DSI LCD。
- 预览不依赖 PC。

实现要点：

- LCD 正常预览建议走 ISP，将 OV01C1B RAW 数据转换为 RGB565 或 RGB888。
- 建立 camera capture 到 LCD frame buffer 的显示路径。
- 优先保证稳定预览，再考虑更高分辨率。

验收：

- 开机 LCD 显示实时画面。
- 连续运行不死机、不花屏。

### 阶段 5：USB CDC + MSC 复合设备

目标：

- 将 USB HS 设备改为 CDC ACM + MSC Mass Storage。
- CDC 用于命令和结果。
- MSC 用于调试文件。

实现要点：

- 先做最小复合设备枚举。
- Windows 设备管理器应出现 CDC 串口。
- Windows 资源管理器应出现虚拟 U 盘。
- MSC 第一版建议使用 PSRAM RAM Disk，便于调试，不磨损 Flash。

验收：

- PC 能打开 CDC 串口。
- PC 能看到 MSC 盘符。
- CDC 收到 `STATUS` 能返回 `OK`。

### 阶段 6：CDC 命令控制框架

目标：

- 支持 PC 通过 CDC 控制抓拍和调试。

实现要点：

- 实现命令解析。
- 实现设备状态机：`IDLE`、`CAPTURING`、`DECODING`、`WRITING_FILES`、`DONE`、`ERROR`。
- 支持 `SCAN`、`SNAP`、`DEBUG ON/OFF`、`STATUS`、`CLEAR`。

验收：

- PC 发送 `SCAN`，设备返回 `OK` 或 `BUSY`。
- PC 发送 `STATUS`，设备返回当前状态。

### 阶段 7：抓拍与文件输出

目标：

- 触发时复制当前帧到 PSRAM。
- 保存原图和基础过程图到 MSC。

第一版输出：

- `original.jpg`
- `gray.pgm`
- `result.txt`
- `debug.json`
- `ready.txt`

实现要点：

- 抓拍和 LCD 预览解耦。
- 不在预览路径中长时间阻塞。
- 文件写完后最后生成 `ready.txt`。

验收：

- PC 发送 `SNAP` 后，MSC 中出现新目录。
- 目录中图片可以正常打开。

### 阶段 8：条码解码算法接入

目标：

- ESP32-P4 本地完成条码识别。

第一阶段建议格式：

- Code128
- Code39
- EAN13

候选方案：

- 优先评估 ZBar。
- 如果移植成本或性能不合适，再实现针对 Code128 的轻量算法。

验收：

- PC 发送 `SCAN`。
- ESP32-P4 本地解码。
- CDC 返回 `RESULT OK ...` 或 `RESULT FAIL`。
- MSC 保存解码时刻的原图和过程图片。

### 阶段 9：算法调试图片输出

目标：

- 保存解码中间结果，便于 PC 端分析失败原因。

输出文件：

- `gray.pgm`
- `roi.pgm`
- `binary.pgm`
- `edge.pgm`
- 扫描线数据或边缘统计数据

实现要点：

- `DEBUG ON` 时输出完整调试文件。
- `DEBUG OFF` 时只输出原图和结果，减少写盘时间。
- 阈值、ROI、条码类型等参数可通过 CDC 设置。

验收：

- 解码失败时能从 MSC 文件看出失败原因。
- 调试参数可在线调整。

### 阶段 10：PC 调试工具

目标：

- 用 PC 工具统一控制扫描、读取文件、显示结果。

第一版建议使用 Python：

- 打开 CDC 串口。
- 发送 `SCAN`。
- 等待 `RESULT`。
- 自动查找 MSC 最新 `SCANxxxx` 目录。
- 显示原图、ROI、二值图和解码字符。

验收：

- 一键触发扫描。
- 自动显示结果和调试图片。

### 阶段 11：性能优化与产品化

目标：

- 提高解码速度和稳定性。
- 减少无效文件输出。

优化方向：

- 扫码任务固定到 Core 1，预览和 USB 任务固定或优先安排在 Core 0。
- ROI 裁剪。
- 灰度化加速。
- 二值化参数自适应。
- 解码任务和 LCD 预览任务隔离。
- MSC 写入异步化。
- 根据实际条码类型裁剪算法功能。

验收：

- 常见条码稳定识别。
- CDC 结果返回及时。
- LCD 预览不卡顿。
- MSC 调试文件完整。

## 风险与注意事项

- USB MSC 和 ESP32-P4 写文件同时发生时，需要文件完成标志，避免 PC 读取半写入文件。
- 如果 MSC 使用 PSRAM RAM Disk，掉电后文件会丢失，但适合调试阶段。
- 如果后续改为 Flash FATFS，需要注意 Flash 擦写寿命。
- CDC + MSC 复合设备会替代当前 UVC 设备，PC 预览将由本地 LCD 承担。
- 条码识别算法不应阻塞 LCD 预览任务。
- 高分辨率原图和多张过程图会占用 PSRAM，需要严格控制缓存生命周期。
