# bread-compact-wifi-s3-wty 板载血压计串口对接说明

本文针对 `CompactWifiBoardS3Wty` 板卡如何通过 YP0X 串口协议与下位机血压计交互做详细说明，涵盖硬件连接、协议格式、固件实现与使用方式。

## 板卡概述
- 控制核心：ESP32-S3（Wi‑Fi SoC），固件类 `CompactWifiBoardS3Wty` 继承自通用 `WifiBoard`。
- 按键与指示：板载 boot 按键 (`BOOT_BUTTON_GPIO`) 以及单色 LED (`BUILTIN_LED_GPIO`) 由 `Button`、`SingleLed` 类管理。
- 显示系统：SPI 接口 LCD（根据编译选项可为 GC9A01 或 ILI9341），`LcdDisplay` 负责显示驱动，`PwmBacklight` 管理背光亮度。
- 音频接口：通过 `NoAudioCodec(Simplex/Duplex)` 提供空实现，默认无实际音频编解码器。
- 血压计接口：`Yp0xHost` 负责与外接血压计的 UART 通信，相关逻辑集中在 `InitializeBloodPressureInterface()`。

## UART 硬件连接

| 项目 | 固件配置 | 说明 |
| --- | --- | --- |
| UART 端口 | `UART_NUM_1` (`BP_UART_PORT`) | 独立于日志口的第二路串口 |
| 波特率 | `115200` (`BP_UART_BAUDRATE`) | 8-N-1，无硬件流控 |
| TX 引脚 | `GPIO_NUM_9` (`BP_UART_TX_PIN`) | 连接到血压计 RX |
| RX 引脚 | `GPIO_NUM_10` (`BP_UART_RX_PIN`) | 连接到血压计 TX |

固件启动后会调用 `uart_driver_install` / `uart_param_config` / `uart_set_pin` 完成驱动初始化，并创建一个 FreeRTOS 任务持续读取血压计数据。

## YP0X UART 协议概要

### 帧结构
所有帧均以 0xFF 作为帧头，随后一个长度字节（含 `command`、数据及校验），整体格式如下：

```
┌─────────┬────────┬─────────┬─────────────┬──────────┐
│ 0xFF    │ LEN    │ COMMAND │ PAYLOAD ... │ CHECKSUM │
└─────────┴────────┴─────────┴─────────────┴──────────┘
```

- `COMMAND` 实际有效位为低 6 位（`kCommandIdMask = 0x3F`），若最高位 0x40 置位表示需要对端 ACK。
- `CHECKSUM` 为从 `LEN` 字节开始累加求和（含 `COMMAND` 与数据）的低 8 位；若结果等于 0xFF 会减 1 以避免与帧头冲突。

### 主机（板卡）下行命令

| Command ID | 数据字节 | 语义 | 对应函数 |
| --- | --- | --- | --- |
| `0x10` | `0x01` | 启动测量 | `Yp0xHost::StartMeasure()` |
| `0x10` | `0x00` | 停止测量/待机 | `Yp0xHost::StopMeasure()` |
| `0x13` | `0x00` | 查询版本号 | `Yp0xHost::QueryVersion()` |
| `0x13` | `0x01` | 查询最后一次测量结果 | `Yp0xHost::QueryLastMeasurement()` |

默认均不要求 ACK（`need_ack = false`），因此发出的 `COMMAND` 字节就是原始 ID。

### 血压计上行数据

| Command ID | 类型 | 说明 |
| --- | --- | --- |
| `0x31`（长帧） | 测量完成推送 | 负载长度 ≥ 8 字节，携带一次完整的血压数据。 |
| `0x31`（短帧） | 心跳状态 | 负载长度 < 8，仅表示设备在线。固件会忽略该帧的测量解析。 |
| `0x32`（长帧） | `0x13/0x01` 命令的响应 | 负载格式同 `0x31`，表示查询到的历史测量值。 |
| `0x39` | 版本信息 | 4 字节，分别为主版本、次版本、修订号（低 7 位有效）及型号。 |

测量结果负载的字段定义：

```
offset 0: info                状态/标志位
offset 1-2: systolic          收缩压 (little endian, 单位 mmHg)
offset 3: diastolic           舒张压 (mmHg)
offset 4: mean                平均压 (MAP, mmHg)
offset 5-6: pulse             脉搏/心率 (little endian, 次/分钟)
offset 7: reserved            保留
```

固件解析后会通过 `Yp0xResult` 结构体对外提供上述信息。

## 固件集成流程

### 初始化
`CompactWifiBoardS3Wty::InitializeBloodPressureInterface()` 在构造函数中调用，主要步骤：

1. 创建或清空二值信号量 `bp_result_sem_`，用于同步查询命令。
2. 调用 `blood_pressure_host_.Init()` 完成 UART 配置。
3. 注册两个回调：
   - `OnVersion`：缓存版本号到 `bp_version_` 并写日志。
   - `OnResult`：缓存测量值、判断是否需要播报并释放信号量。
4. 启动串口接收任务 `Yp0xHost::Start()`，标记 `blood_pressure_initialized_ = true`。

`Yp0xHost` 内部维护一个 64 字节缓冲的状态机（`RxState`），逐字节解析完整帧并进行校验。校验通过后根据 `command_id` 分派给不同的处理逻辑。

### MCP 工具接口
固件在 `InitializeBloodPressureInterface()` 中向 `McpServer` 注册了以下工具，供上层（例如 MCP Agent）调用：

| 工具名 | 功能 | 关键行为 |
| --- | --- | --- |
| `self.bp.start_measure` | 启动测量 | 发送 `0x10/0x01`，并将 `bp_auto_report_pending_` 设为 true，提示下一帧需要自动播报。 |
| `self.bp.stop_measure` | 停止测量 | 发送 `0x10/0x00`，血压计进入待机。 |
| `self.bp.query_version` | 查询版本号 | 发送 `0x13/0x00`，等待血压计上行 `0x39` 帧，版本随后可通过缓存接口读取。 |
| `self.bp.query_last` | 查询最近测量 | 发送 `0x13/0x01` 并阻塞等待信号量（最多 1500 ms）；若成功收到 `0x32` 帧则返回格式化字符串。 |
| `self.bp.get_cached_version` | 获取缓存版本 | 若之前收到过 `0x39` 帧，则返回如 `1.0.3 (model 0x07)` 的文本；否则提示暂无信息。 |
| `self.bp.get_cached_result` | 获取缓存测量 | 返回最近收到并缓存的测量字符串；如果还没有数据会提示暂无结果。 |

除 `query_*` 系列外，其余工具直接返回布尔或字符串结果，无需额外等待。

### 自动播报与缓存策略

- 最近一次测量值保存在 `last_bp_result_`，最后一次对外播报的结果存于 `last_announced_bp_result_`，二者便于比较是否需要再次播报。
- `bp_auto_report_pending_` 置位时，下一次收到测量值即使与之前一致也会通过 `AnnounceBloodPressure()` 主动推送。
- `AnnounceBloodPressure()` 会构造一条 JSON-RPC 消息，通过 `Application::GetInstance().SendMcpMessage()` 发送给上层客户端，默认输出中文描述，例如：“最新测量结果：收缩压 120 毫米汞柱，舒张压 80 毫米汞柱，心率 75 次每分钟。”
- 在 `self.bp.query_last` 的查询场景中，接受到结果后会清除 `bp_auto_report_pending_`，避免重复播报。

## 测量交互流程示例

1. 调用 `self.bp.start_measure`，血压计进入测量状态。
2. 等待血压计完成测量，设备主动以 `0x31` 长帧上报数据；固件解析后：
   - 更新缓存；
   - 触发 MCP 消息推送（若条件满足）；
   - 释放 `bp_result_sem_`，唤醒可能正在等待的查询任务。
3. 若需要主动查询，则调用 `self.bp.query_last`，固件会发送 `0x13/0x01` 并等待 `0x32` 响应，然后返回格式化文本，如：`info=0x00 systolic=120 diastolic=80 mean=90 pulse=75 extra=0x00`。
4. 查询版本时调用 `self.bp.query_version`，随后再调用 `self.bp.get_cached_version` 查看缓存结果。
5. 完成测量后可选调用 `self.bp.stop_measure` 让血压计进入低功耗状态。

## 调试与排查建议
- 在终端开启 `Yp0xHost` 的日志级别，可查看每一帧的十六进制内容和解析结果（关键日志 TAG 为 `Yp0xHost`）。
- 若 `self.bp.query_last` 超时，确认信号量是否被释放以及血压计是否返回 `0x32` 帧。
- 使用 `self.bp.get_cached_result` 可快速检查固件当前缓存的数据，不会触发实际串口通信。
- 若需要扩展更多指令，可在 `Yp0xHost::SendCommand()` 中复用现有帧组装逻辑，注意保持校验规则一致。

## 相关源码
- `main/boards/bread-compact-wifi-s3-wty/compact_wifi_board_s3_wty.cc`
- `main/boards/bread-compact-wifi-s3-wty/config.h`
- `main/protocols/yp0x_uart.h`
- `main/protocols/yp0x_uart.cc`

更新协议或新增命令时，请同步维护本文档以保持信息一致。
