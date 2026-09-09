# RoboMaster 底盘 USB 串口通信协议与发包格式说明

本说明仅针对 **上位机 (ROS2 / Linux / PC) 与 STM32 底盘主控板** 之间的 USB CDC 虚拟串口双向数据包格式进行定义。

---

## 1. 上位机发送控制包格式 (ROS2 -> STM32)

单次发送固定为 **12 字节** 二进制紧凑数据包（小端模式 Little-Endian）。

### 1.1 协议数据包结构定义 (12 字节)

| 字节偏移 | 字段名称 | 数据类型 | 取值范围 | 物理含义与坐标极性说明 |
| :---: | :---: | :---: | :---: | :--- |
| **Byte 0** | `header` | `uint8` | `0xA5` | **固定帧头**，十进制 165，十六进制 `0xA5` |
| **Byte 1 ~ 2** | `stick_vx` | `int16` (小端) | `-660 ~ +660` | **前后波杆量**：**正数前进 (+X)**，负数后退 (-X) |
| **Byte 3 ~ 4** | `stick_vy` | `int16` (小端) | `-660 ~ +660` | **左右平移杆量**：**正数左移 (+Y)**，负数右移 (-Y) |
| **Byte 5 ~ 6** | `stick_wz` | `int16` (小端) | `-660 ~ +660` | **自转杆量**：**正数向左自转/逆时针 (+Wz)**，负数向右自转/顺时针 (-Wz) |
| **Byte 7** | `ctrl_mode` | `uint8` | `0 或 1` | **模式标志**：`0` = 普通麦轮底盘模式，`1` = 开启小陀螺模式 |
| **Byte 8** | `flags` | `uint8` | `0` | 功能保留字节，默认填 `0` |
| **Byte 9 ~ 10** | `reserved` | `uint16` (小端) | `0` | 内存对齐保留字节，填 `0` |
| **Byte 11** | `checksum` | `uint8` | `0 ~ 255` | **累加和校验**：前 11 个字节所有数值累加后取低 8 位 (`sum & 0xFF`) |

### 1.2 校验和计算公式
```text
checksum = (Byte0 + Byte1 + Byte2 + ... + Byte10) & 0xFF
```

---

## 2. 数据发包代码示例

### 2.1 Python 发包示例 (最简 5 行)

```python
import struct
import serial

# 1. 打开虚拟串口 (波特率任意，默认推荐 115200)
ser = serial.Serial('/dev/ttyACM0', 115200)

# 2. 设定期望波杆量 (范围: -660 ~ +660，0 为静止)
vx = 660     # 前进 (正前负后)
vy = 0       # 横移 (正左负右)
wz = 0       # 旋转 (正左逆时针，负右顺时针)

# 3. 按照小端格式打包前 11 字节 (<BhhhBBH)
data = struct.pack('<BhhhBBH', 0xA5, vx, vy, wz, 0, 0, 0)

# 4. 计算校验和并拼装为 12 字节数据包
packet = data + bytes([sum(data) & 0xFF])

# 5. 串口发送
ser.write(packet)
```

### 2.2 C / C++ 发包示例

```c
#include <stdint.h>
#include <unistd.h>

#pragma pack(push, 1)
typedef struct {
    uint8_t  header;       // 0xA5
    int16_t  stick_vx;     // 前后: 正前负后 [-660, 660]
    int16_t  stick_vy;     // 横移: 正左负右 [-660, 660]
    int16_t  stick_wz;     // 自转: 正左逆时针, 负右顺时针 [-660, 660]
    uint8_t  ctrl_mode;    // 0: 普通底盘, 1: 小陀螺
    uint8_t  flags;        // 0
    uint16_t reserved;     // 0
    uint8_t  checksum;     // 前 11 字节累加和
} Chassis_Cmd_Packet_t;
#pragma pack(pop)

void Send_Chassis_Packet(int serial_fd, int16_t vx, int16_t vy, int16_t wz) {
    Chassis_Cmd_Packet_t pkt;
    pkt.header    = 0xA5;
    pkt.stick_vx  = vx;
    pkt.stick_vy  = vy;
    pkt.stick_wz  = wz;
    pkt.ctrl_mode = 0;
    pkt.flags     = 0;
    pkt.reserved  = 0;

    // 计算前 11 字节累加和
    uint8_t *p = (uint8_t*)&pkt;
    uint8_t sum = 0;
    for (int i = 0; i < 11; i++) {
        sum += p[i];
    }
    pkt.checksum = sum;

    // 串口发送 12 字节
    write(serial_fd, &pkt, sizeof(pkt));
}
```

---

## 3. 上位机接收反馈包格式 (STM32 -> ROS2)

STM32 底盘以 **50Hz** 向上位机自动回传 **20 字节** 二进制状态帧。

| 字节偏移 | 字段名称 | 数据类型 | 单位 | 说明 |
| :---: | :---: | :---: | :---: | :--- |
| **Byte 0** | `header` | `uint8` | - | **固定帧头**，固定为 `0x5A` (十进制 90) |
| **Byte 1 ~ 4** | `yaw_deg` | `float` (32位单精度) | 度 (°) | **实时航向角**：**前进时左偏为正 (+)，右偏为负 (-)** |
| **Byte 5 ~ 8** | `wz_dps` | `float` (32位单精度) | °/s | **Z轴角速度**：**逆时针向左自转为正 (+)** |
| **Byte 9 ~ 10** | `actual_vx_mms` | `int16` | mm/s | 编码器实际解算的前进线速度 |
| **Byte 11 ~ 12**| `actual_vy_mms` | `int16` | mm/s | 编码器实际解算的横移线速度 |
| **Byte 13 ~ 14**| `actual_wz_mrads` | `int16` | mrad/s | 编码器实际解算的自转角速度 |
| **Byte 15** | `sys_state` | `uint8` | - | 系统状态：`0` = 失能, `1` = 已使能 |
| **Byte 16** | `active_mode` | `uint8` | - | 控制模式：`0` = 手动遥控, `1` = 自动控制 (ROS2), `2` = 急停 |
| **Byte 17** | `rc_s_right` | `uint8` | - | 遥控器右拨杆档位：`1` = 上档, `3` = 中档, `2` = 下档 |
| **Byte 18** | `reserved` | `uint8` | - | 预留对齐字节 |
| **Byte 19** | `checksum` | `uint8` | - | 前 19 字节累加和取低 8 位 (`sum & 0xFF`) |

---

## 4. 发包要求与控制生效前提

1. **使能前提条件**：
   - 遥控器开机并双摇杆归中完成开机自检；
   - **必须将遥控器右侧拨杆切至最上方（`S2 == 1` 上档）**，底盘才会进入自动控制模式并执行上位机下发的数据包；
   - 若拨杆在中间（`S2 == 3`），底盘由遥控器直接控制；若拨杆在下方（`S2 == 2`），底盘处于硬件急停切断电流状态。
2. **发送频率**：建议以 **20Hz ~ 50Hz**（每 20ms ~ 50ms 一包）循环下发。
3. **心跳保护（看门狗）**：STM32 内部设置了 **200ms 心跳看门狗**。若上位机发包中断超过 200ms，底盘会自动将速度置零刹停停机。
