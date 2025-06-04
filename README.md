# 多协议传输控制器 ROS2 驱动 & 使用说明

* **产品购买链接**：[点此购买](https://m.tb.cn/h.g4sKEcOnkEoye4m?tk=o3wOWCqHrlL)
* **ROS2 驱动仓库**：[https://github.com/fishros/multiproto\_ros2\_driver](https://github.com/fishros/multiproto_ros2_driver)

---

## 一、固件下载与配置助手使用

> **前提**：首次使用设备前，请先刷入最新固件并配置通信参数。

### 1.1 下载配置助手

| 系统      | 下载地址                                                                                                                                                                 |
| ------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Linux   | [fishbot\_tool.linux\_amd64](http://github.fishros.org/https://github.com/fishros/fishbot_tool/releases/download/v1.2.3.alpha/fishbot_tool.v1.2.3.alpha.linux_amd64) |
| Windows | [fishbot\_tool.win.exe](http://github.fishros.org/https://github.com/fishros/fishbot_tool/releases/download/v1.2.3.alpha/fishbot_tool.v1.2.3.alpha.win.exe)          |

### 1.2 固件烧录步骤

1. USB连接控制器至电脑。
2. 打开配置助手，选择串口和设备类型（**ROS2多协议传输控制板**）。
3. 点击“下载”按钮进行固件刷新。

---

## 二、通信参数配置

### 2.1 进入配置模式

* **长按 RST（复位键）** 直到 LED 不闪烁，表示已进入配置模式。
* 配置完成后**长按 RST（复位键）** LED闪烁进入运行模式。

### 2.2 通信模式选择

| 模式      | 配置字符串        | 描述       |
| ------- | ------------ | -------- |
| 静态IP以太网 | `eth_static` | 需手动设定 IP |
| DHCP以太网 | `eth_dhcp`   | 自动获取 IP  |
| USB串行模式 | `usb`        | USB转串口   |

### 2.3 各模式所需配置项

#### 🔌 以太网静态模式 `eth_static`

| 参数               | 示例                |
| ---------------- | ----------------- |
| `transport_mode` | `eth_static`      |
| `eth_ip`         | `192.168.168.250` |
| `eth_gateway`    | `192.168.168.1`   |
| `eth_subnet`     | `255.255.255.0`   |
| `eth_dns`        | `8.8.8.8`         |
| `server_ip`      | `192.168.168.5`   |
| `server_port`    | `8888`            |

#### 🔌 USB串行模式 `usb`

| 参数               | 示例       |
| ---------------- | -------- |
| `transport_mode` | `usb`    |
| `usb_baudrate`   | `921600` |

### 2.4 通用参数

| 参数           | 默认值  | 描述             |
| ------------ | ---- | -------------- |
| `readio_hz`  | 10   | IO状态上报频率 (Hz)  |
| `can_rate`   | 100  | CAN总线速率 (kbps) |
| `rs485_rate` | 9600 | 485总线波特率 (bps) |

### 2.5 快速切换通信模式

* 可**双击 KEY 按键**在不同通信模式间快速切换。

---

## 三、ROS2 驱动安装与运行

### 3.1 安装驱动

```bash
git clone https://github.com/fishros/multiproto_ros_driver
cd multiproto_ros_driver
colcon build
source install/setup.bash
```

### 3.2 启动驱动

#### USB 串口模式

```bash
ros2 run multiproto_driver multiproto_driver_serial --ros-args -p port:=/dev/ttyUSB0 -p baudrate:=921600
```

#### UDP 模式（适用于 `eth_static` 和 `eth_dhcp`）

```bash
ros2 run multiproto_driver multiproto_driver_udp --ros-args -p port:=8888
```

---

## 四、通信测试与验证

### 4.1 检查话题列表

```bash
ros2 topic list
```

应看到如下话题，表示驱动正常工作：

```
/parameter_events  
/read_485  
/read_can  
/read_io  
/rosout  
/write_485  
/write_can  
/write_io
```

### 4.2 验证 /read\_io 话题自动上报

```bash
ros2 topic echo /read_io
```

示例输出：

```yaml
---
index: [1, 2, 3, 4]
data:  [0, 0, 0, 0]
---
```

查看频率：

```bash
ros2 topic hz /read_io
```

示例输出：

```
average rate: 10.000
min: 0.090s max: 0.110s std dev: 0.00424s window: 12
```

---

## 五、例程测试：RS485温湿度传感器

### 5.1 硬件接线

| 控制器     | 传感器  |
| ------- | ---- |
| 5V/GND  | 电源供电 |
| 485 A/B | 通信连接 |

> 建议从控制器的 POWER OUT 接口取电，避免外部干扰。

### 5.2 启动例程

```bash
ros2 run example_rs485_temp_py temp_read
```

### 5.3 示例输出

```bash
[INFO] 发布读取命令 {01 04 00 00 00 02 71 CB}
[INFO] 收到数据 01 04 04 00 EE 02 37 DB 07
[INFO] 接收 ID: 0x01 温度: 23.80 湿度: 56.70
```

