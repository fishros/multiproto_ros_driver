import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_interfaces.msg import RawUInt8
import time

class Write485Publisher(Node):

    def __init__(self):
        super().__init__('write_485_publisher')
        self.publisher_ = self.create_publisher(RawUInt8, '/write_485', 10)
        self.subscription_ = self.create_subscription(
            RawUInt8,
            '/read_485',
            self.handle_read_485,10)
        self.timer_ = self.create_timer(2.0, self.publish_message)

    def publish_message(self):
        '''
        在Modbus RTU协议中，数据包 {0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB} 具有以下含义：

        1. **0x01**：从站地址。这表明命令是发送到地址为1的设备。
        2. **0x04**：功能码。0x04表示读输入寄存器（Read Input Registers）。
        3. **0x00, 0x00**：起始地址。0x0000表示从第一个寄存器开始读。
        4. **0x00, 0x02**：寄存器数量。0x0002表示读取两个寄存器。
        5. **0x71, 0xCB**：CRC校验码。用于错误检查，确保数据传输的完整性和正确性。

        总结起来，这个数据包的含义是：从Modbus网络中的地址为1的设备读取两个输入寄存器，从地址0x0000开始。
        '''
        message = RawUInt8()
        message.size = 8
        message.data = [0x01, 0x04, 0x00, 0x00, 0x00, 0x02, 0x71, 0xCB]
        self.get_logger().info("------------------------------------------------------------")
        self.get_logger().info("发布读取命令 {01 04 00 00 00 02 71 CB}")
        self.publisher_.publish(message)

    def handle_read_485(self, msg):
        '''
        返回数据包 `01 04 04 01 33 02 31 CA C3` 的含义如下：

        1. **01**：从站地址。这表明返回数据来自地址为1的设备。
        2. **04**：功能码。0x04表示读输入寄存器（Read Input Registers）。
        3. **04**：数据字节数。这表明返回的寄存器数据有4个字节。
        4. **01 33 02 31**：寄存器数据。这4个字节是两个寄存器的数据，组合成32位数据。
        5. **CA C3**：CRC校验码。用于错误检查，确保数据传输的完整性和正确性。

        具体的数据部分 `01 33 02 31` 可以拆分和解释为两个16位寄存器的数据：

        - **01 33**：第一个寄存器的数据。
        - **02 31**：第二个寄存器的数据。

        为了得到实际的测量值，我们需要参考设备的文档来解析这些寄存器值。例如，温度和湿度的解析方式等。

        让我们假设这两个寄存器的数据是温度和湿度值，按照手册中的解析方式：

        - 温度值的解析：寄存器值（16位）表示温度，单位是摄氏度，固定1位小数点。
        - 湿度值的解析：寄存器值（16位）表示湿度，单位是百分比，固定1位小数点。

        根据温湿度采集模块的手册信息：

        - **温度值 = 0x0133 = 307**
          - 解析：307 × 0.1 = 30.7°C

        - **湿度值 = 0x0231 = 561**
          - 解析：561 × 0.1 = 56.1%RH

        因此，这个返回数据表示：
        - 温度值：30.7°C
        - 湿度值：56.1%RH

        '''
        if msg.size != 9:
            self.get_logger().error("Received message size is not 9 bytes.")
            return

        received_data = " ".join([f"{byte:02x}" for byte in msg.data])
        self.get_logger().info(f"收到数据 {received_data}")

        id = msg.data[0]
        function_code = msg.data[1]
        byte_count = msg.data[2]

        # 解析温度和湿度数据
        temp_raw = (msg.data[3] << 8) | msg.data[4]
        hum_raw = (msg.data[5] << 8) | msg.data[6]

        # 假设温度和湿度解析方式为每个数据乘以0.1
        temperature = temp_raw * 0.1
        humidity = hum_raw * 0.1
        if temp_raw > 10000:
            temperature = -1 * (temp_raw - 10000) * 0.1

        self.get_logger().info(f"接收 ID: 0x{id:02x} 温度: {temperature:.2f} 湿度: {humidity:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = Write485Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
