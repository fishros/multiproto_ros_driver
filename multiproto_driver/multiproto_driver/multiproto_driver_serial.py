import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import serial
from multiproto_driver.controllerptoto import ControllerProto
from multiproto_driver.fprotocol import *
import time
from robot_interfaces.msg import RawUInt8
from robot_interfaces.msg import CanFrame
from robot_interfaces.msg import IOStruct
import traceback

class SerialServerNode(Node):
    def __init__(self):
        super().__init__('serial_server_node')
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # Create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Serial setup
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                timeout=0.1
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise e

        # Protocol setup
        self.handler = FProtocol(self.read_callback, self.write_callback)
        self.proto = ControllerProto()
        self.proto.read_can.callback = self.can_read_callback
        self.proto.read_485.callback = self.rs485_read_callback
        self.proto.read_io.callback = self.read_io_callback
        self.handler.add_slave_node(0x0001, self.proto)

        # Create timer for serial receiving
        self.create_timer(0.01, self.serial_receive_callback, callback_group=self.timer_callback_group)

        # Create publishers and subscribers
        self.read_485_pub = self.create_publisher(RawUInt8, '/read_485', 10)
        self.write_485_sub = self.create_subscription(
            RawUInt8,
            '/write_485',
            self.write_485_callback,
            10)

        self.read_can_pub = self.create_publisher(CanFrame, '/read_can', 10)
        self.write_can_sub = self.create_subscription(
            CanFrame,
            '/write_can',
            self.write_can_callback,
            10)

        self.read_io_pub = self.create_publisher(IOStruct, '/read_io', 10)
        self.write_io_sub = self.create_subscription(
            IOStruct,
            '/write_io',
            self.write_io_callback,
            10)

        self.get_logger().info(f"Serial server started on {port} at {baudrate} baud")

    # [Previous callback methods remain the same]
    def write_io_callback(self,msg):
        self.get_logger().info(f"{msg}")
        self.proto.write_io.index = bytes(msg.index)+b'\x00'*(4-len(msg.index))
        self.proto.write_io.data = bytes(msg.data)+b'\x00'*(4-len(msg.index))
        self.proto.write_write_io(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)
    
    def write_can_callback(self, msg):
        self.proto.write_can.id = msg.id
        self.proto.write_can._data_size = msg.dlc
        self.proto.write_can.data = bytes(msg.data)
        self.proto.write_can.dlc = msg.dlc
        self.proto.write_can.ext = 1 if msg.is_extended_id else 0
        self.proto.write_can.rtr = 1 if msg.is_rtr else 0
        self.proto.write_write_can(self.handler, FProtocolType.TRANSPORT_DATA, 0x0001)

    def write_485_callback(self, msg):
        self.proto.write_485.data = bytes(msg.data)
        self.proto.write_485._data_size = msg.size
        self.get_logger().info(f"write_485_callback len={msg.size} -> {list(msg.data)}")
        self.proto.write_write_485(self.handler,FProtocolType.TRANSPORT_DATA, 0x0001)

    def read_callback(self):
        if self.serial.in_waiting:
            return self.serial.read(self.serial.in_waiting)
        return None

    def write_callback(self, data):
        try:
            # self.get_logger().info(f"Sending {len(data)} bytes")
            self.serial.write(data)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

    def can_read_callback(self, type, from_node, error_code):
        msg = CanFrame()
        msg.id = self.proto.read_can.id
        msg.data = list(self.proto.read_can.data)
        msg.dlc = self.proto.read_can.dlc
        msg.is_extended_id = self.proto.read_can.ext==1
        msg.is_rtr = self.proto.read_can.rtr==1
        self.read_can_pub.publish(msg)

    def rs485_read_callback(self, type, from_node, error_code):
        msg = RawUInt8()
        msg.size = self.proto.read_485._data_size
        msg.data = list(self.proto.read_485.data[:msg.size])
        self.read_485_pub.publish(msg)

    def read_io_callback(self, type, from_node, error_code):
        msg = IOStruct()
        msg.index = list(self.proto.read_io.index)
        msg.data = list(self.proto.read_io.data)
        self.read_io_pub.publish(msg)

    def serial_receive_callback(self):
        try:
            if self.serial.in_waiting:
                data = self.serial.read(self.serial.in_waiting)
                self.handler.read_put(data)
                self.handler.tick()
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read error: {e}")
        except Exception as e:
            self.get_logger().error(f"Stack trace:\n{traceback.format_exc()}")
            self.get_logger().error(f"Error in serial receive: {str(e)}")

    def __del__(self):
        if hasattr(self, 'serial') and self.serial.is_open:
            self.serial.close()

def main(args=None):
    rclpy.init(args=args)
    node = SerialServerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
