import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import socket
from multiproto_driver.controllerptoto import ControllerProto
from multiproto_driver.fprotocol import *
import time
import threading
from robot_interfaces.msg import RawUInt8
from robot_interfaces.msg  import CanFrame
from robot_interfaces.msg  import IOStruct
import traceback

class UDPServerNode(Node):
    def __init__(self):
        super().__init__('udp_server_node')
        # Create callback groups
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        # UDP setup
        self.host = '0.0.0.0'
        self.declare_parameter('port', 8888)
        self.port = self.get_parameter('port').value
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.1)  # Set socket timeout
        # Protocol setup
        self.handler = None
        self.addr = None
        self.last_call_time = time.time()
        self.call_count = 0
        # Initialize protocol handler
        self.handler = FProtocol(self.read_callback, self.write_callback)
        self.proto = ControllerProto()
        self.proto.read_can.callback = self.can_read_callback
        self.proto.read_485.callback = self.rs485_read_callback
        self.proto.read_io.callback = self.read_io_callback
        self.handler.add_slave_node(0x0001, self.proto)
        # Create timer for UDP receiving
        self.create_timer(0.01, self.udp_receive_callback, callback_group=self.timer_callback_group)
        # Create publisher and subscriber
        self.read_485_pub = self.create_publisher(RawUInt8, '/read_485', 10)
        self.write_485_sub = self.create_subscription(
            RawUInt8,
            '/write_485',
            self.write_485_callback,
            10)
        self.get_logger().info(f"UDP server listening on {self.host}:{self.port}")

        # Create publisher and subscriber for CAN
        self.read_can_pub = self.create_publisher(CanFrame, '/read_can', 10)
        self.write_can_sub = self.create_subscription(
            CanFrame,
            '/write_can',
            self.write_can_callback,
            10)
        # Create publisher and subscriber for IO
        self.read_io_pub = self.create_publisher(IOStruct, '/read_io', 10)
        self.write_io_sub = self.create_subscription(
            IOStruct,
            '/write_io',
            self.write_io_callback,
            10)

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
        return None

    def write_callback(self, data):
        if self.addr:
            # self.get_logger().info(f"Sending {len(data)} bytes to {self.addr}")
            self.sock.sendto(data, self.addr)

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

    def udp_receive_callback(self):
        try:
            data, addr = self.sock.recvfrom(4096)
            # Check if this is a new client connection
            if self.addr is None or self.addr != addr:
                self.get_logger().info(f"Client connected: {addr[0]}:{addr[1]}")
            self.addr = addr
            self.get_logger().info(f"Read Data {len(data)}")
            self.handler.read_put(data)
            self.handler.tick() # Call tick() to process received data
        except socket.timeout:
            pass
        except Exception as e:
            self.get_logger().error(f"Stack trace:\n{traceback.format_exc()}")
            self.get_logger().error(f"Error in UDP receive: {str(e)}")


    def __del__(self):
        self.sock.close()

def main(args=None):
    rclpy.init(args=args)
    node = UDPServerNode()
    # Create and use a MultiThreadedExecutor
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
