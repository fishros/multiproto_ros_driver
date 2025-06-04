import logging
logging.basicConfig(
    level=logging.WARN,
    format='%(levelname)s:%(filename)s:%(funcName)s:%(lineno)d->%(message)s',
)
logger = logging.getLogger(__name__)


import struct

class DynamicStruct:
    def __init__(self, field_defs):
        """
        field_defs: list of tuples (field_name, format_char)
        Example: [("id", "H"), ("payload", "1024s")]
        """
        self.fields = field_defs
        self.format = ''.join(fmt for _, fmt,_ in field_defs)
        self.size = struct.calcsize(self.format)
        self.values = {name: default_value for name, _,default_value in field_defs}


    def parse(self, data: bytes):
        index = 0
        next_array_size = 0
        for field in self.fields:
            field_name, fmt,_ = field
            field_size = struct.calcsize(fmt)
            # parse current data
            if field_name.startswith("_"):
                self.values[field_name] = struct.unpack(fmt, data[index:index+field_size])[0]
                next_array_size = self.values[field_name]
                index+= field_size
            else:
                if next_array_size:
                    fmt_without_num = fmt[-1]  # Get the format type (like 's', 'H', etc.)
                    dynamic_fmt = f"{next_array_size}{fmt_without_num}"
                    self.values[field_name] = struct.unpack(dynamic_fmt, data[index:index+next_array_size])[0]
                    index+= next_array_size
                    next_array_size = 0
                else:
                    self.values[field_name] = struct.unpack(fmt, data[index:index+field_size])[0]
                    index+= field_size
        
    def to_bytes(self) -> bytes:
        packed = bytearray()
        next_array_size = 0
        for field_name, fmt,_ in self.fields:
            val = self.values.get(field_name)
            
            if field_name.startswith("_"):
                # 普通字段，打包并记录大小供下个字段使用
                # print("field_name",field_name,fmt,val)
                packed += struct.pack(fmt, val)
                next_array_size = val
            else:
                if next_array_size:
                    # 使用前面的字段长度生成动态格式
                    fmt_type = fmt[-1]
                    dynamic_fmt = f"{next_array_size}{fmt_type}"
                    if fmt_type == "s":
                        if isinstance(val, str):
                            val = val.encode()
                        val = val.ljust(next_array_size, b'\x00')
                    packed += struct.pack(dynamic_fmt, val)
                    next_array_size = 0
                else:
                    # 非变长字段，正常处理
                    if fmt.endswith("s") and isinstance(val, str):
                        val = val.encode()
                        val = val.ljust(int(fmt[:-1]), b'\x00')
                    packed += struct.pack(fmt, val)
        return bytes(packed)


    def __getattr__(self, name):
        return self.values[name]

    def __setattr__(self, name, value):
        if name in ("fields", "format", "size", "values"):
            super().__setattr__(name, value)
        else:
            self.values[name] = value


class CircularBuffer:
    def __init__(self, size):
        self.csize = size
        self.buffer = [None] * size
        self.head = 0
        self.tail = 0
        self.count = 0

    def put(self, data):
        logger.debug(f"Buffer size: {self.csize}, Current count: {self.count}, Data length: {len(data)}")
        if len(data) > self.csize - self.count:
            return -1  # Not enough space in buffer
        for byte in data:
            self.buffer[self.tail] = byte
            self.tail = (self.tail + 1) % self.csize
            self.count += 1
        return 0  # Success

    def get(self, n=1):
        if self.count == 0:
            return -1  # Buffer is empty
        if n > self.count:
            return -1  # Not enough data
        data = []
        for _ in range(n):
            data.append(self.buffer[self.head])
            self.head = (self.head + 1) % self.csize
            self.count -= 1
        return data

    def size(self):
        return self.count

    def clear(self):
        self.head = 0
        self.tail = 0
        self.count = 0
        self.buffer = [None] * self.csize

class FProtocolHeader:
    def __init__(self, node=0, type=0, index=0, data_size=0):
        self.node = node
        self.type = type
        self.index = index
        self.data_size = data_size

    @classmethod
    def from_bytes(cls, data):
        if len(data) != 7:
            raise ValueError("Invalid data length for FProtocolHeader")
        node = data[0] | (data[1] << 8)
        type = data[2]
        index = data[3] | (data[4] << 8)
        data_size = data[5] | (data[6] << 8)
        return cls(node, type, index, data_size)

    def to_bytes(self):
        return [
            self.node & 0xFF,
            (self.node >> 8) & 0xFF,
            self.type,
            self.index & 0xFF,
            (self.index >> 8) & 0xFF,
            self.data_size & 0xFF,
            (self.data_size >> 8) & 0xFF
        ]

    def __str__(self):
        return f"FProtocolHeader(node={self.node}, type={self.type}, index={self.index}, data_size={self.data_size})"


class FProtocolType():
    SERVICE_REQUEST_WRITE = 0x01
    SERVICE_REQUEST_READ = 0x02  # Data 0 byte
    SERVICE_RESPONSE_WRITE = 0x03
    SERVICE_RESPONSE_READ = 0x04
    SERVICE_RESPONSE_ERROR = 0x05  # Data 2 bytes -> Error code
    TRANSPORT_DATA = 0x06
    HEART_PING = 0x07
    HEART_PONG = 0x08
    MAX = 0x08
    
class FProtocol:
    FRAME_HEAD = [0x55,0xaa,0x55,0xaa]

    def __init__(self,read_callback,write_callback):
        self.slave_nodes = {}
        self.host_node = None
        self.host_node_proto = None
        self.read_callback = read_callback
        self.write_callback = write_callback
        self.rxbuff = CircularBuffer(2048)
        self.frame = {
            'recv_size': 0,
            'data': [],
            'header': FProtocol.FRAME_HEAD,
            'data_size': 0,
            'fdata': None,
            'error_code': 0
        }
    
    def add_slave_node(self, nodeid, nodeproto):
        self.slave_nodes[nodeid] = nodeproto

    def set_host_node(self, node, nodeproto):
        self.host_node = node
        self.host_node_proto = nodeproto

    def tick(self):
        if self.read_callback:
            data = self.read_callback()
            if data:
                self.rxbuff.put(data)
                logging.debug("put",list(data),self.rxbuff.size())
        
        if self.frame['recv_size'] < 4:
            while self.frame['recv_size'] < 4:
                rdata = self.rxbuff.get(1)
                if rdata == -1:
                    break
                rdata = rdata[0]
                if rdata == FProtocol.FRAME_HEAD[self.frame['recv_size']]:
                    self.frame['data'].append(rdata)
                    self.frame['recv_size'] += 1
                elif rdata == FProtocol.FRAME_HEAD[0]:
                    self.frame['data'] = [rdata]
                    self.frame['recv_size'] = 1
                else:
                    self.frame['recv_size'] = 0
        
        if self.frame['recv_size'] >= 4 and self.frame['recv_size'] < 11:
            additional_data = self.rxbuff.get(11 - self.frame['recv_size'])
            if additional_data == -1:
                return
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            if self.frame['recv_size'] != 11:
                return
            header = self.parse_header(self.frame['data'][4:11])
            self.frame['header']  = header
            logger.debug(f"header={header}")
            if header.node == self.host_node:
                logger.debug(f"Recv data from host node {header.node}")
            elif header.node in self.slave_nodes.keys():
                if header.type == FProtocolType.HEART_PING:
                    self.frame['data_size'] = 0
                    self.frame['fdata'] = None
                    logger.debug(f"Recv data from node {self.frame['header'].node} data_size={self.frame['header'].data_size}")
                else:
                    fdata = self.slave_nodes[self.frame['header'].node].get_index_data(self.frame['header'].index)
                    self.frame['fdata'] = fdata
                    if header.type == FProtocolType.SERVICE_RESPONSE_ERROR:
                        self.frame['data_size'] = self.frame['header'].data_size  # 2
                    elif header.type == FProtocolType.SERVICE_REQUEST_WRITE:
                        self.frame['data_size'] = self.frame['header'].data_size # 0
                    elif header.type == FProtocolType.SERVICE_RESPONSE_READ:
                        self.frame['data_size'] = self.frame['header'].data_size #self.frame['fdata'].csize
                    elif header.type == FProtocolType.TRANSPORT_DATA:
                        self.frame['data_size'] = self.frame['header'].data_size #self.frame['fdata'].csize
                    else:
                        self.frame['recv_size'] = 0 # 重新接收
                # print(self.frame['recv_size'],self.frame['data_size'])
            else:
                self.frame['recv_size'] = 0
                
        if self.frame['recv_size'] >= 11 and self.frame['recv_size'] < (11 + self.frame['data_size']):
            additional_data = self.rxbuff.get(self.frame['data_size'])
            if additional_data == -1:
                return
            self.frame['data'].extend(additional_data)
            self.frame['recv_size'] = len(self.frame['data'])
            logger.debug(f"Recv data from node {self.frame['header'].node} data_size={self.frame['data_size']}")
            # print()
        
        if self.frame['recv_size'] >= (11 + self.frame['data_size']):
            checksum_data = self.rxbuff.get(2) # checksum
            if checksum_data == -1:
                return
            # print("self.frame['recv_size']",self.frame['recv_size'],self.frame['data'],checksum_data)
            calculated_checksum = self.checksum16(self.frame['data'])
            received_checksum = checksum_data[0] << 8 | checksum_data[1]
            if calculated_checksum == received_checksum:
                if self.frame['header'].type == FProtocolType.SERVICE_RESPONSE_ERROR:
                    self.frame['error_code'] = self.frame['data'][11] << 8 | self.frame['data'][12]
                self.process_frame(self.frame)  

            self.frame['data'].clear()
            self.frame['recv_size'] = 0
            # print("self.frame['recv_size']",self.frame['recv_size'])
        if self.frame['recv_size'] > 1024:
            self.frame['recv_size'] = 0


    def process_frame(self,frame):
        header = frame['header']
        if header.node in self.slave_nodes.keys():
            if header.type == FProtocolType.HEART_PING:
                self.fprotocol_write(header.node,FProtocolType.HEART_PONG,0,[])
            elif header.type == FProtocolType.SERVICE_RESPONSE_READ or header.type == FProtocolType.TRANSPORT_DATA:
                # self.fprotocol_unpack_data(frame,bytes(frame['data'][11:]))
                frame['fdata'].parse(bytes(frame['data'][11:]))

            if frame['fdata'] and frame['fdata'].callback:
                frame['fdata'].callback(header.type,frame['fdata'],frame['error_code'])


    # def fprotocol_unpack_data(self,frame,data):
    #     fdata = frame['fdata']
    #     print(fdata.cstruct,fdata.csize,data) # H12sH12s 28 b'\x04\x00\x01\x02\x03\x04\x04\x00\x00\x00\x00\x00' 
    #     # TODO: 根据fdata.cstruct解析数据，遇到字符根据对应长度直接分割解析，遇到数字的要看前一个H的大小来解析
    #     return

    def fprotocol_write(self, node, type, index, data, data_size=0):
        frame = []
        frame.extend(FProtocol.FRAME_HEAD)
        frame.extend([
            node & 0xFF,
            (node >> 8) & 0xFF,
            type,
            index & 0xFF,
            (index >> 8) & 0xFF,
            data_size & 0xFF,
            (data_size >> 8) & 0xFF
        ])
        if data:
            frame.extend(data)
        checksum = self.checksum16(frame)
        frame.append(checksum >> 8 & 0xFF)
        frame.append(checksum & 0xFF)
        self.write_callback(bytes(frame))

    def checksum16(self,data):
        sum_val = 0
        for byte in data:
            sum_val += byte
            # Handle carry (keep sum within 16 bits)
            if sum_val & 0xFFFF0000:
                sum_val = (sum_val & 0xFFFF) + (sum_val >> 16)
        # Return one's complement of the lower 16 bits
        return (~sum_val & 0xFFFF)

    def parse_header(self,data):
        head = FProtocolHeader.from_bytes(data)
        # print("parse_header",head,head.to_bytes())
        return head
    
    def read_put(self, data):
        result = self.rxbuff.put(data)
        if result == -1:
            logger.debug("Buffer is full, cannot write data")
        else:
            logger.debug(f"Data written to buffer: {data}")
            self.tick()
