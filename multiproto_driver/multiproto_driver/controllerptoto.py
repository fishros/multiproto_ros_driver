import struct
from multiproto_driver.fprotocol import *
class ControllerProto:
    rawuint8_t_desc = [("_data_size","H",0),("data","1024s",b'\x00' * 1024)]
    canframe_t_desc = [("id","I",0),("dlc","B",0),("ext","B",0),("rtr","B",0),("_data_size","H",0),("data","8s",b'\x00' * 8)]
    io_t_desc = [("_index_size","H",0),("index","12s",b'\x00' * 12),("_data_size","H",0),("data","12s",b'\x00' * 12)]

    def __init__(self):
        self.index_table = {}
        self.write_485 = DynamicStruct(self.rawuint8_t_desc)
        self.index_table[0x0001] = self.write_485
        self.read_485 = DynamicStruct(self.rawuint8_t_desc)
        self.index_table[0x0002] = self.read_485
        self.write_can = DynamicStruct(self.canframe_t_desc)
        self.index_table[0x0003] = self.write_can
        self.read_can = DynamicStruct(self.canframe_t_desc)
        self.index_table[0x0004] = self.read_can
        self.write_io = DynamicStruct(self.io_t_desc)
        self.index_table[0x0005] = self.write_io
        self.read_io = DynamicStruct(self.io_t_desc)
        self.index_table[0x0006] = self.read_io

    def get_index_data(self,index):
        return self.index_table[index]

    def write_write_485(self,fprotocol,type,node):
        bytes_data = self.write_485.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0001,self.write_485.to_bytes(),len(bytes_data))

    def write_read_485(self,fprotocol,type,node):
        bytes_data = self.read_485.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0002,self.read_485.to_bytes(),len(bytes_data))

    def write_write_can(self,fprotocol,type,node):
        bytes_data = self.write_can.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0003,self.write_can.to_bytes(),len(bytes_data))

    def write_read_can(self,fprotocol,type,node):
        bytes_data = self.read_can.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0004,self.read_can.to_bytes(),len(bytes_data))

    def write_write_io(self,fprotocol,type,node):
        bytes_data = self.write_io.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0005,self.write_io.to_bytes(),len(bytes_data))

    def write_read_io(self,fprotocol,type,node):
        bytes_data = self.read_io.to_bytes()
        fprotocol.fprotocol_write(node,type,0x0006,self.read_io.to_bytes(),len(bytes_data))