# CanFrame.msg
# 这是用于描述CAN数据帧的ROS2消息接口定义。

# 标准ID或扩展ID
uint32 id

# 数据帧的长度（以字节为单位）
uint8 dlc

# 实际传输的数据
uint8[8] data

# 是否为扩展ID（如果是扩展ID，则为1，标准ID为0）
bool is_extended_id

# 是否为远程帧
bool is_rtr
