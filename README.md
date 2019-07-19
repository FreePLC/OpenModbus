# OpenModbus
OpenModbus是一个开源的Modbus协议栈，支持主/从机模式，并支持多个主从机在同一个设备运行。
运行平台：
1. 芯片：NXP KV4x
2. 开发板：TWR-KV4x
3. 编译环境：IAR
4. SDK：NXP SDK_2.4.1
文件说明：
1. Modbus.c/.h 协议栈核心，用户不需要修改
2. Modbus_Porting.c/.h 移植文件，主要处理UART的接收/发送，更换MCU平台时需要用户修改
3. ModbusSlaveApp.c/.h 从站示例代码，用户可以根据需求修改
4. ModbusMasterApp.c/.h 主站示例代码，用户可以根据需求修改



OpenModbus is an opensource modbus stack, it support multi-master and slave mode in one device.
TestPlatform:
1. Chip:NXP KV4x
2. Board:TWR-KV4x
3. IDE: IAR
4. SDK: NXP SDK_2.4.1
File description
1. Modbus.c/.h stack core file, no need to modify.
2. Modbus_Porting.c/.h porting file, user need to modify it if change the MCU platform.
3. ModbusSlaveApp.c/.h Slave demo code, user need to modify it according to requirement.
4. ModbusMasterApp.c/.h Master demo code, user need to modify it according to requirement.
