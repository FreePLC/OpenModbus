# OpenModbus
OpenModbus is an opensource modbus stack, noblocking function call by main, without RTOS, it support multi-master and slave mode in one device.

TestPlatform:
1. Chip:NXP K64
2. Board:FRDM-K64
3. IDE: IAR
4. SDK: NXP SDK_2.6.0

File description
1. Modbus.c/.h stack core file, no need to modify.
2. Modbus_Porting.c/.h porting file, user need to modify it if change the MCU platform, please fellow this [guide](https://github.com/FreePLC/OpenModbus/blob/master/PortingGuide.md).
3. ModbusSlaveApp.c/.h Slave demo code, user need to modify it according to requirement.
4. ModbusMasterApp.c/.h Master demo code, user need to modify it according to requirement.

OpenModbus是一个开源的Modbus协议栈，采用非阻塞函数调用，无需使用RTOS，支持主/从机模式，并支持多个主从机在同一个设备运行。

运行平台：
1. 芯片：NXP K64
2. 开发板：FRDM-K64
3. 编译环境：IAR
4. SDK：NXP SDK_2.6.0

文件说明：
1. Modbus.c/.h 协议栈核心，用户不需要修改
2. Modbus_Porting.c/.h 移植文件，主要处理UART以及定时器的配置，如果更换MCU平台请根据[移植要点](https://github.com/FreePLC/OpenModbus/blob/master/PortingGuide.md)进行移植
3. ModbusSlaveApp.c/.h 从站示例代码，用户可以根据需求修改
4. ModbusMasterApp.c/.h 主站示例代码，用户可以根据需求修改




