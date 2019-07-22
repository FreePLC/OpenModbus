There are five generic functions and one ISR function(one port Modbus)must be porting to other MCU platform:

  void	(*Timer0_Init)(void);   //init the timer <br>
  uint32_t	(*Timer0_Value_Get)(void);  // get timer value <br>
  uint32_t	(*Timer0_Wait3_5char)(void);  //get modbus 3.5char value. <br>
  void    (*UART_SendData)(uint8_t *pbyData, uint16_t uCount, uint8_t port);  //Uart Send by port. <br>
  void	(*UART_BaudrateSet)(uint32_t buadrate); //Uart init by buadrate. <br>
  
  void MODBUS_xxx_Rx_ISR()  //Modbus UART Rx function. <br>
  
  The default Modbus_Porting.c/.h used DMA to Tx, interrupt Rx and main process data to decrease the CPU loading. 
 #Note
  1. Demo use UART_RTS pin to as RS485 send/receive enable pin, it no need to operate it by software, if new platform don't support it, plseas use UART TC bit to switch the enable.
  2. Demo use PIT module as timer0, it will decrease the value by init, if new platform increase the value, please check the Timer0_Value_Get used place. 
  
针对1个Modubs需要有五个通用函数和一个中断服务函数需要被移植：

  void	(*Timer0_Init)(void);   //init the timer <br>
  uint32_t	(*Timer0_Value_Get)(void);  // get timer value <br>
  uint32_t	(*Timer0_Wait3_5char)(void);  //get modbus 3.5char value. <br>
  void    (*UART_SendData)(uint8_t *pbyData, uint16_t uCount, uint8_t port);  //Uart Send by port. <br>
  void	(*UART_BaudrateSet)(uint32_t buadrate); //Uart init by buadrate. <br>
  
  void MODBUS_xxx_Rx_ISR()  //Modbus UART Rx function. <br>
默认代码使用DMA进行发送，ISR进行接收，主循环进行数据处理以减小CPU的负荷。<br>
#注意
  1. 示例代码使用UART_RTS作为RS485的收发使能，如果新平台不支持此功能，则需要软件根据UART的TC位来处理改引脚。
  2. 示例代码使用PIT作为3.5char的判断，该定时器是自动减定时器，如果需要更新为自动增，用户需要手动修改相关逻辑。
