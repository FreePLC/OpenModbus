
/*
* The Clear BSD License
* Copyright (c) 2019 Liang.Yang
* Copyright 2019-2019 Liang.Yang <WeChat:kala4tgo><Email:17389711@qq.com>
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted (subject to the limitations in the disclaimer below) provided
* that the following conditions are met:
*
* o Redistributions of source code must retain the above copyright notice, this list
*   of conditions and the following disclaimer.
*
* o Redistributions in binary form must reproduce the above copyright notice, this
*   list of conditions and the following disclaimer in the documentation and/or
*   other materials provided with the distribution.
*
* o Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from this
*   software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "Modbus.h"



/*******************************************************************************
* Definitions
******************************************************************************/


/*******************************************************************************
* Prototypes
******************************************************************************/



/*******************************************************************************
* Variables
******************************************************************************/



//Need to Change
#ifdef MODBUS_MASTER_USED
UART_Type *const s_MasteruartBase[MODBUS_MASTER_NUMBER] = {MODBUS_UART1};
uint32_t g_u32ModbusBuadrate[MODBUS_MASTER_NUMBER] = {MODBUS_UART1_BAUDRATE};
#endif


//TX
#if UART_TX_DMA
volatile bool g_Transfer_UART1_Tx_Done = false; /* DMA transfer completion flag. */
edma_handle_t g_EDMA_UART1_Tx_Handle; /* Edma handler. */
edma_transfer_config_t g_UART1_Tx_transferConfig;

volatile bool g_Transfer_UART2_Tx_Done = false; /* DMA transfer completion flag. */
edma_handle_t g_EDMA_UART2_Tx_Handle; /* Edma handler. */
edma_transfer_config_t g_UART2_Tx_transferConfig;

#endif

//RX
#if UART_RX_DMA
volatile bool g_Transfer_UART1_Rx_Done = false; /* DMA transfer completion flag. */
edma_handle_t g_EDMA_UART1_Rx_Handle; /* Edma handler. */
edma_transfer_config_t g_UART1_Rx_transferConfig;

volatile bool g_Transfer_UART2_Rx_Done = false; /* DMA transfer completion flag. */
edma_handle_t g_EDMA_UART2_Rx_Handle; /* Edma handler. */
edma_transfer_config_t g_UART2_Rx_transferConfig;

#endif

#ifdef MODBUS_SLAVE_USED
UART_Type *const s_SlaveuartBase[MODBUS_SLAVE_NUMBER] = {MODBUS_UART2};
extern Modbus_Port g_ModbusSlavePort[];
extern uint8_t g_ucMBUF[MODBUS_BUFF_SIZE];

#endif

#ifdef MODBUS_MASTER_USED
extern MODBUS_PROCESStyp g_MProcess[MODBUS_MASTER_NUMBER];
extern Modbus_Port g_ModbusMasterPort[MODBUS_MASTER_NUMBER];
extern uint8_t g_u8MasterBuf[MODBUS_MASTER_NUMBER][PLC_MEMORY_LEN];
#endif



/*******************************************************************************
* Code
******************************************************************************/


uint32_t PIT0_Value_Get()
{
  return PIT->CHANNEL[0].CVAL;
}


void PIT0_Configuration(void)
{
  /* Structure of initialize PIT */
  pit_config_t pitConfig;
  /*
  * pitConfig.enableRunInDebug = false;
  */
  PIT_Init(PIT, &pitConfig);
  PIT_GetDefaultConfig(&pitConfig);
  PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, 0xFFFFFFFF);
  PIT_StartTimer(PIT, kPIT_Chnl_0);
  
  
  
}


#ifdef MODBUS_SLAVE_USED

void UART2_Configuration(uint32_t buadrate)
{
  uart_config_t uartConfig;
  //edma_config_t config;
  //uart_transfer_t sendXfer;
#if UART_DMA_USED
  edma_config_t userConfig;
#endif
  port_pin_config_t pin_config;

  CLOCK_EnableClock(kCLOCK_PortB);							 /* Port B Clock Gate Control: Clock enabled */
  
  UART_GetDefaultConfig(&uartConfig);
  uartConfig.baudRate_Bps = buadrate;
  uartConfig.enableTx = true;
  uartConfig.enableRx = true;
  uartConfig.enableRxRTS = true;
  
  pin_config.pullSelect = kPORT_PullUp;
  //pin_config.slewRate = kPORT_SlowSlewRate;
  pin_config.driveStrength = kPORT_LowDriveStrength;
  pin_config.lockRegister = kPORT_UnlockRegister;
  //pin_config.openDrainEnable = kPORT_OpenDrainDisable;
  pin_config.mux = UART2_TX_ALT;
  pin_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
  
  PORT_SetPinConfig(UART2_TX_PORT, UART2_TX_PIN, &pin_config);
  pin_config.mux = UART2_RX_ALT;
  PORT_SetPinConfig(UART2_RX_PORT, UART2_RX_PIN, &pin_config);
#if UART_RX_DMA
  uartConfig.rxFifoWatermark = 1;
#endif
  
  UART_Init(MODBUS_UART2, &uartConfig, MODBUS_UART2_CLK_FREQ);
  
  
#if UART_TX_DMA
  UART_EnableTxDMA(MODBUS_UART2, true);
#endif
  
#if UART_RX_DMA
  /* Enable UART TX EDMA. */
  UART_EnableRxDMA(MODBUS_UART2, true);
#else
  UART_EnableInterrupts(MODBUS_UART2, kUART_RxDataRegFullInterruptEnable 
  | kUART_RxOverrunInterruptEnable | kUART_NoiseErrorInterruptEnable | kUART_FramingErrorInterruptEnable
  | kUART_ParityErrorInterruptEnable);
  EnableIRQ(UART2_Rx_IRQ);
#endif
  
#if UART_DMA_USED
  /* Set channel for UART */
  DMAMUX_Init(UART_DMAMUX_BASEADDR);
  
#if UART_TX_DMA
  DMAMUX_SetSource(UART_DMAMUX_BASEADDR, UART2_TX_DMA_CHANNEL, UART_TX2_DMA_REQUEST);
  DMAMUX_EnableChannel(UART_DMAMUX_BASEADDR, UART2_TX_DMA_CHANNEL);
#endif
  
#if UART_RX_DMA
  /* Set channel for UART */
  DMAMUX_SetSource(UART_DMAMUX_BASEADDR, UART2_RX_DMA_CHANNEL, UART_RX2_DMA_REQUEST);
  DMAMUX_EnableChannel(UART_DMAMUX_BASEADDR, UART2_RX_DMA_CHANNEL);
#endif
  
#endif
  
#if UART_DMA_USED
  
  //UART2_TX
#if UART_TX_DMA
  
  EDMA_GetDefaultConfig(&userConfig);
  EDMA_Init(UART_DMA_BASEADDR, &userConfig);
  EDMA_CreateHandle(&g_EDMA_UART2_Tx_Handle, UART_DMA_BASEADDR, UART2_TX_DMA_CHANNEL);
  
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
  /* Enable async DMA request. */
  EDMA_EnableAsyncRequest(UART_DMA_BASEADDR, UART2_TX_DMA_CHANNEL, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
  
#endif
  
  //UART2_RX
#if UART_RX_DMA
  EDMA_GetDefaultConfig(&userConfig);
  EDMA_Init(UART_DMA_BASEADDR, &userConfig);
  EDMA_CreateHandle(&g_EDMA_UART2_Rx_Handle, UART_DMA_BASEADDR, UART2_RX_DMA_CHANNEL);
  
  EDMA_PrepareTransfer(&g_UART2_Rx_transferConfig, (void *)UART_GetDataRegisterAddress(MODBUS_UART2), sizeof(uint8_t),
                       (void *)&g_ModbusSlavePort[0].s_RxBuf.byData, sizeof(uint8_t), sizeof(uint8_t),
                       sizeof(g_ModbusSlavePort[0].s_RxBuf.byData), kEDMA_PeripheralToMemory);
  
  
  EDMA_SubmitTransfer(&g_EDMA_UART2_Rx_Handle, &g_UART2_Rx_transferConfig);
  
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
  /* Enable async DMA request. */
  EDMA_EnableAsyncRequest(UART_DMA_BASEADDR, UART2_RX_DMA_CHANNEL, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
  /* Enable transfer. */
  EDMA_StartTransfer(&g_EDMA_UART2_Rx_Handle);
  
#endif
  
  
#endif
  
  
}



uint32_t Modbus_Slave0_Wait3_5char()
{
  return USEC_TO_COUNT(35*1000*1000/MODBUS_UART2_BAUDRATE,TIMER_SOURCE_CLOCK);
}


void SendUART_Slave(uint8_t *pbyData, uint16_t uCount, uint8_t port)
{
  
  if(pbyData != (unsigned char *)&g_ModbusSlavePort[0].s_TxBuf.byData)
  {
    if(uCount < sizeof(g_ModbusSlavePort[0].s_TxBuf.byData))
    {
      User_Memcpy((void *)&g_ModbusSlavePort[0].s_TxBuf.byData, pbyData, uCount);
    }
    else
    {
      User_Memcpy((void *)&g_ModbusSlavePort[0].s_TxBuf.byData, pbyData, sizeof(g_ModbusSlavePort[0].s_TxBuf.byData));
    }
    
  }
  else if(uCount > sizeof(g_ModbusSlavePort[0].s_TxBuf.byData))
  {
    uCount = sizeof(g_ModbusSlavePort[0].s_TxBuf.byData);
  }
  
  
#if UART_TX_DMA
  
  EDMA_PrepareTransfer(&g_UART2_Tx_transferConfig, (void*)g_ModbusSlavePort[port].s_TxBuf.byData, sizeof(uint8_t),
                       (void *)UART_GetDataRegisterAddress(s_SlaveuartBase[port]), sizeof(uint8_t), sizeof(uint8_t),
                       uCount, kEDMA_MemoryToPeripheral);
  EDMA_SubmitTransfer(&g_EDMA_UART2_Tx_Handle, &g_UART2_Tx_transferConfig);
  EDMA_StartTransfer(&g_EDMA_UART2_Tx_Handle);
#else
  
  UART_WriteBlocking(s_SlaveuartBase[port], (uint8_t *)g_ModbusSlavePort[port].s_TxBuf.byData, uCount);
#endif
}




void ModbusSlaveInitPort(uint8_t port)
{
  g_ModbusSlavePort[port].Local_Address = MODBUS_SLAVE0_ADDRESS;
  g_ModbusSlavePort[port].g_ucSendLen = 0;
  g_ModbusSlavePort[port].ReceiveFlag = 0;
  g_ModbusSlavePort[port].SendFlag = 0;
  g_ModbusSlavePort[port].SendStartTime = 0;
  g_ModbusSlavePort[port].ucSize = 0;
  g_ModbusSlavePort[port].Timer0_Init = PIT0_Configuration;
  g_ModbusSlavePort[port].Timer0_Value_Get = PIT0_Value_Get;
  g_ModbusSlavePort[port].UART_SendData = SendUART_Slave;
  g_ModbusSlavePort[port].Timer0_Wait3_5char = Modbus_Slave0_Wait3_5char;
  g_ModbusSlavePort[port].UART_BaudrateSet = UART2_Configuration;
  
  g_ModbusSlavePort[port].UART_BaudrateSet(MODBUS_UART2_BAUDRATE);
  g_ModbusSlavePort[port].Timer0_Init();
}
void MODBUS_SLAVE_Rx_ISR()
{
  uint8_t data = 0, len = 0, i = 0;
  static uint32_t last_time = 0;
  uint32_t current_time = 0;
  
  if((kUART_RxOverrunFlag|kUART_NoiseErrorFlag|kUART_FramingErrorFlag|kUART_ParityErrorFlag) & UART_GetStatusFlags(MODBUS_UART2))
  {
    data = UART_ReadByte(MODBUS_UART2); 
    MODBUS_UART2->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);
    return;
  }
  
  /* If new data arrived. */
  if ((kUART_RxDataRegFullFlag) & UART_GetStatusFlags(MODBUS_UART2))
  {
    current_time =  g_ModbusSlavePort[SLAVE_PORT0].Timer0_Value_Get();
    #if TIMER_MINUS == 1
    if ((last_time - current_time) > g_ModbusSlavePort[SLAVE_PORT0].Timer0_Wait3_5char())
	#else
	if ((current_time - last_time) > g_ModbusSlavePort[SLAVE_PORT0].Timer0_Wait3_5char())
	#endif
    {
      g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount = 0;
    }
    
    len = MODBUS_UART2->RCFIFO;
    
    for(i = 0; i< len; i++)
    {
      data = UART_ReadByte(MODBUS_UART2);    
      g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.byData[g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount++] = data;    
    }
    
    if(g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount >= 6)
    {
      if (ModbusSlaveReceiveInt((unsigned char*)g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.byData, g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount, SLAVE_PORT0))
      {
        ;
      }
      
    }
    
    if(g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount >= MODBUS_BUFF_SIZE)
    {
      g_ModbusSlavePort[SLAVE_PORT0].s_RxBuf.uCount = 0;
    }
    
    last_time = g_ModbusSlavePort[SLAVE_PORT0].Timer0_Value_Get();
  }
  
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
  __DSB();
#endif
  
}

uint8_t * GetSlave_MemoryAddr(uint8_t select, uint8_t port)
{
#if 0
  if(select == MODBUS_I_AREA)
  {
    return MODBUS_I_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_INPUT);
  }
  else if(select == MODBUS_Q_AREA)
  {
    return MODBUS_Q_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_OUTPUT);
  }
  else
  {
    return MODBUS_M_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_MEMORY);
  }
#endif
  select = select;
  
  return &g_ucMBUF[0];
}


#endif



#ifdef MODBUS_MASTER_USED

void UART1_Configuration(uint32_t buadrate)
{
  uart_config_t uartConfig;
  //edma_config_t config;
  //uart_transfer_t sendXfer;
#if UART_DMA_USED
  edma_config_t userConfig;
#endif
  port_pin_config_t pin_config;

  CLOCK_EnableClock(kCLOCK_PortD);							 /* Port B Clock Gate Control: Clock enabled */

  
  UART_GetDefaultConfig(&uartConfig);
  uartConfig.baudRate_Bps = buadrate;
  uartConfig.enableTx = true;
  uartConfig.enableRx = true;
  uartConfig.enableRxRTS = true;
  
  pin_config.pullSelect = kPORT_PullUp;
  //pin_config.slewRate = kPORT_SlowSlewRate;
  pin_config.driveStrength = kPORT_LowDriveStrength;
  pin_config.lockRegister = kPORT_UnlockRegister;
  //pin_config.openDrainEnable = kPORT_OpenDrainDisable;
  pin_config.mux = UART1_TX_ALT;
  pin_config.passiveFilterEnable = kPORT_PassiveFilterDisable;
  
  PORT_SetPinConfig(UART1_TX_PORT, UART1_TX_PIN, &pin_config);
  pin_config.mux = UART1_RX_ALT;
  PORT_SetPinConfig(UART1_RX_PORT, UART1_RX_PIN, &pin_config);
#if UART_RX_DMA
  uartConfig.rxFifoWatermark = 1;
#endif
  
  UART_Init(MODBUS_UART1, &uartConfig, MODBUS_UART1_CLK_FREQ);
  
#if UART_TX_DMA
  UART_EnableTxDMA(MODBUS_UART1, true);
#endif
  
#if UART_RX_DMA
  /* Enable UART TX EDMA. */
  UART_EnableRxDMA(MODBUS_UART1, true);
#else
  UART_EnableInterrupts(MODBUS_UART1, kUART_RxDataRegFullInterruptEnable 
  | kUART_RxOverrunInterruptEnable | kUART_NoiseErrorInterruptEnable | kUART_FramingErrorInterruptEnable
  | kUART_ParityErrorInterruptEnable);
  EnableIRQ(UART1_Rx_IRQ);
#endif
  
#if UART_DMA_USED
  /* Set channel for UART */
  DMAMUX_Init(UART_DMAMUX_BASEADDR);
#if UART_TX_DMA
  DMAMUX_SetSource(UART_DMAMUX_BASEADDR, UART1_TX_DMA_CHANNEL, UART_TX1_DMA_REQUEST);
  DMAMUX_EnableChannel(UART_DMAMUX_BASEADDR, UART1_TX_DMA_CHANNEL);
#endif
  
#if UART_RX_DMA
  /* Set channel for UART */
  DMAMUX_SetSource(UART_DMAMUX_BASEADDR, UART1_RX_DMA_CHANNEL, UART_RX1_DMA_REQUEST);
  DMAMUX_EnableChannel(UART_DMAMUX_BASEADDR, UART1_RX_DMA_CHANNEL);
#endif
  
#endif
  
#if UART_DMA_USED
  //UART1_TX
#if UART_TX_DMA
  
  EDMA_GetDefaultConfig(&userConfig);
  EDMA_Init(UART_DMA_BASEADDR, &userConfig);
  EDMA_CreateHandle(&g_EDMA_UART1_Tx_Handle, UART_DMA_BASEADDR, UART1_TX_DMA_CHANNEL);
  
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
  /* Enable async DMA request. */
  EDMA_EnableAsyncRequest(UART_DMA_BASEADDR, UART1_TX_DMA_CHANNEL, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
  
#endif
  
  //UART1_RX
#if UART_RX_DMA
  EDMA_GetDefaultConfig(&userConfig);
  EDMA_Init(UART_DMA_BASEADDR, &userConfig);
  EDMA_CreateHandle(&g_EDMA_UART1_Rx_Handle, UART_DMA_BASEADDR, UART1_RX_DMA_CHANNEL);
  
  EDMA_PrepareTransfer(&g_UART1_Rx_transferConfig, (void *)UART_GetDataRegisterAddress(MODBUS_UART1), sizeof(uint8_t),
                       (void *)&g_ModbusMasterPort[0].s_RxBuf.byData, sizeof(uint8_t), sizeof(uint8_t),
                       sizeof(g_ModbusMasterPort[0].s_RxBuf.byData), kEDMA_PeripheralToMemory);
  
  
  EDMA_SubmitTransfer(&g_EDMA_UART1_Rx_Handle, &g_UART1_Rx_transferConfig);
  
#if defined(FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT) && FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT
  /* Enable async DMA request. */
  EDMA_EnableAsyncRequest(UART_DMA_BASEADDR, UART1_RX_DMA_CHANNEL, true);
#endif /* FSL_FEATURE_EDMA_ASYNCHRO_REQUEST_CHANNEL_COUNT */
  /* Enable transfer. */
  EDMA_StartTransfer(&g_EDMA_UART1_Rx_Handle);
  
#endif
  
  
#endif
  
  
}


uint32_t Modbus_Master0_Wait3_5char()
{
  return USEC_TO_COUNT(35*1000*1000/MODBUS_UART1_BAUDRATE,TIMER_SOURCE_CLOCK);
}

void SendUART_Master(uint8_t *pbyData, uint16_t uCount, uint8_t port)
{
  //unsigned char temp = 0;
  
  if(pbyData != (unsigned char *)&g_ModbusMasterPort[port].s_TxBuf.byData)
  {
    if(uCount < sizeof(g_ModbusMasterPort[port].s_TxBuf.byData))
    {
      User_Memcpy((void *)&g_ModbusMasterPort[port].s_TxBuf.byData, pbyData, uCount);
    }
    else
    {
      User_Memcpy((void *)&g_ModbusMasterPort[port].s_TxBuf.byData, pbyData, sizeof(g_ModbusMasterPort[port].s_TxBuf.byData));
    }
    
  }
  else if(uCount > sizeof(g_ModbusMasterPort[port].s_TxBuf.byData))
  {
    uCount = sizeof(g_ModbusMasterPort[port].s_TxBuf.byData);
  }
  
#if UART_TX_DMA
  
  EDMA_PrepareTransfer(&g_UART1_Tx_transferConfig, (void*)g_ModbusMasterPort[port].s_TxBuf.byData, sizeof(uint8_t),
                       (void *)UART_GetDataRegisterAddress(s_MasteruartBase[port]), sizeof(uint8_t), sizeof(uint8_t),
                       uCount, kEDMA_MemoryToPeripheral);
  EDMA_SubmitTransfer(&g_EDMA_UART1_Tx_Handle, &g_UART1_Tx_transferConfig);
  EDMA_StartTransfer(&g_EDMA_UART1_Tx_Handle);
#else
  
  UART_WriteBlocking(s_MasteruartBase[port], (uint8_t *)g_ModbusMasterPort[port].s_TxBuf.byData, uCount);
#endif
  
  
}


void ModbusMasterInitPort(uint8_t port)
{  
  g_ModbusMasterPort[port].Local_Address = 0;
  g_ModbusMasterPort[port].g_ucSendLen = 0;
  g_ModbusMasterPort[port].ReceiveFlag = 0;
  g_ModbusMasterPort[port].SendFlag = 0;
  g_ModbusMasterPort[port].SendStartTime = 0;		
  g_ModbusMasterPort[port].ucSize = 0;
  g_ModbusMasterPort[port].Timer0_Init = PIT0_Configuration;
  g_ModbusMasterPort[port].Timer0_Value_Get = PIT0_Value_Get;
  g_ModbusMasterPort[port].UART_SendData = SendUART_Master;
  g_ModbusMasterPort[port].Timer0_Wait3_5char = Modbus_Master0_Wait3_5char;
  

  g_ModbusMasterPort[port].UART_BaudrateSet = UART1_Configuration;
  g_ModbusMasterPort[port].TimeOutLimit = NET1_TIMEOUT_LIMIT;			
  
  g_ModbusMasterPort[port].UART_BaudrateSet(g_u32ModbusBuadrate[port]);
  g_ModbusMasterPort[port].Timer0_Init(); 
  
}


void MODBUS_MASTER_Rx1_ISR()
{
  uint8_t data = 0, len = 0, i = 0;
  static uint32_t last_time = 0;
  uint32_t current_time = 0;
  
  if((kUART_RxOverrunFlag|kUART_NoiseErrorFlag|kUART_FramingErrorFlag|kUART_ParityErrorFlag) & UART_GetStatusFlags(MODBUS_UART1))
  {
    data = UART_ReadByte(MODBUS_UART1); 
    MODBUS_UART1->CFIFO |= (UART_CFIFO_TXFLUSH_MASK | UART_CFIFO_RXFLUSH_MASK);
    
    return;
  }
  
  /* If new data arrived. */
  if ((kUART_RxDataRegFullFlag) & UART_GetStatusFlags(MODBUS_UART1))
  {
    current_time =  g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
    #if TIMER_MINUS == 1
    if ((last_time - current_time) > g_ModbusMasterPort[MASTER_PORT0].Timer0_Wait3_5char())
	#else
	if ((current_time - last_time) > g_ModbusMasterPort[MASTER_PORT0].Timer0_Wait3_5char())
	#endif
    {
      g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.uCount = 0;
    }
    
    len = MODBUS_UART1->RCFIFO;
    
    for(i = 0; i< len; i++)
    {
      data = UART_ReadByte(MODBUS_UART1);  
      g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.byData[g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.uCount++] = data;    
    }
    
    
    if(ModbusMasterReceiveInt((unsigned char*)g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.byData, g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.uCount, MASTER_PORT0))
    {
      ;
    }
    
    if(g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.uCount >= MODBUS_BUFF_SIZE)
    {
      g_ModbusMasterPort[MASTER_PORT0].s_RxBuf.uCount = 0;
    }
    
    last_time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
  }
  
  /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
  exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
  __DSB();
#endif
  
}


uint8_t * GetMaster_MemoryAddr(uint8_t select, uint8_t port)
{
#if 0
  if(select == MODBUS_I_AREA)
  {
    return MODBUS_I_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_INPUT);
  }
  else if(select == MODBUS_Q_AREA)
  {
    return MODBUS_Q_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_OUTPUT);
  }
  else
  {
    return MODBUS_M_AREA_BASE;		//CtrlGetSegmentAddress(DATAIDX_MEMORY);
  }
#endif
  select = select;
  
  return &g_u8MasterBuf[port][0];
}



#endif


void User_Memcpy(uint8_t *dst, uint8_t *src, uint32_t len)
{
  memcpy(dst, src, len);
}


