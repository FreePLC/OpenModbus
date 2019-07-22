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

#ifndef __MODBUS_H__

#define __MODBUS_H__

#include "Modbus_Porting.h"

#include <stdint.h>

/*******************************************************************************
* Definitions
******************************************************************************/

#ifndef FALSE
#define FALSE (0==1)
#endif
#ifndef TRUE
#define TRUE  (1==1)
#endif



#define MODBUS_READ_DIGITAL_OUTPUT					0x01
#define MODBUS_READ_DIGITAL_INPUT					0x02
#define MODBUS_READ_ANOLOG_OUTPUT					0x03
#define MODBUS_READ_ANOLOG_INPUT					0x04
#define MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT			0x05
#define MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT			0x06
#define MODBUS_FORCE_MUL_DIGITAL_OUTPUT				0x0F		
#define MODBUS_FORCE_MUL_ANOLOG_OUTPUT				0x10



#define MODBUS_SEND      0xaa
#define MODBUS_CLOSE     0

#define MODBUS_MASTER_STATUS_END		0
#define MODBUS_MASTER_STATUS_START   	1
#define MODBUS_MASTER_STATUS_SENDING   	2


#define MODBUS_ERROR_OK				0
#define MODBUS_ERROR_MEMORY			1
#define MODBUS_ERROR_OPENFAILD  	2
#define MODBUS_ERROR_REGISTER_ADDRESS_ODD	3
#define MODBUS_ERROR_TIMEOUT_LIMIT	4	
#define MODBUS_ERROR_SLAVEADDR		5
#define MODBUS_ERROR_REGISTER_ADDRESS_OVERFLOW		6
#define MODBUS_ERROR_NUMBER			7
#define MODBUS_ERROR_FUNCTION		8
#define MODBUS_ERROR_TIMEOUT		9
#define MODBUS_ERROR_PROTOCOL		10


#define MODBUS_ACCESS_HEADER(header)	((MODBUSHeader*)(header))

#define MAKEWORD(high,low)		((high*256)+low)


#ifdef MODBUS_MASTER_USED
//yangliang add
typedef struct
{
  uint8_t MasterStatus;				// 1,表示开始
  uint8_t Error;					// 0 OK, 1 Error
  uint16_t Offset;					//用户数据区偏移
  uint16_t Number;					//寄存器个数
  uint16_t RegisterAddress;			//寄存器地址
  uint8_t SlaveAddress;				//从站地址
  uint32_t TimeOut;					//超时时间
  uint32_t SendTime;				//发送时间
  uint8_t Function;					//功能码
  uint8_t *MBUF;
}
MODBUS_PROCESStyp;
#endif


typedef struct tagMODBUSHeader
{
  uint8_t	bSlaveAdd;
  uint8_t	bFuncCode;
  uint8_t	StartAddHigh;
  uint8_t	StartAddLow;/*Motorola Format*/
}MODBUSHeader;



typedef struct tagRxBuffer
{ 
  uint16_t	uCount;
  uint8_t	byData[MODBUS_BUFF_SIZE];  
}RxBuffer;  // Receive

typedef struct tagTxBuffer
{ 
  uint16_t	uCount;
  uint16_t	uSend;
  uint8_t	byData[MODBUS_BUFF_SIZE];  
}TxBuffer;  // Transmit


typedef struct _modbus_port
{
  uint8_t SendFlag;
  uint8_t ReceiveFlag;
  uint8_t ucSize;
  uint8_t g_ucSendLen;
  uint8_t Local_Address;
  uint32_t SendStartTime;
  uint32_t TimeOutLimit;
  RxBuffer  s_RxBuf;
  TxBuffer  s_TxBuf;
  void	(*Timer0_Init)(void);
  uint32_t	(*Timer0_Value_Get)(void);
  uint32_t	(*Timer0_Wait3_5char)(void);
  void    (*UART_SendData)(uint8_t *pbyData, uint16_t uCount, uint8_t port);
  void	(*UART_BaudrateSet)(uint32_t buadrate);
  
}
Modbus_Port;


/*******************************************************************************
* API
******************************************************************************/

uint8_t * Get_MemoryAddr(uint8_t select, uint8_t port);

#ifdef MODBUS_SLAVE_USED
void ModbusSlaveInitPort(uint8_t port);
void ModbusSlaveMainProcess(uint8_t port);
uint8_t ModbusSlaveReceiveInt(uint8_t *pbyData, uint16_t uCount,uint8_t port);
void ModbusSlavePollSend(uint8_t port);
#endif

#ifdef MODBUS_MASTER_USED
int8_t ModbusMasterReceiveInt(uint8_t *pbyData, uint16_t uCount,uint8_t port);
void ModbusMasterMainReceive(uint8_t port);
static void ModbusMasterProcessReceive(uint16_t port);
static void ModbusMasterService(uint16_t port);
void ModbusMasterSendMessage(uint8_t port);
#endif



#endif /*__RS232_H__*/

