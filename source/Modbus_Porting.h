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

#ifndef __MODBUS_PORTING_H__

#define __MODBUS_PORTING_H__

#include "ModbusUserConfig.h"
#include "fsl_pit.h"
#include "fsl_port.h"
#include "fsl_uart.h"


/*******************************************************************************
* Definitions
******************************************************************************/

//UART
#define UART_DMA_USED      1
#define UART_TX_DMA        1
#define UART_RX_DMA		   0


#if UART_DMA_USED
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_uart_edma.h"
#endif


#ifdef MODBUS_MASTER_USED

#define MODBUS_UART1_BAUDRATE  115200
//UART0 for Master
#define UART1_Rx_IRQ        	UART2_RX_TX_IRQn
#define MODBUS_MASTER_Rx1_ISR   UART2_RX_TX_IRQHandler

#define MODBUS_UART1 			UART2
#define MODBUS_UART1_CLKSRC 	kCLOCK_BusClk
#define MODBUS_UART1_CLK_FREQ 	CLOCK_GetFreq(MODBUS_UART1_CLKSRC)

#define UART1_TX_PORT		PORTD
#define UART1_TX_PIN        3
#define UART1_TX_ALT		kPORT_MuxAlt3

#define UART1_RX_PORT		PORTD
#define UART1_RX_PIN        2
#define UART1_RX_ALT		kPORT_MuxAlt3

#define UART1_RTS_PORT		PORTD
#define UART1_RTS_PIN       0
#define UART1_RTS_ALT		kPORT_MuxAlt3

#if UART_DMA_USED
#define UART_DMAMUX_BASEADDR DMAMUX0
#define UART_DMA_BASEADDR DMA0
#define UART_TX1_DMA_REQUEST kDmaRequestMux0UART2Tx
#define UART_RX1_DMA_REQUEST kDmaRequestMux0UART2Rx
#endif


#endif

#ifdef MODBUS_SLAVE_USED

#define MODBUS_UART2_BAUDRATE  115200
//UART1 for Slave
#define UART2_Rx_IRQ        	UART0_RX_TX_IRQn
#define MODBUS_SLAVE_Rx_ISR     UART0_RX_TX_IRQHandler

#define MODBUS_UART2 			UART0
#define MODBUS_UART2_CLKSRC 	kCLOCK_CoreSysClk
#define MODBUS_UART2_CLK_FREQ 	CLOCK_GetFreq(MODBUS_UART2_CLKSRC)

#define UART2_TX_PORT		PORTB
#define UART2_TX_PIN        17
#define UART2_TX_ALT		kPORT_MuxAlt3

#define UART2_RX_PORT		PORTB
#define UART2_RX_PIN        16
#define UART2_RX_ALT		kPORT_MuxAlt3

#define UART2_RTS_PORT		PORTB
#define UART2_RTS_PIN       3
#define UART2_RTS_ALT		kPORT_MuxAlt3

#if UART_DMA_USED
#define UART_DMAMUX_BASEADDR DMAMUX0
#define UART_DMA_BASEADDR DMA0
#define UART_TX2_DMA_REQUEST kDmaRequestMux0UART0Tx
#define UART_RX2_DMA_REQUEST kDmaRequestMux0UART0Rx
#endif


#endif


#if UART_TX_DMA
#define UART1_TX_DMA_CHANNEL 4U
#define UART2_TX_DMA_CHANNEL 6U
#endif

#if UART_RX_DMA
#define UART1_RX_DMA_CHANNEL 5U
#define UART2_RX_DMA_CHANNEL 7U
#endif






//MODBUS APP

#define PLC_INPUT_START_ADDR   0x20000000
#define PLC_INPUT_LEN   	   1024
#define PLC_INPUT_WORD_LEN     (PLC_INPUT_LEN/2)

#define PLC_OUTPUT_START_ADDR  0x20000400
#define PLC_OUTPUT_LEN         1024
#define PLC_OUTPUT_WORD_LEN    (PLC_OUTPUT_LEN/2)

#define PLC_MEMORY_START_ADDR  0x20000800
#define PLC_MEMORY_LEN         256
#define PLC_MEMORY_WORD_LEN    (PLC_MEMORY_LEN/2)




#define MODBUS_I_AREA					0
#define MODBUS_Q_AREA					1
#define MODBUS_M_AREA					2

#define M_AREA_OFFSET					3000		//512
#define MODBUS_BUFF_SIZE 				(254 + 5)

#define MODBUS_I_AREA_BASE 				(uint8_t *)PLC_INPUT_START_ADDR
#define MODBUS_Q_AREA_BASE 				(uint8_t *)PLC_OUTPUT_START_ADDR
#define MODBUS_M_AREA_BASE 				(uint8_t *)PLC_MEMORY_START_ADDR

//TIMER
#ifndef TIMER_MINUS
#define TIMER_MINUS 1
#endif

#define TIMER_SOURCE_CLOCK  CLOCK_GetFreq(kCLOCK_BusClk)

#define NET1_TIMEOUT_LIMIT	MSEC_TO_COUNT(500,TIMER_SOURCE_CLOCK)
#define NET2_TIMEOUT_LIMIT	MSEC_TO_COUNT(100,TIMER_SOURCE_CLOCK)


/*******************************************************************************
* API
******************************************************************************/


#ifdef MODBUS_MASTER_USED
void SendUART_Master(uint8_t *pbyData, uint16_t uCount, uint8_t port);
void ModbusMasterInitPort(uint8_t port);
uint8_t * GetMaster_MemoryAddr(uint8_t select, uint8_t port);
uint32_t Modbus_Master0_Wait3_5char();

#endif


#ifdef MODBUS_SLAVE_USED
void ModbusSlaveInitPort(uint8_t port);
void SendUART_Slave(uint8_t *pbyData, uint16_t uCount, uint8_t port);
uint8_t * GetSlave_MemoryAddr(uint8_t select, uint8_t port);
uint32_t Modbus_Slave0_Wait3_5char();
#endif



void PIT0_Configuration(void);

void UART1_Configuration(uint32_t buadrate);
void UART2_Configuration(uint32_t buadrate);
uint32_t PIT0_Value_Get();
void User_Memcpy(uint8_t *dst, uint8_t *src, uint32_t len);




#endif

