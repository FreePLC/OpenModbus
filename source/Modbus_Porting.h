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
#include "fsl_gpio.h"
#include "fsl_edma.h"
#include "fsl_dmamux.h"
#include "fsl_uart_edma.h"



/*******************************************************************************
* Definitions
******************************************************************************/


#ifdef MODBUS_MASTER_USED

#define MASTER0_DMA_USED      1
#define MASTER0_TX_DMA        1
#define MASTER0_RX_DMA        0


#define MASTER0_RS485_GPIO	       0

#define MASTER0_UART_BAUDRATE  115200
//UART0 for Master
#define MASTER0_UART_Rx_IRQ        	UART2_RX_TX_IRQn
#define MASTER0_Rx_ISR   UART2_RX_TX_IRQHandler

#define MASTER0_UART 			UART2
#define MASTER0_UART_CLKSRC 	kCLOCK_BusClk
#define MASTER0_UART_CLK_FREQ 	CLOCK_GetFreq(MASTER0_UART_CLKSRC)

#define MASTER0_TX_PORT		PORTD
#define MASTER0_TX_PIN        3
#define MASTER0_TX_ALT		kPORT_MuxAlt3

#define MASTER0_RX_PORT		PORTD
#define MASTER0_RX_PIN        2
#define MASTER0_RX_ALT		kPORT_MuxAlt3

#define MASTER0_RTS_PORT		PORTD
#define MASTER0_RTS_GPIO		GPIOD
#define MASTER0_RTS_PIN       0
#define MASTER0_RTS_ALT		kPORT_MuxAlt3

#if MASTER0_DMA_USED
#define UART_DMAMUX_BASEADDR DMAMUX0
#define UART_DMA_BASEADDR DMA0
#define MASTER0_TX_DMA_REQUEST kDmaRequestMux0UART2Tx
#define MASTER0_RX_DMA_REQUEST kDmaRequestMux0UART2Rx
#define MASTER0_TX_DMA_CHANNEL 4U
#define MASTER0_RX_DMA_CHANNEL 5U
#endif


#endif

#ifdef MODBUS_SLAVE_USED

#define SLAVE0_DMA_USED      1
#define SLAVE0_TX_DMA        1
#define SLAVE0_RX_DMA        0


#define SLAVE0_UART_BAUDRATE  115200
//UART1 for Slave
#define SLAVE0_UART_Rx_IRQ        	UART0_RX_TX_IRQn
#define SLAVE0_Rx_ISR     UART0_RX_TX_IRQHandler

#define SLAVE0_UART 			UART0
#define SLAVE0_UART_CLKSRC 	kCLOCK_CoreSysClk
#define SLAVE0_UART_CLK_FREQ 	CLOCK_GetFreq(SLAVE0_UART_CLKSRC)

#define SLAVE0_TX_PORT		PORTB
#define SLAVE0_TX_PIN        17
#define SLAVE0_TX_ALT		kPORT_MuxAlt3

#define SLAVE0_RX_PORT		PORTB
#define SLAVE0_RX_PIN        16
#define SLAVE0_RX_ALT		kPORT_MuxAlt3

#define SLAVE0_RTS_PORT		PORTB
#define SLAVE0_RTS_GPIO		GPIOB
#define SLAVE0_RTS_PIN       3
#define SLAVE0_RTS_ALT		kPORT_MuxAlt3

#if SLAVE0_DMA_USED
#define UART_DMAMUX_BASEADDR DMAMUX0
#define UART_DMA_BASEADDR DMA0
#define SLAVE0_TX_DMA_REQUEST kDmaRequestMux0UART0Tx
#define SLAVE0_RX_DMA_REQUEST kDmaRequestMux0UART0Rx
#define SLAVE0_TX_DMA_CHANNEL 6U
#define SLAVE0_RX_DMA_CHANNEL 7U
#endif


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
void Master0_SendUart(uint8_t *pbyData, uint16_t uCount, uint8_t port);
void Master0_InitPort(uint8_t port);
uint8_t * GetMaster_MemoryAddr(uint8_t select, uint8_t port);
uint32_t Master0_Wait3_5char();

#endif


#ifdef MODBUS_SLAVE_USED
void Slave0_InitPort(uint8_t port, uint8_t slave_addr);
void Slave0_SendUart(uint8_t *pbyData, uint16_t uCount, uint8_t port);
uint8_t * GetSlave_MemoryAddr(uint8_t select, uint8_t port);
uint32_t Slave0_Wait3_5char();
#endif



void PIT0_Configuration(void);

void Master0_UART_Configuration(uint32_t buadrate);
void Slave0_UART_Configuration(uint32_t buadrate);
uint32_t PIT0_Value_Get();
void User_Memcpy(uint8_t *dst, uint8_t *src, uint32_t len);




#endif

