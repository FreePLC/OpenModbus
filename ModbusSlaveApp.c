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

#include "ModbusSlaveApp.h"

/*******************************************************************************
* Definitions
******************************************************************************/




/*******************************************************************************
* Prototypes
******************************************************************************/



/*******************************************************************************
* Variables
******************************************************************************/



//UART

AT_NONCACHEABLE_SECTION_ALIGN(uint8_t g_txBuffer[MODBUS_BUFF_SIZE], 64);
AT_NONCACHEABLE_SECTION_ALIGN(uint8_t g_rxBuffer[MODBUS_BUFF_SIZE], 64);


//APP
uint8_t g_ucMBUF[MODBUS_BUFF_SIZE];

static void WaitWctClose(WDOG_Type *base);


/*******************************************************************************
* Code
******************************************************************************/



/*******************************************************************************
* WDT
******************************************************************************/

static void WaitWctClose(WDOG_Type *base)
{
  /* Accessing register by bus clock */
  for (uint32_t i = 0; i < WDOG_WCT_INSTRUCITON_COUNT; i++)
  {
    (void)base->RSTCNT;
  }
}


void WDOG_Configuration()
{
  wdog_config_t config;
  
  WDOG_GetDefaultConfig(&config);
  //config.timeoutValue = 0x7ffU;
  
  WDOG_Init(WDOG, &config);
  WaitWctClose(WDOG);
  
}




