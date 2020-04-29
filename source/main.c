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

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_pit.h"
#include "fsl_wdog.h"

#include "pin_mux.h"
#include "clock_config.h"

#include "Modbus.h"
#include "ModbusMasterApp.h"
#include "ModbusSlaveApp.h"
#include "ModbusUserConfig.h"


/*******************************************************************************
* Definitions
******************************************************************************/

#define WDOG_WCT_INSTRUCITON_COUNT (256U)


/*******************************************************************************
* Prototypes
******************************************************************************/
static void WaitWctClose(WDOG_Type *base);



/*******************************************************************************
* Variables
******************************************************************************/

/*******************************************************************************
* Code
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



/*!
* @brief Main function
*/
int main(void)
{
  BOARD_InitPins();
  BOARD_BootClockRUN();
  //WDOG_Configuration();
  
#ifdef MODBUS_SLAVE_USED
  Slave0_InitPort(SLAVE_PORT0, MODBUS_SLAVE0_ADDRESS);
#endif
  
#ifdef MODBUS_MASTER_USED
  MB_Init();
  Master0_InitPort(MASTER_PORT0);
#endif
  
  while (1)
  {
#ifdef MODBUS_SLAVE_USED
    ModbusSlaveMainProcess(SLAVE_PORT0);
    ModbusSlavePollSend(SLAVE_PORT0);
    ModbusNet1SlaveAPP();
#endif
    
#ifdef MODBUS_MASTER_USED
    ModbusNet1MasterAPP();
    ModbusMasterSendMessage(MASTER_PORT0);
    ModbusMasterMainReceive(MASTER_PORT0);
#endif
    
    //WDOG_Refresh(WDOG);
  }
}


