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
#include "ModbusMasterApp.h"

#ifdef MODBUS_MASTER_USED

/*******************************************************************************
* Definitions
******************************************************************************/


/*******************************************************************************
* Prototypes
******************************************************************************/



/*******************************************************************************
* Variables
******************************************************************************/




MODBUS_TASK_STRUC_T g_MBNet1_task;

//Modbus发送/接收数据Buffer
uint8_t g_u8MasterBuf[MODBUS_MASTER_NUMBER][PLC_MEMORY_LEN] = {0};
uint16_t g_u8UserData[2] = {0x1234, 0x5678};
extern MODBUS_PROCESStyp g_MProcess[MODBUS_MASTER_NUMBER];
extern Modbus_Port g_ModbusMasterPort[MODBUS_MASTER_NUMBER];



/*******************************************************************************
* Code
******************************************************************************/


void MB_Init()
{
  g_MBNet1_task.delay = 0;
  g_MBNet1_task.status = NET1_DEVICE1_WRITE;
  g_MBNet1_task.time = 0;
  
  g_MProcess[0].MBUF = &g_u8MasterBuf[0][0];
  
}

/*************************************************************
//用于定义子设备通讯参数，主循环调用
*************************************************************/


void ModbusNet1MasterAPP()
{
  switch(g_MBNet1_task.status)
  {
  case NET1_DEVICE1_WRITE:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {
        g_MProcess[MASTER_PORT0].Function = MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT;
        g_MProcess[MASTER_PORT0].SlaveAddress = DEVICE1_SLAVE_ADDR;
        g_MProcess[MASTER_PORT0].Offset = DEVICE1_WRITE_OFFSET;
        g_MProcess[MASTER_PORT0].Number= 1;
        g_MProcess[MASTER_PORT0].RegisterAddress = DEVICE1_WRITE_REGISTER;
        g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
        g_MProcess[MASTER_PORT0].Error = MODBUS_ERROR_OK;
        //设置需要写入Device1的数据
        g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset] = g_u8UserData[0]&0xFF;
        g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset + 1] = (g_u8UserData[0]>>8)&0xFF;
        
        //切换状态
        g_MBNet1_task.pending_status = NET1_DEVICE1_WRITE;
        g_MBNet1_task.next_status = NET1_DEVICE1_READ;
        g_MBNet1_task.delay = MSEC_TO_COUNT(NET1_DELAY_MS,TIMER_SOURCE_CLOCK);
        g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
        g_MBNet1_task.status = NET1_WAIT;
        
      }
      
    }
    break;
  case NET1_DEVICE2_WRITE:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {
        g_MProcess[MASTER_PORT0].Function = MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT;
        g_MProcess[MASTER_PORT0].SlaveAddress = DEVICE2_SLAVE_ADDR;
        g_MProcess[MASTER_PORT0].Offset = DEVICE2_WRITE_OFFSET;
        g_MProcess[MASTER_PORT0].Number= 1;
        g_MProcess[MASTER_PORT0].RegisterAddress = DEVICE2_WRITE_REGISTER;
        g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
        g_MProcess[MASTER_PORT0].Error = MODBUS_ERROR_OK;
        
        g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset] = g_u8UserData[1]&0xFF;
        g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset + 1] = (g_u8UserData[1]>>8)&0xFF;
        
        //切换状态
        g_MBNet1_task.pending_status = NET1_DEVICE2_WRITE;
        g_MBNet1_task.next_status = NET1_DEVICE2_READ;
        g_MBNet1_task.delay = MSEC_TO_COUNT(NET1_DELAY_MS,TIMER_SOURCE_CLOCK);
        g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
        g_MBNet1_task.status = NET1_WAIT;
        
      }
      
    }
    break; 
    
  case NET1_DEVICE1_READ:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {
        g_MProcess[MASTER_PORT0].Function = MODBUS_READ_ANOLOG_OUTPUT;
        g_MProcess[MASTER_PORT0].SlaveAddress = DEVICE1_SLAVE_ADDR;
        g_MProcess[MASTER_PORT0].Offset = DEVICE1_READ_OFFSET;
        g_MProcess[MASTER_PORT0].Number= 1;
        g_MProcess[MASTER_PORT0].RegisterAddress = DEVICE1_READ_REGISTER;
        g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
        
        //切换状态
        g_MBNet1_task.pending_status = NET1_DEVICE1_READ;
        g_MBNet1_task.next_status = NET1_DEVICE2_WRITE;
        g_MBNet1_task.delay = MSEC_TO_COUNT(NET1_DELAY_MS,TIMER_SOURCE_CLOCK);
        g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
        g_MBNet1_task.status = NET1_WAIT;
        
      }
      
    }
    break;
  case NET1_DEVICE2_READ:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {
        g_MProcess[MASTER_PORT0].Function = MODBUS_READ_ANOLOG_OUTPUT;
        g_MProcess[MASTER_PORT0].SlaveAddress = DEVICE2_SLAVE_ADDR;
        g_MProcess[MASTER_PORT0].Offset = DEVICE2_READ_OFFSET;
        g_MProcess[MASTER_PORT0].Number= 1;
        g_MProcess[MASTER_PORT0].RegisterAddress = DEVICE2_READ_REGISTER;
        g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
        
        //切换状态
        g_MBNet1_task.pending_status = NET1_DEVICE2_READ;
        g_MBNet1_task.next_status = NET1_DEVICE1_WRITE;
        g_MBNet1_task.delay = MSEC_TO_COUNT(NET1_DELAY_MS,TIMER_SOURCE_CLOCK);
        g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
        g_MBNet1_task.status = NET1_WAIT;
        
      }
      
    }
    break;
  case NET1_STOP:
    {
      ;
    }
    break;	
    
  case NET1_WAIT:
    {
      if((g_MBNet1_task.time - g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get()) >= g_MBNet1_task.delay)
      { 
        if(g_MProcess[MASTER_PORT0].MasterStatus == MODBUS_MASTER_STATUS_END)
        {
          if(g_MProcess[MASTER_PORT0].Error == MODBUS_ERROR_OK)
          {
            //处理设备返回数据
            if(ModbusNet1Checking(g_MBNet1_task.pending_status) == MODBUS_ERROR_OK)
            {
              
              g_MBNet1_task.status = g_MBNet1_task.next_status;
            }
            else
            {
              g_MBNet1_task.status = g_MBNet1_task.pending_status;
            }
          }
          //接收数据有异常，重新发送本包数据
          else
          {
            g_MBNet1_task.status = g_MBNet1_task.pending_status;
          }
          
        }
        
        
      }
      
      
    }
    break;	
    
  default:
    break;
  }
  
  
  
}


static uint8_t ModbusNet1Checking(uint8_t status)
{
  uint8_t ret = MODBUS_ERROR_OK;
  
  
  switch(status)
  {
  case NET1_DEVICE1_WRITE:
    {
      ;
    }
    break;
    
  case NET1_DEVICE2_WRITE:
    {	
      ;
    }
    break;
    
  case NET1_DEVICE1_READ:
    {
      ;
    }
    break;
  case NET1_DEVICE2_READ:
    {
      ;
    }
    break;
    
  default:
    {
      ;
    }
    break;
  }
  
  return ret;
}

#endif
