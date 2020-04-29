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

const MODBUS_SLAVELISTtyp g_SlaveWriteList[SLAVE_WRITE_NUM] = {
{0, 1, 0, 51,  1000,  6},
{2, 1, 1, 51,  1000,  6},
{4, 1, 2, 51,  1000,  6},
{6, 1, 3, 51,  1000,  6},
{8, 1, 4, 51,  1000,  6}
};

const MODBUS_SLAVELISTtyp g_SlaveReadList[SLAVE_READ_NUM] = {
{10, 1, 0, 51,  1000,  3},
{12, 1, 1, 51,  1000,  3},
{14, 1, 2, 51,  1000,  3},
{16, 1, 3, 51,  1000,  3},
{18, 1, 4, 51,  1000,  3}
};




MODBUS_TASK_STRUC_T g_MBNet1_task;

//Modbus发送/接收数据Buffer
uint8_t g_u8MasterBuf[MODBUS_MASTER_NUMBER][PLC_MEMORY_LEN + 4] = {0};
uint16_t g_u8UserData[2] = {0x1234, 0x5678};

uint16_t g_u16DeviceReadCout = 0;
uint16_t g_u16DeviceWriteCout = 0;


extern MODBUS_PROCESStyp g_MProcess[MODBUS_MASTER_NUMBER];
extern Modbus_Port g_ModbusMasterPort[MODBUS_MASTER_NUMBER];



/*******************************************************************************
* Code
******************************************************************************/


void MB_Init()
{
  g_MBNet1_task.delay = 0;
  g_MBNet1_task.status = NET_DEVICE_READ;
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
  case NET_DEVICE_WRITE:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {

		if(g_u16DeviceWriteCout < SLAVE_WRITE_NUM)
		{
	        g_MProcess[MASTER_PORT0].Function = g_SlaveWriteList[g_u16DeviceWriteCout].Function;
        	g_MProcess[MASTER_PORT0].SlaveAddress = g_SlaveWriteList[g_u16DeviceWriteCout].SlaveAddress;
        	g_MProcess[MASTER_PORT0].Offset = g_SlaveWriteList[g_u16DeviceWriteCout].Offset;
        	g_MProcess[MASTER_PORT0].Number= g_SlaveWriteList[g_u16DeviceWriteCout].Number;
        	g_MProcess[MASTER_PORT0].RegisterAddress = g_SlaveWriteList[g_u16DeviceWriteCout].RegisterAddress;
	        g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
	        g_MProcess[MASTER_PORT0].Error = MODBUS_ERROR_OK;
	        //设置需要写入Device1的数据
	        //g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset] = g_u8UserData[0]&0xFF;
	        //g_MProcess[MASTER_PORT0].MBUF[g_MProcess[MASTER_PORT0].Offset + 1] = (g_u8UserData[0]>>8)&0xFF;
	        
	        //切换状态
	        g_MBNet1_task.pending_status = NET_DEVICE_WRITE;
	        g_MBNet1_task.next_status = NET_DEVICE_READ;
	        g_MBNet1_task.delay = g_SlaveWriteList[g_u16DeviceWriteCout].TimeOut;
	        g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
	        g_MBNet1_task.status = NET_WAIT;		

		}
		
		        
      }
      
    }
    break;
    
  case NET_DEVICE_READ:
    {
      if(MODBUS_MASTER_STATUS_END == g_MProcess[MASTER_PORT0].MasterStatus)
      {

      	if(g_u16DeviceReadCout < SLAVE_READ_NUM)
      	{
			g_MProcess[MASTER_PORT0].Function = g_SlaveReadList[g_u16DeviceReadCout].Function;
			g_MProcess[MASTER_PORT0].SlaveAddress = g_SlaveReadList[g_u16DeviceReadCout].SlaveAddress;
			g_MProcess[MASTER_PORT0].Offset = g_SlaveReadList[g_u16DeviceReadCout].Offset;
			g_MProcess[MASTER_PORT0].Number= g_SlaveReadList[g_u16DeviceReadCout].Number;
			g_MProcess[MASTER_PORT0].RegisterAddress = g_SlaveReadList[g_u16DeviceReadCout].RegisterAddress;
			g_MProcess[MASTER_PORT0].MasterStatus = MODBUS_MASTER_STATUS_START;
			
			//切换状态
			g_MBNet1_task.pending_status = NET_DEVICE_READ;
			g_MBNet1_task.next_status = NET_DEVICE_WRITE;
			g_MBNet1_task.delay = g_SlaveReadList[g_u16DeviceReadCout].TimeOut;
			g_MBNet1_task.time = g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get();
			g_MBNet1_task.status = NET_WAIT;

		}

        
      }
      
    }
    break;

  case NET_STOP:
    {
      ;
    }
    break;	
    
  case NET_WAIT:
    {	  
	  //如果当前在处理写数据
      if(g_MBNet1_task.pending_status == NET_DEVICE_WRITE)
      { 
        //从站未超时或者主机收到设备的响应帧并处理完成后，g_MProcess状态会设置为END
      	if(((g_MBNet1_task.time - g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get()) < g_MBNet1_task.delay)||
			(g_MProcess[MASTER_PORT0].MasterStatus == MODBUS_MASTER_STATUS_END))
        {
          //接收到的数据处理完毕，同时没有帧错误
          if(g_MProcess[MASTER_PORT0].Error == MODBUS_ERROR_OK)
          {
            g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
          }
          //接收数据有异常，访问下一个table
          else
          {
            g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
          }
		  
		  g_u16DeviceWriteCout++;	//尝试下一个table
		  if(g_u16DeviceWriteCout >= SLAVE_WRITE_NUM)
		  {
			g_u16DeviceWriteCout = 0;
		  }
          
        }    
		//超时没有接收到设备的响应帧，尝试下一个设备或地址
		else
		{
			g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
			g_u16DeviceWriteCout++;	//尝试下一个table
			if(g_u16DeviceWriteCout >= SLAVE_WRITE_NUM)
		  	{
				g_u16DeviceWriteCout = 0;
		  	}
		}
        
      }
	  //如果当前在处理读数据
	  else if(g_MBNet1_task.pending_status == NET_DEVICE_READ)
	  { 
      	//从站未超时或主站收到设备的响应帧并处理完成后，g_MProcess状态会设置为END
      	if(((g_MBNet1_task.time - g_ModbusMasterPort[MASTER_PORT0].Timer0_Value_Get()) < g_MBNet1_task.delay)||
			(g_MProcess[MASTER_PORT0].MasterStatus == MODBUS_MASTER_STATUS_END))

        {
          //接收到的数据处理完毕，同时没有帧错误
          if(g_MProcess[MASTER_PORT0].Error == MODBUS_ERROR_OK)
          {
            g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
          }
          //接收数据有异常，访问下一个table
          else
          {
            g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
          }

		  g_u16DeviceReadCout++;	//尝试下一个table
		  if(g_u16DeviceReadCout >= SLAVE_READ_NUM)
		  {
			g_u16DeviceReadCout = 0;
		  }
          
        }    
		//超时没有接收到设备的响应帧，尝试下一个设备或地址
		else
		{
			g_MBNet1_task.status = g_MBNet1_task.next_status;	//尝试下一个table
			g_u16DeviceReadCout++;	//尝试下一个table
			if(g_u16DeviceReadCout >= SLAVE_READ_NUM)
		  	{
				g_u16DeviceReadCout = 0;
		  	}
		}
        
      }
	  //状态机异常
	  else
	  {
	  	//重新初始化
		MB_Init();
	  }

    }
    break;	
    
  default:
    break;
  }
  
  
  
}



#endif
