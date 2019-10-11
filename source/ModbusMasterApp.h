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

#ifndef __MODBUSMASTERAPP_H__

#define __MODBUSMASTERAPP_H__


/*******************************************************************************
* Definitions
******************************************************************************/

//status

//NET1
#define NET1_DEVICE1_WRITE    					0

#define NET1_DEVICE2_WRITE    					1

#define NET1_DEVICE1_READ    					2

#define NET1_DEVICE2_READ    					3

#define NET1_STOP    							4

#define NET1_WAIT  	  					    	5


//parameter
#define DEVICE1_SLAVE_ADDR				1

#define DEVICE2_SLAVE_ADDR				2

#define DEVICE_COMM_RETRY				3

//Register
#define DEVICE1_WRITE_REGISTER			0

#define DEVICE1_READ_REGISTER			1

#define DEVICE2_WRITE_REGISTER			0

#define DEVICE2_READ_REGISTER			1


//APP

//MASTER NET1 


#define DEVICE1_WRITE_OFFSET			0

#define DEVICE1_READ_OFFSET				2

#define DEVICE2_WRITE_OFFSET			4

#define DEVICE2_READ_OFFSET				6

#define NET1_DELAY_MS					1000




typedef struct
{
  uint8_t	status;
  uint8_t	pending_status;
  uint8_t next_status;
  uint32_t time;
  uint32_t delay;
  
}MODBUS_TASK_STRUC_T;


/*******************************************************************************
* API
******************************************************************************/


void MB_Init();
void ModbusNet1MasterAPP();
static uint8_t ModbusNet1Checking(uint8_t status);



#endif

