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

#include <string.h>
#include "Modbus.h"


/*******************************************************************************
* Definitions
******************************************************************************/


/*******************************************************************************
* Prototypes
******************************************************************************/

static uint32_t GenCrcCode(uint8_t *datapt,int16_t bytecount);


/*******************************************************************************
* Variables
******************************************************************************/
#ifdef MODBUS_SLAVE_USED
Modbus_Port g_ModbusSlavePort[MODBUS_SLAVE_NUMBER];
static uint8_t ModbusSlaveService(uint8_t port);
static void ModbusSlaveSendAck(uint16_t port);
#endif

#ifdef MODBUS_MASTER_USED
MODBUS_PROCESStyp g_MProcess[MODBUS_MASTER_NUMBER] = {0};
Modbus_Port g_ModbusMasterPort[MODBUS_MASTER_NUMBER] = {0};
#endif

//每个串口都有各自的接收发送缓冲区，处理的时候都Copy下面变量

uint8_t ModbusRecvBuffer[MODBUS_BUFF_SIZE] = {0};	
uint8_t ModbusSendBuffer[MODBUS_BUFF_SIZE] = {0}; 


uint8_t ClearMask[8] = { 0xfe,0xfd,0xfb,0xf7,0xef,0xdf,0xbf,0x7f };
uint8_t SetMask[8] = { 0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80 };
uint8_t BitClearHiMask[8] = { 0x00,0x01,0x03,0x07,0x0f,0x1f,0x3f,0x7f};
uint8_t BitClearLowMask[8] = { 0xff,0xfe,0xfc,0xf8,0xf0,0xe0,0xc0,0x80};


/*******************************************************************************
* Code
******************************************************************************/


//MODBUS通讯协议中CRC校验码算法
static uint32_t GenCrcCode(uint8_t *datapt,int16_t bytecount)
{
  uint16_t  Reg16=0xFFFF,mval=0xA001;
  int16_t   i;
  
  int8_t  j,flg; 
  for(i=0; i<bytecount; i++)
  {
    Reg16^=*(datapt+i);
    for(j=0;j<8;j++)
    {
      flg = 0;
      flg = (int8_t)(Reg16&0x0001);  
      Reg16>>=1;       
      if(flg==1)
      {
        Reg16=Reg16^mval;
      }
    }
  }
  
  return(Reg16);
}


#ifdef MODBUS_SLAVE_USED

void ModbusSlaveMainProcess(uint8_t port)
{
  
  if(g_ModbusSlavePort[port].ReceiveFlag == 1)
  {
    ModbusSlaveSendAck(port);
    g_ModbusSlavePort[port].ReceiveFlag = 0;
  }
  
}

void ModbusSlavePollSend(uint8_t port)
{
  
  if((g_ModbusSlavePort[port].SendFlag == MODBUS_SEND)&&(port < MODBUS_SLAVE_NUMBER))
  {
#if TIMER_MINUS == 1
    if((g_ModbusSlavePort[port].SendStartTime - g_ModbusSlavePort[port].Timer0_Value_Get()) > g_ModbusSlavePort[0].Timer0_Wait3_5char())
#else
      if((g_ModbusSlavePort[port].Timer0_Value_Get() - g_ModbusSlavePort[port].SendStartTime) > g_ModbusSlavePort[0].Timer0_Wait3_5char())
#endif
      {
        g_ModbusSlavePort[port].UART_SendData((unsigned char *)&g_ModbusSlavePort[port].s_TxBuf.byData, g_ModbusSlavePort[port].g_ucSendLen, port);
        g_ModbusSlavePort[port].g_ucSendLen = 0;
        g_ModbusSlavePort[port].SendFlag = MODBUS_CLOSE;
        
      }
  }
  
}



static void Wait35Char(uint8_t port)
{
  if(port < MODBUS_SLAVE_NUMBER)
  {
    g_ModbusSlavePort[port].SendStartTime = g_ModbusSlavePort[port].Timer0_Value_Get();
    g_ModbusSlavePort[port].SendFlag = MODBUS_SEND;
  }
  
}



/*************************************************************
Modbus Slave
*************************************************************/

static uint8_t ModbusSlaveService(uint8_t port)
{
  uint16_t wByteAddOffset, wChannelNum;
  uint8_t ucStartBit, ucWriteByteNum;
  uint8_t ucDataSize,ucSendDataLen,i,temp;
  uint16_t CrcReal;
  uint8_t  *pIn, *pOut;
  
  uint16_t wReadBitNum,wReturnByteNum;
  
  ModbusSendBuffer[0] = g_ModbusSlavePort[port].Local_Address;
  ModbusSendBuffer[1] = ModbusRecvBuffer[1];
  
  wByteAddOffset = MAKEWORD(ModbusRecvBuffer[2],ModbusRecvBuffer[3]);
  
  
  switch(MODBUS_ACCESS_HEADER(ModbusRecvBuffer)->bFuncCode)
  {
  case MODBUS_READ_DIGITAL_OUTPUT:
    
    wReadBitNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
    wReturnByteNum = wReadBitNum / 8;
    if( wReadBitNum % 8 != 0 )
    {
      wReturnByteNum++;
    }
    if(wReturnByteNum > (MODBUS_BUFF_SIZE - 5))
    {
      return 0;
    }
    ModbusSendBuffer[2] = (uint8_t)wReturnByteNum;			
    ucSendDataLen = 5 + (uint8_t)wReturnByteNum;
    
    
    if( wByteAddOffset >= M_AREA_OFFSET ) 
    {
      wByteAddOffset -= M_AREA_OFFSET;	
      if((wReturnByteNum + wByteAddOffset/8) < PLC_MEMORY_LEN)
      {
        ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
        wByteAddOffset = wByteAddOffset >> 3;
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
        
        if( (ucStartBit==0)&&(wReadBitNum%8==0) )
        {
          for( i=0; i<wReturnByteNum; i++ )
          {
            ModbusSendBuffer[3+i] = pOut[wByteAddOffset+i];
          }
          break;
        }
        
        if( wReadBitNum == 1 )
        {
          ModbusSendBuffer[3] = (pOut[wByteAddOffset] >> ucStartBit) & 0x01;
          break;
        }
        else
        {
          for(i=0; i<wReturnByteNum; i++)
          {
            ModbusSendBuffer[3+i] = pOut[wByteAddOffset+i] >> ucStartBit;
            temp = pOut[wByteAddOffset+i+1] << (8-ucStartBit);
            ModbusSendBuffer[3+i] |= temp;
            if( wReadBitNum < 8 )
            {
              ModbusSendBuffer[3+i] &= BitClearHiMask[wReadBitNum];
              break;
            }
            else
            {
              wReadBitNum -= 8;
            }
          }
        }
      }
      else
      {
        return 0;
      }
      
      
    }
    else
    {
      pOut = GetSlave_MemoryAddr(MODBUS_I_AREA, port);
      if((wReturnByteNum + wByteAddOffset/8) < PLC_INPUT_LEN)
      {
        ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
      	wByteAddOffset = wByteAddOffset >> 3;
        
        {
          if( (ucStartBit==0)&&(wReadBitNum%8==0))
          {
            for( i=0; i<wReturnByteNum; i++ )
            {
              ModbusSendBuffer[3+i] = pOut[wByteAddOffset+i];
            }
            break;
          }
          
          if( wReadBitNum == 1 )
          {
            ModbusSendBuffer[3] = (pOut[wByteAddOffset] >> ucStartBit) & 0x01;
            break;
          }
          else
          {
            for(i=0; i<wReturnByteNum; i++)
            {
              ModbusSendBuffer[3+i] = pOut[wByteAddOffset+i] >> ucStartBit;
              temp = pOut[wByteAddOffset+i+1] << (8-ucStartBit);
              ModbusSendBuffer[3+i] |= temp;
              if( wReadBitNum < 8 )
              {
                ModbusSendBuffer[3+i] &= BitClearHiMask[wReadBitNum];
                break;
              }
              else
              {
                wReadBitNum -= 8;
              }
            }
          }
          
        }       
      }
      else
      {
        return 0;
      }
      
      
    }		
    break;
    
  case MODBUS_READ_DIGITAL_INPUT:
    
    wReadBitNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
    wReturnByteNum = wReadBitNum / 8;
    if( wReadBitNum % 8 != 0 )
    {
      wReturnByteNum++;
    }
    if(wReturnByteNum > MODBUS_BUFF_SIZE - 5)
    {
      return 0;
    }
    
    ModbusSendBuffer[2] = (uint8_t)wReturnByteNum;		
    ucSendDataLen = 5 + (uint8_t)wReturnByteNum;
    
    
    if( wByteAddOffset >= M_AREA_OFFSET ) 
    {
      wByteAddOffset -= M_AREA_OFFSET;
      
      if((wReturnByteNum + wByteAddOffset/8) < PLC_MEMORY_LEN)
      {
        pIn = GetSlave_MemoryAddr(MODBUS_M_AREA, port); 
      }
      else
      {
        return 0;
      }
    }
    else
    {
      
      if((wReturnByteNum + wByteAddOffset/8) < PLC_INPUT_LEN)
      {
        pIn = GetSlave_MemoryAddr(MODBUS_I_AREA, port);
      }
      else
      {
        return 0;
      }
    }
    
    ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
    
    wByteAddOffset = wByteAddOffset >> 3;
    
    
    if( (ucStartBit==0)&&(wReadBitNum%8==0) )
    {
      for( i=0; i<wReturnByteNum; i++ )
      {
        ModbusSendBuffer[3+i] = pIn[wByteAddOffset+i];
      }
      break;
    }
    
    if( wReadBitNum == 1 )
    {
      ModbusSendBuffer[3] = (pIn[wByteAddOffset] >> ucStartBit) & 0x01;
      break;
    }
    else
    {
      for(i=0; i<wReturnByteNum; i++)
      {
        ModbusSendBuffer[3+i] = pIn[wByteAddOffset+i] >> ucStartBit;
        temp = pIn[wByteAddOffset+i+1] << (8-ucStartBit);
        ModbusSendBuffer[3+i] |= temp;
        if( wReadBitNum < 8 )
        {
          ModbusSendBuffer[3+i] &= BitClearHiMask[wReadBitNum];
          break;
        }
        else
        {
          wReadBitNum -= 8;
        }
      }
    }							
    break;
  case MODBUS_READ_ANOLOG_OUTPUT:	
    wChannelNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
    ucDataSize = (uint8_t)(wChannelNum*2);
    
    if(ucDataSize > (MODBUS_BUFF_SIZE - 5))
    {
      return 0;
    }
    
    
    ModbusSendBuffer[2] = ucDataSize;
    
    if( wByteAddOffset >= M_AREA_OFFSET )
    {
      wByteAddOffset -= M_AREA_OFFSET;		
      
      wByteAddOffset *= 2;
      
      if((wByteAddOffset + ucDataSize) < PLC_MEMORY_LEN)
      {
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port); 
      }
      else
      {
        return 0;
      }
      
      for(i=0; i<ucDataSize; i=i+2)
      {
        ModbusSendBuffer[3+i] = *(pOut+wByteAddOffset+i+1);
        ModbusSendBuffer[4+i] = *(pOut+wByteAddOffset+i);
      }	
    }
    else
    {		
      wByteAddOffset *= 2;
      if((wByteAddOffset + ucDataSize) < (PLC_INPUT_LEN+PLC_OUTPUT_LEN))
      {
        pOut = GetSlave_MemoryAddr(MODBUS_I_AREA, port);
      }
      else
      {
        return 0;
      }
      
      {
        for(i=0; i<ucDataSize; i=i+2)
        {
          ModbusSendBuffer[3+i] = *(pOut+wByteAddOffset+i+1);
          ModbusSendBuffer[4+i] = *(pOut+wByteAddOffset+i);
        }	
      }	
    }			
    ucSendDataLen = 5 + ucDataSize;	
    break;
    
  case MODBUS_READ_ANOLOG_INPUT:
    wChannelNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
    ucDataSize = (uint8_t)(wChannelNum*2);
    
    if(ucDataSize > (MODBUS_BUFF_SIZE - 5))
    {
      return 0;
    }
    
    
    ModbusSendBuffer[2] = ucDataSize;
    
    if( wByteAddOffset >= M_AREA_OFFSET ) 
    {
      wByteAddOffset -= M_AREA_OFFSET;		
      
      wByteAddOffset *= 2;
      pIn = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
      if((wByteAddOffset + ucDataSize) > PLC_MEMORY_LEN)
      {
        return 0;
      }
    }
    else
    {
      wByteAddOffset *= 2;
      if((wByteAddOffset + ucDataSize) < (PLC_INPUT_LEN+PLC_OUTPUT_LEN) )
      {
        pIn = GetSlave_MemoryAddr(MODBUS_I_AREA, port);
      }
      else
      {
        return 0;
      }
    }
    
    for(i=0; i<ucDataSize; i=i+2)
    {
      ModbusSendBuffer[3+i] = *(pIn+wByteAddOffset+i+1);
      ModbusSendBuffer[4+i] = *(pIn+wByteAddOffset+i);
    }	
    ucSendDataLen = 5 + ucDataSize;	
    break;
    
  case MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT:
    
    if( wByteAddOffset >= M_AREA_OFFSET ) 
    {
      wByteAddOffset -= M_AREA_OFFSET;
      ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
      wByteAddOffset = wByteAddOffset >> 3;
      if( wByteAddOffset < PLC_MEMORY_LEN )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
        if(( ModbusRecvBuffer[4] == 0xFF )&&(ModbusRecvBuffer[5] == 0x00))
        {
          *(pOut + wByteAddOffset) |= SetMask[ucStartBit];
        }
        if(( ModbusRecvBuffer[4] == 0x00) &&(ModbusRecvBuffer[5] == 0x00))
        {
          *(pOut + wByteAddOffset) &= ClearMask[ucStartBit];
        }
      }
      else
      {
        return 0;
      }
    }
    else
    {
      ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
      wByteAddOffset = wByteAddOffset >> 3;
      if( wByteAddOffset < PLC_OUTPUT_LEN )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_Q_AREA, port);
        if(( ModbusRecvBuffer[4] == 0xFF )&&(ModbusRecvBuffer[5] == 0x00))
        {
          *(pOut + wByteAddOffset) |= SetMask[ucStartBit];
        }
        if(( ModbusRecvBuffer[4] == 0x00) &&(ModbusRecvBuffer[5] == 0x00))
        {
          *(pOut + wByteAddOffset) &= ClearMask[ucStartBit];
        }
      }
      else
      {
        return 0;
      }
      
    }
    
    for(i=0; i<6; i++)
    {
      ModbusSendBuffer[i] = ModbusRecvBuffer[i];
    }
    ucSendDataLen = 8;
    break;
    
  case MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT:
    if( wByteAddOffset >= M_AREA_OFFSET )
    {
      wByteAddOffset -= M_AREA_OFFSET;		
      wByteAddOffset *= 2; 
      if( wByteAddOffset < PLC_MEMORY_LEN )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
        *(pOut+wByteAddOffset) = ModbusRecvBuffer[5];
        *(pOut+wByteAddOffset+1) = ModbusRecvBuffer[4];
      }
      else
      {
        return 0;
      }
    }
    else
    {
      wByteAddOffset *= 2; 
      if( wByteAddOffset < PLC_OUTPUT_LEN )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_Q_AREA, port);
        *(pOut+wByteAddOffset) = ModbusRecvBuffer[5];
        *(pOut+wByteAddOffset+1) = ModbusRecvBuffer[4];
      }
      else
      {
        return 0;
      }
      
    }
    
    for(i=0; i<6; i++)
    {
      ModbusSendBuffer[i] = ModbusRecvBuffer[i];
    }
    ucSendDataLen = 8;
    break;
  case MODBUS_FORCE_MUL_DIGITAL_OUTPUT:
    
    if( wByteAddOffset >= M_AREA_OFFSET )
    {
      wByteAddOffset -= M_AREA_OFFSET;
      if(((wByteAddOffset/8)+ModbusRecvBuffer[6]) < PLC_MEMORY_LEN)
      {
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
      }
      else
      {
        return 0;
      }
      
    }
    else if(((wByteAddOffset/8)+ModbusRecvBuffer[6]) < PLC_OUTPUT_LEN)
    {
      pOut = GetSlave_MemoryAddr(MODBUS_Q_AREA, port);
    }
    else
    {
      return 0;
    }
    
    
    {
      ucStartBit = (uint8_t)(wByteAddOffset & 0x0007);
      wByteAddOffset = wByteAddOffset >> 3;
      
      {
        ucWriteByteNum = ModbusRecvBuffer[6];
        wReadBitNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
        
        if( ucStartBit==0 )
        {
          for(i=0; i<ucWriteByteNum; i++)
          {
            if( wReadBitNum >= 8 )
            {
              pOut[wByteAddOffset+i] = ModbusRecvBuffer[7+i];
              wReadBitNum -= 8;
            }
            else
            {
              pOut[wByteAddOffset+i] &= BitClearLowMask[wReadBitNum];
              ModbusRecvBuffer[7+i] &= BitClearHiMask[wReadBitNum];
              pOut[wByteAddOffset+i] |= ModbusRecvBuffer[7+i];
            }
          }
        }
        else
        {
          for(i=0; i<ucWriteByteNum; i++)
          {
            if( wReadBitNum >= 8 )
            {
              pOut[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];
              temp = ModbusRecvBuffer[7+i] << ucStartBit;
              pOut[wByteAddOffset+i] |= temp;
              pOut[wByteAddOffset+i+1] &= BitClearLowMask[ucStartBit];
              temp = ModbusRecvBuffer[7+i] >> (8-ucStartBit);
              pOut[wByteAddOffset+i+1] |= temp;
              wReadBitNum -= 8;
            }
            else 
            {
              if( wReadBitNum < (8-ucStartBit))
              {
                temp = pOut[wByteAddOffset+i] >> ucStartBit;
                temp &= BitClearLowMask[wReadBitNum];
                ModbusRecvBuffer[7+i] &=  BitClearHiMask[wReadBitNum];
                temp |= ModbusRecvBuffer[7+i];
                temp <<= ucStartBit;
                pOut[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];
                pOut[wByteAddOffset+i] |= temp;
              }
              else
              {
                pOut[wByteAddOffset+i] &= BitClearHiMask[ucStartBit];
                temp = ModbusRecvBuffer[7+i] << ucStartBit;
                pOut[wByteAddOffset+i] |= temp;
                pOut[wByteAddOffset+i+1] &= BitClearLowMask[wReadBitNum-(8-ucStartBit)];
                temp = ModbusRecvBuffer[7+i] >> (8-ucStartBit);
                temp &= BitClearHiMask[wReadBitNum-(8-ucStartBit)];
                pOut[wByteAddOffset+i+1] |= temp;
              }
            }
          }
        }//end else
      }//end if(wByteAddOffset<DATAIDX_MEMORY_SIZE): in safe address			
    }//end M Area
    for(i=0; i<6; i++)
    {
      ModbusSendBuffer[i] = ModbusRecvBuffer[i];
    }
    ucSendDataLen = 8;
    break;
  case MODBUS_FORCE_MUL_ANOLOG_OUTPUT:
    ucDataSize = ModbusRecvBuffer[6];
    wChannelNum = MAKEWORD(ModbusRecvBuffer[4],ModbusRecvBuffer[5]);
    
    if( wByteAddOffset >= M_AREA_OFFSET )
    {
      wByteAddOffset -= M_AREA_OFFSET;		
      
      wByteAddOffset *= 2; 								
      if( (wByteAddOffset+wChannelNum*2) < PLC_MEMORY_LEN )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_M_AREA, port);
        for(i=0; i<ucDataSize; i=i+2)
        {
          *(pOut+wByteAddOffset+i) = ModbusRecvBuffer[8+i];
          *(pOut+wByteAddOffset+1+i) = ModbusRecvBuffer[7+i];
        }
      }
      else
      {
        return 0;
      }
      
    }
    else
    {
      wByteAddOffset *= 2;							  
      if( (wByteAddOffset+wChannelNum*2) < (PLC_INPUT_LEN + PLC_OUTPUT_LEN) )
      {
        pOut = GetSlave_MemoryAddr(MODBUS_Q_AREA, port);
        for(i=0; i<ucDataSize; i=i+2)
        {
          *(pOut+wByteAddOffset+i) = ModbusRecvBuffer[8+i];
          *(pOut+wByteAddOffset+1+i) = ModbusRecvBuffer[7+i];
        }
      }
      else
      {
        return 0;
      }
      
    }
    
    ucSendDataLen = 8;
    for(i=0; i<6; i++)
    {
      ModbusSendBuffer[i]= ModbusRecvBuffer[i];
    }
    break;
  default:
    ucSendDataLen = 6;
    break;
  }
  CrcReal = GenCrcCode(ModbusSendBuffer, ucSendDataLen-2);
  ModbusSendBuffer[ucSendDataLen-2] = (uint8_t)(CrcReal & 0x00FF);
  ModbusSendBuffer[ucSendDataLen-1] = (uint8_t)(CrcReal >> 8);
  return ucSendDataLen;
}




uint8_t ModbusSlaveReceiveInt(uint8_t *pbyData, uint16_t uCount,uint8_t port)
{
  if ((uCount >= 6)&&(port < MODBUS_SLAVE_NUMBER))
  {
    if ( MODBUS_ACCESS_HEADER(pbyData)->bSlaveAdd == g_ModbusSlavePort[port].Local_Address)
    {		
      switch(MODBUS_ACCESS_HEADER(pbyData)->bFuncCode)
      {
      case MODBUS_READ_DIGITAL_OUTPUT:
      case MODBUS_READ_DIGITAL_INPUT:
      case MODBUS_READ_ANOLOG_OUTPUT:
      case MODBUS_READ_ANOLOG_INPUT:
      case MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT:
      case MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT:
        {
          g_ModbusSlavePort[port].ucSize = 8;
          
          break;
        }
      case MODBUS_FORCE_MUL_DIGITAL_OUTPUT:
      case MODBUS_FORCE_MUL_ANOLOG_OUTPUT:
        if (uCount < 7)
        {
          return FALSE;
        }
        
        g_ModbusSlavePort[port].ucSize =  *(pbyData+6) + 9;
        
        
        break;
      default:
        
        if(port < MODBUS_SLAVE_NUMBER)
        {
          return TRUE;
        }
        else
        {
          return FALSE;
        }				
        
        
      }
      
      if (uCount < g_ModbusSlavePort[port].ucSize)
      {
        return FALSE;
      }
      else
      { 
        g_ModbusSlavePort[port].ReceiveFlag = 1;
        g_ModbusSlavePort[port].s_RxBuf.uCount = 0;
        return TRUE;
      }
      
    }
    
    return FALSE;
  }
  else
  {
    return FALSE;
  }	
  
}


static void ModbusSlaveSendAck(uint16_t port)
{
  uint8_t *pbyData;
  uint16_t wCrcCheck, wCrcReal;
  uint8_t ucSendLen;
  
  if(port < MODBUS_SLAVE_NUMBER)
  {
    
    pbyData= (uint8_t*)g_ModbusSlavePort[port].s_RxBuf.byData;					  
    wCrcCheck = GenCrcCode(pbyData, g_ModbusSlavePort[port].ucSize-2);
    wCrcReal = MAKEWORD(*(pbyData+(g_ModbusSlavePort[port].ucSize-1)), *(pbyData+(g_ModbusSlavePort[port].ucSize-2)));
    if( wCrcCheck == wCrcReal )
    {
      User_Memcpy(ModbusRecvBuffer, pbyData, g_ModbusSlavePort[port].ucSize);
      ucSendLen = ModbusSlaveService(port); 
      if(ucSendLen != 0)
      {
        g_ModbusSlavePort[port].g_ucSendLen = ucSendLen;
        User_Memcpy((void*)g_ModbusSlavePort[port].s_TxBuf.byData, &ModbusSendBuffer[0], g_ModbusSlavePort[port].g_ucSendLen);	  
        Wait35Char(port); 
        
      }
      else
      {
        ;
      }
    }
    
  }
  
}

#endif 


/*************************************************************
Modbus Master
*************************************************************/

#ifdef MODBUS_MASTER_USED

/*************************************************************
中断接收函数中调用，用于接收Modbus帧数据
*************************************************************/

int8_t ModbusMasterReceiveInt(uint8_t *pbyData, uint16_t uCount,uint8_t port)
{
  
  if((uCount >= 6)&&(port < MODBUS_MASTER_NUMBER))
  {
    //主站帧头判断
    if((MODBUS_ACCESS_HEADER(pbyData)->bSlaveAdd == g_MProcess[port].SlaveAddress)
       &&(MODBUS_ACCESS_HEADER(pbyData)->bFuncCode == g_MProcess[port].Function)
         )
    {
      switch(MODBUS_ACCESS_HEADER(pbyData)->bFuncCode)
      {
      case MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT:
      case MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT:
      case MODBUS_FORCE_MUL_DIGITAL_OUTPUT:
      case MODBUS_FORCE_MUL_ANOLOG_OUTPUT:
        {
          g_ModbusMasterPort[port].ucSize = 8;
          
          break;
        }
      case MODBUS_READ_DIGITAL_OUTPUT:
      case MODBUS_READ_DIGITAL_INPUT:
      case MODBUS_READ_ANOLOG_OUTPUT:
      case MODBUS_READ_ANOLOG_INPUT:
        
        g_ModbusMasterPort[port].ucSize = *(pbyData+2)+5;
        
        break;
        
      default:
        
        if(port < MODBUS_MASTER_NUMBER)
        {
          return TRUE;
        }
        else
        {
          return FALSE;
        }	
        
      }
      
      if (uCount < g_ModbusMasterPort[port].ucSize)
      {
        return FALSE;
      }
      else
      {
        g_ModbusMasterPort[port].ReceiveFlag = 1;
        return TRUE;
      }
      
    }
    else
    {
      return FALSE;
    }
  }
  
  return FALSE;
}

/*************************************************************
主循环中调用，用于处理接收Modbus数据
*************************************************************/


void ModbusMasterMainReceive(uint8_t port)
{
  
  if((port < MODBUS_MASTER_NUMBER)&&(g_ModbusMasterPort[port].ReceiveFlag == 1))
  {
    ModbusMasterProcessReceive(port);
    g_ModbusMasterPort[port].ReceiveFlag = 0 ;
  }
  
}
/*************************************************************
内部函数用于校验数据
*************************************************************/

static void ModbusMasterProcessReceive(uint16_t port)
{
  uint8_t *pbyData;
  uint16_t wCrcCheck, wCrcReal;
  
  if(port < MODBUS_MASTER_NUMBER)
  {
    pbyData= (uint8_t*)g_ModbusMasterPort[port].s_RxBuf.byData;					  
    wCrcCheck = GenCrcCode(pbyData, g_ModbusMasterPort[port].ucSize-2);
    wCrcReal = MAKEWORD(*(pbyData+(g_ModbusMasterPort[port].ucSize-1)), *(pbyData+(g_ModbusMasterPort[port].ucSize-2)));
    if( wCrcCheck == wCrcReal )
    {
      User_Memcpy(ModbusRecvBuffer, pbyData, g_ModbusMasterPort[port].ucSize);
      ModbusMasterService(port);
    }
    
  }
  
}
/*************************************************************
内部函数用于服务数据
*************************************************************/

static void ModbusMasterService(uint16_t port)
{
  uint8_t i = 0, funcode = 0;
  uint8_t *pby;
  
  funcode = MODBUS_ACCESS_HEADER(ModbusRecvBuffer)->bFuncCode;
  
  switch(funcode)
  {
    
  case MODBUS_READ_DIGITAL_OUTPUT:
  case MODBUS_READ_DIGITAL_INPUT:
    
    pby = GetMaster_MemoryAddr(MODBUS_M_AREA, port);
    if(pby == NULL)
    {
      g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END; 						
      
      return;
    }
    pby = (uint8_t *)((uint32_t)pby + g_MProcess[port].Offset);
    
    if((g_MProcess[port].Offset + ModbusRecvBuffer[2]) > PLC_MEMORY_LEN)
    {
      
      g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END;						  
      
      return;
      
    }
    
    User_Memcpy(pby, (uint8_t *)&ModbusRecvBuffer[3], ModbusRecvBuffer[2]);
    
    g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END; 
    
  case MODBUS_READ_ANOLOG_OUTPUT:
  case MODBUS_READ_ANOLOG_INPUT:		
    pby = GetMaster_MemoryAddr(MODBUS_M_AREA, port);
    if(pby == NULL)
    {
      
      g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
      
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END; 						
      
      return;
    }
    pby = (uint8_t *)((uint32_t)pby + g_MProcess[port].Offset);
    
    if((g_MProcess[port].Offset + ModbusRecvBuffer[2]) < PLC_MEMORY_LEN)
    {
      uint8_t uctemp;
      for(i=0; i<ModbusRecvBuffer[2]; i=i+2)
      {
        uctemp = ModbusRecvBuffer[3+i];
        ModbusRecvBuffer[3+i] = ModbusRecvBuffer[4+i];	
        ModbusRecvBuffer[4+i] = uctemp;
      }
      User_Memcpy(pby, (uint8_t *)&ModbusRecvBuffer[3], ModbusRecvBuffer[2]);
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END; 
    }
    else
    {
      g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END;						  
      
      return;
      
    }
    
    
    break;
    
  case MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT:
  case MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT:
  case MODBUS_FORCE_MUL_DIGITAL_OUTPUT:
  case MODBUS_FORCE_MUL_ANOLOG_OUTPUT:
    
    g_MProcess[port].Error = MODBUS_ERROR_OK;
    g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END; 
    
    break;
    
  default:
    
    break;
    
  }
  
  
}



/*************************************************************
主循环中调用，用于生成发送报文
*************************************************************/


void ModbusMasterSendMessage(uint8_t port)
{
  uint16_t SendLen = 0, CrcReal = 0, i = 0;
  
  if(MODBUS_MASTER_STATUS_START == g_MProcess[port].MasterStatus)
  {
    ModbusSendBuffer[0] = g_MProcess[port].SlaveAddress;
    ModbusSendBuffer[1] = g_MProcess[port].Function;
    ModbusSendBuffer[2] = (uint8_t)(g_MProcess[port].RegisterAddress >> 8);
    ModbusSendBuffer[3] = (uint8_t)g_MProcess[port].RegisterAddress & 0xFF;
    ModbusSendBuffer[4] = 0;
    ModbusSendBuffer[5] = g_MProcess[port].Number;
    
    
    switch(g_MProcess[port].Function)
    {
    case MODBUS_READ_DIGITAL_OUTPUT:
      ModbusSendBuffer[1] = MODBUS_READ_DIGITAL_OUTPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number+7)/8) > PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        
        return;
      }
      
      ModbusSendBuffer[4] = 0;
      ModbusSendBuffer[5] = (uint8_t)g_MProcess[port].Number;		
      CrcReal = GenCrcCode(ModbusSendBuffer, 6);
      ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);
      ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
      
      SendLen = 8;
      
      break;
    case MODBUS_READ_DIGITAL_INPUT:
      ModbusSendBuffer[1] = MODBUS_READ_DIGITAL_INPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number+7)/8)>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        
        return;
      }
      ModbusSendBuffer[4] = 0;
      ModbusSendBuffer[5] = (uint8_t)g_MProcess[port].Number;		
      CrcReal = GenCrcCode(ModbusSendBuffer, 6);
      ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);
      ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
      
      SendLen = 8;
      
      break;
    case MODBUS_READ_ANOLOG_INPUT:
      ModbusSendBuffer[1] = MODBUS_READ_ANOLOG_INPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number*2))>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        return;
      }
      
      ModbusSendBuffer[4] = 0;
      ModbusSendBuffer[5] = (uint8_t)g_MProcess[port].Number;		
      CrcReal = GenCrcCode(ModbusSendBuffer, 6);
      ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);
      ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
      
      SendLen = 8;
      
      break;
    case MODBUS_READ_ANOLOG_OUTPUT:
      ModbusSendBuffer[1] = MODBUS_READ_ANOLOG_OUTPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number*2))>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        return;
      }
      
      ModbusSendBuffer[4] = 0;
      ModbusSendBuffer[5] = (uint8_t)g_MProcess[port].Number;		
      CrcReal = GenCrcCode(ModbusSendBuffer, 6);
      ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);
      ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
      
      SendLen = 8;
      
      break;
      
    case MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT:	
      ModbusSendBuffer[1] = MODBUS_FORCE_SINGLE_DIGITAL_OUTPUT;
      if((g_MProcess[port].Offset + 1)>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        
        return;
      }
      
      ModbusSendBuffer[4] = g_MProcess[port].MBUF[g_MProcess[port].Offset];
      ModbusSendBuffer[5] = g_MProcess[port].MBUF[g_MProcess[port].Offset + 1];					
      {
        CrcReal = GenCrcCode(ModbusSendBuffer, 6);
        ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);	
        ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
        
      }
      
      SendLen = 8;
      
      break;
    case MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT:
      ModbusSendBuffer[1] = MODBUS_FORCE_SINGLE_ANOLOG_OUTPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number*2))>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        return;
      }
      
      ModbusSendBuffer[4] = g_MProcess[port].MBUF[g_MProcess[port].Offset];
      ModbusSendBuffer[5] = g_MProcess[port].MBUF[g_MProcess[port].Offset + 1];					
      {
        CrcReal = GenCrcCode(ModbusSendBuffer, 6);
        ModbusSendBuffer[6] = (unsigned char)(CrcReal & 0x00FF);	
        ModbusSendBuffer[7] = (unsigned char)(CrcReal >> 8);
        
      }
      SendLen = 8;	
      
      break;
    case MODBUS_FORCE_MUL_DIGITAL_OUTPUT:	 
      ModbusSendBuffer[1] = MODBUS_FORCE_MUL_DIGITAL_OUTPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number+7)/8)>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        return;
      }
      ModbusSendBuffer[4] = 0;
      ModbusSendBuffer[5] = (unsigned char)g_MProcess[port].Number;
      
      if((g_MProcess[port].Number % 8) != 0)
      {
        ModbusSendBuffer[6] = (uint8_t)(g_MProcess[port].Number / 8)+1;
      }
      else
      {
        ModbusSendBuffer[6] = (g_MProcess[port].Number / 8);
      }
      
      for(i=0; i<ModbusSendBuffer[6]; i++)
      {
        ModbusSendBuffer[i+7] = g_MProcess[port].MBUF[i + g_MProcess[port].Offset];
      }
      
      CrcReal = GenCrcCode(ModbusSendBuffer, 7+i);
      ModbusSendBuffer[i+7] = (unsigned char)(CrcReal & 0x00FF); 
      ModbusSendBuffer[i+8] = (unsigned char)(CrcReal >> 8); 
      SendLen = i + 9;	
      break;
    case MODBUS_FORCE_MUL_ANOLOG_OUTPUT:
      ModbusSendBuffer[1] = MODBUS_FORCE_MUL_ANOLOG_OUTPUT;
      if((g_MProcess[port].Offset+(g_MProcess[port].Number*2))>PLC_MEMORY_LEN)
      {
        g_MProcess[port].Error = MODBUS_ERROR_MEMORY;
        return;
      }
      
      ModbusSendBuffer[6] = (unsigned char)(g_MProcess[port].Number * 2);
      
      for(i = 0; i < ModbusSendBuffer[6]; i=i+2)
      {
        ModbusSendBuffer[i+7] = g_MProcess[port].MBUF[i+1+g_MProcess[port].Offset];
        ModbusSendBuffer[i+8] = g_MProcess[port].MBUF[i+g_MProcess[port].Offset];
      }
      
      CrcReal = GenCrcCode(ModbusSendBuffer, 7+i);
      ModbusSendBuffer[i+7] = (unsigned char)(CrcReal & 0x00FF); 
      ModbusSendBuffer[i+8] = (unsigned char)(CrcReal >> 8); 
      SendLen = i + 9;
      
      break;
      
      
    default:
      g_MProcess[port].Error = MODBUS_ERROR_FUNCTION; 
      break;
    }
    
    if(g_MProcess[port].Error == MODBUS_ERROR_OK)
    {
      g_ModbusMasterPort[port].SendStartTime = g_ModbusMasterPort[port].Timer0_Value_Get();
      g_ModbusMasterPort[port].UART_SendData(&ModbusSendBuffer[0], SendLen,port);
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_SENDING;
    }
    
    
  }
  else if(MODBUS_MASTER_STATUS_SENDING == g_MProcess[port].MasterStatus)
  {
    if((g_ModbusMasterPort[port].SendStartTime - g_ModbusMasterPort[port].Timer0_Value_Get()) > g_ModbusMasterPort[port].TimeOutLimit)
    {
      g_MProcess[port].Error = MODBUS_ERROR_TIMEOUT_LIMIT;
      g_MProcess[port].MasterStatus = MODBUS_MASTER_STATUS_END;
    }
  }
  
  else
  {
    ;
  }
  
  
}


#endif


