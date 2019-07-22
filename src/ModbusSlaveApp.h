/*
* The Clear BSD License
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* Copyright 2016-2017 NXP
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
#ifndef __MODBUSSLAVEAPP_H__

#define __MODBUSSLAVEAPP_H__


#include "fsl_common.h"

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_cadc.h"
#include "fsl_dmamux.h"
#include "fsl_edma.h"
#include "fsl_pit.h"
#include "fsl_cmp.h"
#include "fsl_dac.h"
#include "fsl_ftm.h"
#include "fsl_xbara.h"
#include "fsl_edma.h"
#include "fsl_uart_edma.h"
#include "fsl_flash.h"
#include "fsl_port.h"
#include "fsl_pwm.h"
#include "fsl_wdog.h"
#include "Modbus.h"

#include "pin_mux.h"
#include "clock_config.h"
#if defined(__cplusplus)
extern "C" {
#endif
  
  
  /*******************************************************************************
  * Definitions
  ******************************************************************************/
  
  
#define WDOG_WCT_INSTRUCITON_COUNT (256U)

  
  /*******************************************************************************
  * API
  ******************************************************************************/
  
  
  void WDOG_Configuration();
  
  
#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* _FSL_PIT_H_ */
