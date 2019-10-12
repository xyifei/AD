#ifndef __USCI_UART_H
#define __USCI_UART_H
/*************************************************************************************************************************************************/
/*!     usci_uart.h
*
*       Header file for USCI Library UART software
*
*       April 2013
*
*/
/**************************************************************************************************************************************************
*       Copyright © 2013 Texas Instruments Incorporated - http://www.ti.com/                                                                      *
***************************************************************************************************************************************************
*  Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met: *
*                                                                                                                                                 *
*    Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.                 *
*                                                                                                                                                 *
*    Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the        *
*    documentation and/or other materials provided with the distribution.                                                                         *
*                                                                                                                                                 *
*    Neither the name of Texas Instruments Incorporated nor the names of its contributors may be used to endorse or promote products derived      *
*    from this software without specific prior written permission.                                                                                *
*                                                                                                                                                 *
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT          *
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT     *
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT         *
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    *
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE      *
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                                           *
**************************************************************************************************************************************************/
#include "usci_digital_io.h"


//======================================================================================================
#define UART_ACTION_ONGOING             0
#define UART_ACTION_COMPLETE            1

#define UART_RX_INT                     1
#define UART_TX_INT                     2

#define UART_COMMAND_STARTED            0
#define UART_FAIL_IN_USE                1
       

typedef enum {
    UART_9600_BAUD        = 0,
    UART_19200_BAUD       = 1,
    UART_38400_BAUD       = 2,
    UART_76800_BAUD       = 3,
    UART_115200_BAUD      = 4
} uartClkRate_t;

/*************************************************************************************************************/
/*                              PROTOTYPES                                                                   */
/*************************************************************************************************************/

#ifdef __CPLUSPLUS
extern "C" {
#endif

void Setup_UART (Port_t uartPort, unsigned char tx_pin, unsigned char rx_pin, primSecPeripheral_t primary_secondary, smclkSpeed_t smclkSpeed, uartClkRate_t uartClkRate);
unsigned char uart_getc(void);
void uart_gets(char* array, int max_length_bytes);
void uart_putc(unsigned char c);
void uart_puts(char *str);
void uart_putLong (unsigned long number);
void uart_WaitForChar(void);
unsigned char uart_GetChar(void);
unsigned char uart_GetFlag (void);
void uart_ClearWaitFlag(void);
void pause_UART(void);
void resume_UART(void);
unsigned char uart_rx_isr (void);
void uartPutHex (unsigned char hexByte);

#ifdef __CPLUSPLUS
}
#endif

#endif   // #ifndef __USCI_UART_H
