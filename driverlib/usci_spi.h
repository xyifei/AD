#ifndef __USCI_SPI_H
#define __USCI_SPI_H
/*************************************************************************************************************************************************/
/*!     usci_spi.h
*
*       Header file for USCI Library SPI software
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
#define SPI_INVALID_RATE         0xFF
#define SPI_INVALID_COMMAND      0x01
#define SPI_IN_USE               0x02
#define SPI_SUCCESS              0x00
       
typedef enum {
    SPI_1_MHZ        = 0,
    SPI_2_MHZ        = 1,
    SPI_4_MHZ        = 2,
    SPI_8_MHZ        = 3
} spiClkRate_t;

typedef enum {
    SPI_CHANGE_ON_FIRST_EDGE      = 0,
    SPI_CAPT_ON_FIRST_EDGE        = (1<<7)
} spiClkPhase_t;

typedef enum {
    SPI_INACTIVE_LOW_POL          = 0,
    SPI_ACTIVE_HIGH_POL           = 0,
    SPI_ACTIVE_LOW_POL            = (1<<6),
    SPI_INACTIVE_HIGH_POL         = (1<<6)
} spiPolarity_t;

typedef struct {
    volatile unsigned char    *PortPtr;
    unsigned char              bitNum;
} SpiRegisterStore_type;

/*************************************************************************************************************/
/*                              PROTOTYPES                                                                   */
/*************************************************************************************************************/

#ifdef __CPLUSPLUS
extern "C" {
#endif

unsigned char Setup_SPI_Master (Port_t spiPort, unsigned char somi_pin, unsigned char simo_pin, unsigned char sclk_pin,
                                primSecPeripheral_t primary_secondary, usci_a_b_side_t a_b_side,
                                smclkSpeed_t smclkSpeed, spiClkRate_t spiClkRate,
                                spiPolarity_t polarity, spiClkPhase_t phase);
unsigned char RegisterSpiDevice (Port_t ste_port, unsigned char ste_pin);
unsigned char SPI_Write (unsigned char spiHandle, unsigned char *outData, unsigned char *inData, unsigned char length);
unsigned char SPI_Write_Byte (unsigned char data);
void SPI_Write_Shutdown_Byte (unsigned char data);
void Change_SPI_Polarity (spiPolarity_t polarity);
void Change_SPI_Phase (spiClkPhase_t phase);

#ifdef __CPLUSPLUS
}
#endif

#endif  // #ifndef __USCI_SPI_H

