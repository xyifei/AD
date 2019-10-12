#ifndef __USCI_DIGITAL_IO_H
#define __USCI_DIGITAL_IO_H
/*************************************************************************************************************************************************/
/*!     usci_digital_io.h
*
*       Header file for USCI Library Digital IO software
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

#define DONT_WAKE_CPU                  0
#define WAKE_CPU                       1

typedef enum {
    USCI_A_SIDE    = 0,
    USCI_B_SIDE    = 1
} usci_a_b_side_t;

typedef enum {
    PORT_1         = 1,
    PORT_2         = 2,
    PORT_3         = 3,
    PORT_4         = 4
} Port_t;

typedef enum {
    SMCLK_1_MHZ    = 0,
    SMCLK_2_MHZ    = 1,
    SMCLK_4_MHZ    = 2,
    SMCLK_8_MHZ    = 3,
    SMCLK_12_MHZ   = 4,
    SMCLK_16_MHZ   = 5
} smclkSpeed_t;

typedef enum {
    PRIMARY_PERIPHERAL    = 0,
    SECONDARY_PERIPHERAL  = 1
} primSecPeripheral_t;

#define IF_PSxSEL2_PRESENT            ( (defined __MSP430AFE221__) || (defined __MSP430AFE222__) || (defined __MSP430AFE223__) || (defined __MSP430AFE231__) || \
                                        (defined __MSP430AFE232__) || (defined __MSP430AFE233__) || (defined __MSP430AFE251__) || (defined __MSP430AFE252__) || \
                                        (defined __MSP430AFE253__) || (defined __MSP430F2112__ ) || (defined __MSP430F2122__ ) || (defined __MSP430F2132__ ) || \
                                        (defined __MSP430F477__  ) || (defined __MSP430F478__  ) || (defined __MSP430F479__  ) || (defined __MSP430FG477__ ) || \
                                        (defined __MSP430FG478__ ) || (defined __MSP430FG479__ ) || (defined __MSP430G2102__ ) || (defined __MSP430G2112__ ) || \
                                        (defined __MSP430G2113__ ) || (defined __MSP430G2132__ ) || (defined __MSP430G2152__ ) || (defined __MSP430G2153__ ) || \
                                        (defined __MSP430G2202__ ) || (defined __MSP430G2203__ ) || (defined __MSP430G2212__ ) || (defined __MSP430G2213__ ) || \
                                        (defined __MSP430G2232__ ) || (defined __MSP430G2233__ ) || (defined __MSP430G2252__ ) || (defined __MSP430G2253__ ) || \
                                        (defined __MSP430G2302__ ) || (defined __MSP430G2303__ ) || (defined __MSP430G2312__ ) || (defined __MSP430G2313__ ) || \
                                        (defined __MSP430G2332__ ) || (defined __MSP430G2333__ ) || (defined __MSP430G2352__ ) || (defined __MSP430G2353__ ) || \
                                        (defined __MSP430G2402__ ) || (defined __MSP430G2403__ ) || (defined __MSP430G2412__ ) || (defined __MSP430G2413__ ) || \
                                        (defined __MSP430G2432__ ) || (defined __MSP430G2433__ ) || (defined __MSP430G2452__ ) || (defined __MSP430G2453__ ) || \
                                        (defined __MSP430G2513__ ) || (defined __MSP430G2533__ ) || (defined __MSP430G2553__ ) || (defined __MSP430G2755__ ) || \
                                        (defined __MSP430G2855__ ) || (defined __MSP430G2955__ ) || (defined __MSP430TCH5E__ ) || (defined __MSP430X21X2__ ) || \
                                        (defined __MSP430X47X__  ) || (defined __MSP430XG47X__) )


#endif /* __USCI_DIGITAL_IO_H */
