/*************************************************************************************************************************************************/
/*!     USCI UART Code
*
*       This code is designed for USRT blocks on the MSP430 using an USCI hardware module.
* 
*       The code was desigmed on MSP430 that do not have the UCA0IV, Interrupt Vector, registers for the UAT module.
*        
*       The code is specifically written to use UCA0 UART functions the user can indicate the location of the UART pins in Setup_UART()
* 
*       It is important to note that the UART Receive function uart_getc(), which is used in uart_gets(), will place the processor in LPM0 mode
*       and wait for the next UART RX interrupt. As such, this is a blocking function. The uart_getchar() function can be used to gather the
*       last byte received from the UART for people who wish to implement their own UART 
* 
*       This software has an interrupt service routine that is tied to the USCIAB0RX vector. This vestor is used for the UART RX a and the
*       I2C error indicators. If a user wishes to utilize the I2C error interrupts, the ISR code must be modified.
* 
*       Note, the library version of the I2C logic does not utilize the I2C error interrupts.
* 
*       The UART TX is accomplished via polling (the uart_putc() function waits for the TX Fifo to be empty), so it does not use the USCIAB0TX
*       interrupt vector (which is shared with the I2C TX/RX interrupts).
* 
*       Standard functions that will be used by implementors are:
*          Setup_UART() - This function is called once to configure the UART hardware
*          uart_gets()  - This function will receive a string of data from the UART. It is a blocking function that will place the MSP430 in LPM0
*          uart_getc()  - This function will receive a single character from the UART. It is a blocking function that will place the MSP430 in LPM0
*          uart_puts()  - This function will write a string out on the UART buffer. It is a blocking function as it will poll for TX Buffer to be empty
*                                 between each character sent. It does NOT place the system in LPM0 mode nor depend on the UART TX interrupts
*          uart_putc()  - This function will write a single character out on the UART buffer. It is a blocking function as it will poll for TX Buffer
*                                 to be empty prior to sending the character. It does NOT place the system in LPM0 mode nor depend on the
*                                 UART TX interrupts
* 
*       Expert users may write some of their own UART handler and utilize:
*          uart_GetChar()     - This function will return the last character received on the UART Buffer. Note that if you don't wait for the UART Flag
*                               this function will continue to return the dame character. Note this also clears the receive flag indicator and UCARXIE. 
*          uart_WaitForChar() - This function will kick off the wait for a character to be received. The UARTRXIE will be set. 
*          uart_GetFlag()     - This function returns the current state of the Receive Flag
* 
*       Error recovery processing is not implemented in this software.
*        
*       April 2013
*
*       \note that the functions in this file are not re-entrant. It is the user's responsibility to assure that these functions
*       are not called until the previous function has completed.
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
#include <msp430.h>
#include "usci_digital_io.h"
#include "usci_uart.h"

static unsigned char rx_char;
static unsigned char UartFlag = 0;

                      // 9600, 19200, 38400, 76800, 115200
static const unsigned int UART_ClockRates[10][5] =
{
        104, 52, 26, 13, 9,                        // 1 MHz
        208, 104, 52, 26, 17,                      // 2 MHz
        416, 208, 104, 52, 35,                     // 4 MHz
        833, 416, 208, 104, 69,                    // 8 MHz
        1250,625, 312, 156, 104,                   // 12 MHz
        1667,832, 416, 208, 139                    // 16 MHz
};

unsigned char UART_Interrupts;

/*************************************************************************************************************************************************
*  pause_UART                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function halts progress of UART send/receive by masking the UART based interrupts.
*
* The function is called to pause the UART base interrupt processing. The UART processing may be continued by calling resume_UART().
*
* @return  None
**************************************************************************************************************************************************/
void pause_UART(void)
{
    UART_Interrupts = IE2 & (UCA0TXIE |UCA0RXIE);
    IE2 &= ~(UCA0RXIE |UCA0TXIE);

}

/*************************************************************************************************************************************************
*  resume_UART                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function re-enables the UART based interrupts 
*
* The function is called to undo the work of pause_UART(). It turns back on the UART interrupts (UCA0RXIE and UCA0TXIE) that were already enabled
* before the pause_UART) function was called.
*
* @return  None
**************************************************************************************************************************************************/
void resume_UART(void)
{
    IE2 |= UART_Interrupts;
}

/*************************************************************************************************************************************************
*  Setup_UART                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Initialize the software structures used in UART communications. Uses USCI A0 pins 
*
* The function is called once to configure the UART hardware settings for UART Communications.
*
* @param[in]   uartPort            Indicates the MSP430 port number containing the UART pins
* @param[in]   tx_pin              Pin number (BIT0 - BIT7) of the SDA pin
* @param[in]   rx_pin              Pin number (BIT0 - BIT7) of the SCL pin
* @param[in]   primary_secondary   Indicates if this is a primary peripheral or secondary (PxSEL2 set) on the MSP430
* @param[in]   smclk_speed         Speed of the MSCLK bus (used to determine bit rate divisors
* @param[in]   uartClkRate         Desired UART Baud Rate / Clock speed (UART_9600_BAUD, UART_19200_BAUD, UART_38400_BAUD,
*                                                                        UART_76800_BAUD or UART_19200_BAUD)
* @return  None
*
* @note This function must be called prior to any UART Write or UART Read functions
**************************************************************************************************************************************************/
void Setup_UART (Port_t uartPort, unsigned char tx_pin, unsigned char rx_pin, primSecPeripheral_t primary_secondary, 
                 smclkSpeed_t smclkSpeed, uartClkRate_t uartClkRate)
{
#if defined __MSP430_HAS_PORT1_R__
   if (uartPort == PORT_1)
   {
       P1SEL  |= tx_pin + rx_pin;           // Assign UART pins to USCI_A0
#if IF_PSxSEL2_PRESENT
       if (primary_secondary == SECONDARY_PERIPHERAL)
       {
           P1SEL2  |= tx_pin + rx_pin;      // Assign I2C pins to Secondary Peripheral of USCI_B0
       }
#endif
   }
#endif
#if defined __MSP430_HAS_PORT2_R__
   if (uartPort == PORT_2)
   {
       P2SEL  |= tx_pin + rx_pin;           // Assign UART pins to USCI_A0
#if IF_PSxSEL2_PRESENT
       if (primary_secondary == SECONDARY_PERIPHERAL)
       {
           P2SEL2  |= tx_pin + rx_pin;      // Assign I2C pins to Secondary Peripheral of USCI_B0
       }
#endif
   }
#endif
#if defined __MSP430_HAS_PORT3_R__
   if (uartPort == PORT_3)
   {
       P3SEL  |= tx_pin + rx_pin;           // Assign UART pins to USCI_A0
#if IF_PSxSEL2_PRESENT
       if (primary_secondary == SECONDARY_PERIPHERAL)
       {
           P3SEL2  |= tx_pin + rx_pin;      // Assign I2C pins to Secondary Peripheral of USCI_B0
       }
#endif
   }
#endif
#if defined __MSP430_HAS_PORT4_R__
   if (uartPort == PORT_4)
   {
       P4SEL  |= tx_pin + rx_pin;           // Assign UART pins to USCI_A0
#if IF_PSxSEL2_PRESENT
       if (primary_secondary == SECONDARY_PERIPHERAL)
       {
           P4SEL2  |= tx_pin + rx_pin;      // Assign I2C pins to Secondary Peripheral of USCI_B0
       }
#endif
   }
#endif

	UCA0CTL1 |= (UCSWRST | UCSSEL_2);   // Use SMCLK and place UART in reset to change clocking
	UCA0BR0 = UART_ClockRates[smclkSpeed][uartClkRate] & 0xff;    
    UCA0BR1 = UART_ClockRates[smclkSpeed][uartClkRate] >> 8;    
	UCA0MCTL = UCBRS0;                  // Modulation UCBRSx = 1
	UCA0CTL1 &= ~UCSWRST;               // **Initialize USCI state machine**
	IE2 |= UCA0RXIE;                    // Enable USCI_A0 RX interrupt
	return;
}

/*************************************************************************************************************************************************
*  uart_getc                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function will place the CPU in Low power mode and wait for a character to arrive on the UART RX line
*
* The function returns the next character received on the UART. This is a blocking function. It will place the system in Low Power mode and stall
* all non-interrupt processing until a character is received on the UART bus.
*
* @return  unsigned char   - character received on UART A0 bus
**************************************************************************************************************************************************/
unsigned char uart_getc(void)				//Waits for a valid char from the UART
{
    UartFlag = 0;
    IE2 |= UCA0RXIE;                       // Make sure UART RX interrupt is enabled

    // Place system to sleep waiting for next UART RX character. 
    while (!UartFlag)
    {
        __bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts; Wait for character to arrive
    }

    return rx_char;
}

/*************************************************************************************************************************************************
*  uart_gets                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function will receive an entire string from the UART RX bus
*
* The function returns an entire UART string until either the maximum number of bytes are received or an end of line character is found.
* This function uses the uart_getc() function, so it will block all non-interrupt processing until the UART string is completely received.
* The length parameter prevents overrun by defining the size of the array used to receive the data. Characters received after the maximum length
* will not be placed into the receive array.
* 
* @note This function will fill the remaining bytes of the receive array with NULL, including overwriting the CRLF characters.
*
* @param[in]   array               Memory to place the received data from the UART RX
* @param[in]   max_length_bytes    Maximum number of bytes in Array to prevent overrun
* 
* @return  None
**************************************************************************************************************************************************/
void uart_gets(char* array, int max_length_bytes)
{
	unsigned int i = 0;
	unsigned short gie = _get_SR_register() & GIE;

	while((i < max_length_bytes))		            //Grab data till the array fills
	{
		array[i] = uart_getc();
		uart_putc(array[i]);
		if(array[i] == '\r')				        //If we receive a \r the master wants to end
		{
			for( ; i < max_length_bytes ; i++)		//fill the rest of the string with \0 nul. Overwrites the \r with \0
			{
				array[i] = '\0';
			}
			break;
		}
		i++;
	}

	if (gie)
	    _bis_SR_register(GIE);

    return;
}

/*************************************************************************************************************************************************
*  uart_putc                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function will place one ASCII character on the UART TX line
*
* The function sends a single character onto the UART TX line. It will wait until the UART status indicates the TX line is empty before sending
* the character.
* 
* @note This function returns when the character is placed into the UART TXBUF. It is actually sent on the line by the hardware sometime after
* this occurs. Therefore, a return from this function does NOT indicate the character has been sent on the hardware line.
* 
* @param[out]   c             Character to send onto the UART TX line
* 
* @return  None
**************************************************************************************************************************************************/
void uart_putc(unsigned char c)
{
	while (!(IFG2&UCA0TXIFG));          // Wait till TX Buffer is empty
    UCA0TXBUF=c;						// TX next character
	return;
}


/*************************************************************************************************************************************************
*  uart_puts                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function will place an ASCII string (ends with a NULL character) on the UART TX line
*
* The function sends an ASCII string onto the UART TX line. It will call the uart_putc() function for each byte until a NULL characetre (\0) is
* found in the string indicating end of string. NOte that this function does not place the system in low power mode between bytes. It will end up
* trying to send each byte and waiting until the previous byte is sent. Therefore, this is not the most power efficient algorithm ever developed
* for UART transmissions.
* 
* @note This function returns when the last character is placed into the UART TXBUF. It may not have actually been sent on the line by the hardware
* when the function returns. Therefore, a return from this function does NOT indicate the character has been sent on the hardware line.
* 
* @param[out]   str             String to send onto the UART TX line
* 
* @return  None
**************************************************************************************************************************************************/
void uart_puts(char *str)				//Sends a String to the UART.
{
    while(*str) 
        uart_putc(*str++);		//Advance though string till end

    // The last byte is now in the TX Fifo. Wait until the UCBUSY bit clears to indicate the last byte has been sent out on the UART bus
    while (UCA0STAT & UCBUSY);

    return;
}

/*************************************************************************************************************************************************
*  uart_putLong                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function will convert an unsigned long integer into ASCII characters and place them on the UART TX line.
*
* The function will convert a number into ASCII characters and place them on the UART TX line. This allows a user to "print" a numerical value.
* As this handles unsigned long integers, it does not deal with fractions or negative numbers.
* 
* @param[out]   number             Unsigned long number to convert into ASCII characters and place on UART TX Line
* 
* @return  None
**************************************************************************************************************************************************/
void uart_putLong (unsigned long number)
{
unsigned char start = 0;
unsigned long target = 1000000000;
unsigned char value;

    while (target > 1)
    {
        if (number >= target)
        {
            start = 1;
            value = number / target;
            uart_putc ('0' + value);
            number = number - (target * value);
        }
        else
        {
            if (start)
                uart_putc ('0');
        }
        target = target / 10;
    }

    uart_putc('0' + number);

    // The last byte is now in the TX Fifo. Wait until the UCBUSY bit clears to indicate the last byte has been sent out on the UART bus
    while (UCA0STAT & UCBUSY);

}

void uartPutHex (unsigned char hexByte)
{
unsigned char target;

    target = hexByte >> 4;
    if (target < 10)
        uart_putc('0' + target);
    else
        uart_putc('A' + (target - 10));

    target = hexByte & 0xf;
    if (target < 10)
        uart_putc('0' + target);
    else
        uart_putc('A' + (target - 10));

    // The last byte is now in the TX Fifo. Wait until the UCBUSY bit clears to indicate the last byte has been sent out on the UART bus
    while (UCA0STAT & UCBUSY);

}

unsigned char uart_GetFlag (void)
{
    return (UartFlag);
}

void uart_WaitForChar(void)               //Waits for a valid char from the UART
{
    UartFlag = 0;
    IE2 |= UCB0RXIE;
}

unsigned char uart_GetChar(void)               //Gets a valid char from the UART already returned from UART
{
    UartFlag = 0;
    IE2 &= ~UCB0RXIE;
    return rx_char;
}

void uart_ClearWaitFlag(void)
{
    UartFlag = 0;
}

unsigned char uart_rx_isr (void)
{
    if ((IE2 & UCA0RXIE) && (IFG2 & UCA0RXIFG))
    {
        UartFlag = 1;
        rx_char = UCA0RXBUF;			          	   //Copy from RX buffer, in doing so we ACK the interrupt as well
        return (WAKE_CPU);
    }

    return (DONT_WAKE_CPU);
}
