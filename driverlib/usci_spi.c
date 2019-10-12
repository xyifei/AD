/*************************************************************************************************************************************************/
/*!     USCI I2C Code
*
*       This code is designed for I2C blocks on the MSP430 using an USCI hardware module.
* 
*       The code was desigmed on MSP430 that do not have the UCB0IV, Interrupt Vector, registers for the I2C module. It also was designed for
*       USCI blocks that have a single I2c Own Address register. Newer MSP430s exist will multiple I2C Own Addresses with individual interrupt
*       vectors for each.
* 
*       The code is specifically written to use UCB0 I2C functions the user can indicate the location of the I2C pins in Setup_I2C()
*       The code only supports Master communications (Reads and Writes). There is no support currently present in this software for Slave or
*       Multi-Master systems.
* 
*       Standard functions that will be used by implementors are:
*          Setup_I2C() - This function is called once to configure the I2C hardware
*          Write_I2C() - This function will perform an I2C write to the I2c address indicated, sending the bytes provided
*          Read_I2C()  - This function will perform a dummy I2C write of 1 byte, followed by a RESTART and
*                                     then read the number of bytes indicated into the buffer provided
*          Read_I2C_With_Stop() - This function is the same as Read_I2C, except that a STOP/START occurs between the dummy write and read,
*                                     rather than a RESTART
* 
*       Note that the Read_I2C() is specifically written to support a 1 byte dummy write. This is intended for a register offset. For I2C devices
*       that use a 2 byte register offset (or any other different dummy write), changes would be needed to the code to support it.
* 
*       The software is written to place the CPU into CPUOFF mode when executing the I2C functionality. All processing occurs in I2C interrupts.
*       At completion of an I2C functionality, the processor will exit CPUOFF mode to continue processing.
* 
*       The i2c_ISR() function is included which must be tied to the USCIAB0TX_VECTOR interrupt vector. Since this interrupt can be shared with
*       other (UART) USCI blocks, the implementation is left to a higher level program (often main.c). The i2c_ISR() function will return an
*       indicator that the processor should exit low power mode upon exit of the interrupt code.
* 
*               if (i2c_ISR())
*                   __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
* 
*       Error recovery processing is not implemented in this software. The Write_I2C(), and Read_I2C*() functions will return an error if the I2C
*       bus is already in use or a NAK is received. (I2C_FAIL_NACK)
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
#include "usci_spi.h"



volatile unsigned char *UCxxCTL0;
volatile unsigned char *UCxxCTL1;
volatile unsigned char *UCxxBR0;
volatile unsigned char *UCxxBR1;
volatile unsigned char *UCxxSTAT;
volatile unsigned char *UCxxRXBUF;
volatile unsigned char *UCxxTXBUF;
unsigned char SPIRXIFG;
unsigned char SPITXIFG;
static unsigned char SpiInUseFlag = 0;
static unsigned char SpiRegisterIndex = 0;
static SpiRegisterStore_type   SpiRegisterStore[10] = {{(volatile unsigned char *)0xff, 0xff}, {(volatile unsigned char *)0xff, 0xff}, 
                                                       {(volatile unsigned char *)0xff, 0xff}, {(volatile unsigned char *)0xff, 0xff}, 
                                                       {(volatile unsigned char *)0xff, 0xff}, {(volatile unsigned char *)0xff, 0xff}, 
                                                       {(volatile unsigned char *)0xff, 0xff}, {(volatile unsigned char *)0xff, 0xff}, 
                                                       {(volatile unsigned char *)0xff, 0xff}, {(volatile unsigned char *)0xff, 0xff}};


static const unsigned char SPI_ClockRates[10][4] =         // SPI CLK 0f 1, 2, 4, and 8MHz
{
        0x01, 0x00, 0x00, 0x00,            // 1 MHz SMCLK
        0x02, 0x01, 0x00, 0x00,            // 2 MHz SMCLK
        0x04, 0x02, 0x01, 0x00,            // 4 MHz
        0x08, 0x04, 0x02, 0x01,            // 8 MHz
        0x0c, 0x06, 0x03, 0x00,            // 12 MHz
        0x10, 0x08, 0x04, 0x02             // 16 MHz
};


/*************************************************************************************************************************************************
*  Setup_SPI                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Initialize the software structures used in SPI communications. Uses USCI A0 or B0 
*
* The function is called once to configure the SPI hardware settings for SPI Master Communications.
* As a system that has multiple devices on the SPI bus will configure independent, there will be independent GPIO pins used as STE for each
* separate devices. 
* 
* The RegisterSpiDevice() is used to identify the pin used for SPI STE (CS), as many systems use different GPIO to configure multiple SPI devices
* on a single bus. Each different end device uses a different GPIO pin as STE (CS).
* 
* @param[in]   spiPort             Indicates the MSP430 port number containing the SPI pins (SIMO, SOMI and SCLK)
* @param[in]   simo_pin            Pin number (BIT0 - BIT7) of the SPI SIMO (MOSI) pin
* @param[in]   somi_pin            Pin number (BIT0 - BIT7) of the SPI SOMI (MISO) pin
* @param[in]   sclk_pin            Pin number (BIT0 - BIT7) of the SPI SCLK pin
* @param[in]   primary_secondary   Indicates if this is a primary peripheral or secondary (PxSEL2 set) on the MSP430
* @param[in]   a_b_side            Indicates whether we are using the A0 or B0 Spi USCI
* @param[in]   smclk_speed         Speed of the MSCLK bus (used to determine bit rate divisors
* @param[in]   spiClkRate          Desired SPI Clock speed (SPI_1_MHZ, SPI_2_MHZ, SPI_4_MHZ, or SPI_8_MHZ)
* @param[in]   polarity            Identifies the STE (CS) setting when bus is inactive (SPI_INACTIVE_LOW_POL or SPI_INACTIVE_HIGH_POL )
* @param[in]   phase               Indicates hich edge will be used to capture data (SPI_CHANGE_ON_FIRST_EDGE or SPI_CAPT_ON_FIRST_EDGE)
* 
* @return  error indicator         SPI_SUCCESS or SPI_INVALID_RATE
*
* @note This function must be called prior to any SPI_Write or SPI_Read functions.
* @note The RegisterSpiDevice() function must be called for each SPI device on the bus to complete SPI initialization.
**************************************************************************************************************************************************/
unsigned char Setup_SPI_Master (Port_t spiPort, unsigned char somi_pin, unsigned char simo_pin, unsigned char sclk_pin,
                                primSecPeripheral_t primary_secondary, usci_a_b_side_t a_b_side,
                                smclkSpeed_t smclkSpeed, spiClkRate_t spiClkRate,
                                spiPolarity_t polarity, spiClkPhase_t phase)
{

    if (SPI_ClockRates[smclkSpeed][spiClkRate] == 0 )
        return (SPI_INVALID_RATE);

#if defined __MSP430_HAS_PORT1_R__
    if (spiPort == PORT_1)
    {
        P1SEL  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Primary Peripheral of USCI_B0
#if IF_PSxSEL2_PRESENT
        if (primary_secondary == SECONDARY_PERIPHERAL)
        {
            P1SEL2  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Secondary Peripheral of USCI_B0
        }
#endif
    }
#endif
#if defined __MSP430_HAS_PORT2_R__
    if (spiPort == PORT_2)
    {
        P2SEL  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Primary Peripheral of USCI_B0
#if IF_PSxSEL2_PRESENT
        if (primary_secondary == SECONDARY_PERIPHERAL)
        {
            P2SEL2  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Secondary Peripheral of USCI_B0
        }
#endif
    }
#endif
#if defined __MSP430_HAS_PORT3_R__
    if (spiPort == PORT_3)
    {
        P3SEL  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Primary Peripheral of USCI_B0
#if IF_PSxSEL2_PRESENT
        if (primary_secondary == SECONDARY_PERIPHERAL)
        {
            P3SEL2  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Secondary Peripheral of USCI_B0
        }
#endif
    }
#endif
#if defined __MSP430_HAS_PORT4_R__
    if (spiPort == PORT_4)
    {
        P4SEL  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Primary Peripheral of USCI_B0
#if IF_PSxSEL2_PRESENT
        if (primary_secondary == SECONDARY_PERIPHERAL)
        {
            P4SEL2  |= somi_pin + simo_pin + sclk_pin;  // Assign SPI pins to Secondary Peripheral of USCI_B0
        }
#endif
    }
#endif

    if (a_b_side == USCI_A_SIDE)
    {
        UCxxCTL0  = &UCA0CTL0;
        UCxxCTL1  = &UCA0CTL1;
        UCxxBR0   = &UCA0BR0;
        UCxxBR1   = &UCA0BR1;
        UCxxSTAT  = &UCA0STAT;
        UCxxRXBUF = &UCA0RXBUF;
        UCxxTXBUF = &UCA0TXBUF;
        SPIRXIFG  = UCA0RXIFG;
        SPITXIFG  = UCA0TXIFG;
    }
    else
    {
        UCxxCTL0  = &UCB0CTL0;
        UCxxCTL1  = &UCB0CTL1;
        UCxxBR0   = &UCB0BR0;
        UCxxBR1   = &UCB0BR1;
        UCxxSTAT  = &UCB0STAT;
        UCxxRXBUF = &UCB0RXBUF;
        UCxxTXBUF = &UCB0TXBUF;
        SPIRXIFG  = UCB0RXIFG;
        SPITXIFG  = UCB0TXIFG;
    }


    *UCxxCTL1  |= UCSWRST;                            // Enable SW reset
    *UCxxCTL0 = UCMST + UCMODE_0 + UCMSB + UCSYNC + polarity + phase;     // SPI Master, 3 wire, synchronous mode
    *UCxxCTL1 = UCSSEL_2 + UCSWRST;                    // Use SMCLK, keep SW reset
    *UCxxBR0 = SPI_ClockRates[smclkSpeed][spiClkRate];
    *UCxxBR1 = 0;
    *UCxxCTL1 &= ~UCSWRST;                     // Clear SW reset, resume operation
    SpiInUseFlag = 0;

    return SPI_SUCCESS;

}

/*************************************************************************************************************************************************
*  Change_SPI_Polarity                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief For systems that need to change the SPI polarity to support multiple SPI devices on a shared bus requiring different polarities.
*
* The function is called to modify the polarity of the SPI bus.
* 
* @param[in]   polarity            Identifies the STE (CS) setting when bus is inactive (SPI_INACTIVE_LOW_POL or SPI_INACTIVE_HIGH_POL )
* 
* @return  none
**************************************************************************************************************************************************/
void Change_SPI_Polarity (spiPolarity_t polarity)
{
    *UCxxCTL0 &= ~UCCKPL;    // Clear previous polarity setting
    *UCxxCTL0 |=  polarity;     // Add new phase
}

/*************************************************************************************************************************************************
*  Change_SPI_Phase                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief For systems that need to change the SPI phase to support multiple SPI devices on a shared bus requiring different phases.
*
* The function is called to modify the phase of the SPI bus.
* 
* @param[in]   phase               Indicates hich edge will be used to capture data (SPI_CHANGE_ON_FIRST_EDGE or SPI_CAPT_ON_FIRST_EDGE)
* 
* @return  none
**************************************************************************************************************************************************/
void Change_SPI_Phase (spiClkPhase_t phase)
{
    *UCxxCTL0 &= ~UCCKPH;    // Clear previous phase setting
    *UCxxCTL0 |=  phase;     // Add new phase
}

/*************************************************************************************************************************************************
*  RegisterSpiDevice                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Identify the SPI STE (CS) line used for a given SPI device on the bus.
*
* The function is called once for each SPI device to identify the STE (CS) line that will be used to activate that SPI device for commmunications.
* 
* As a system that has multiple devices on the SPI bus will configure independent, there will be independent GPIO pins used as STE for each
* separate devices. 
* 
* The RegisterSpiDevice() is used to identify the pin used for SPI STE (CS), as many systems use different GPIO to configure multiple SPI devices
* on a single bus. Each different end device uses a different GPIO pin as STE (CS).
* 
* The function will return a handle that is used in subsequent SPI function read/write calls that will manage which SPI device is being accessed.
* 
* @param[in]   ste_Port             Indicates the MSP430 port number containing the SPI STE pin for this SPI device
* @param[in]   ste_pin              Pin number (BIT0 - BIT7) of the SPI STE pin for this SPI device
* 
* @return  handle         This value is used in subsequent SPI communication functions to indicate the specific SPI device.
*                         A 0 value indicates an error (too many devices)
*
* @note This function must be called prior to any SPI_Write or SPI_Read functions.
**************************************************************************************************************************************************/
unsigned char RegisterSpiDevice (Port_t ste_port, unsigned char ste_pin)
{
    if (++SpiRegisterIndex > 10)
        return (0);

#if defined __MSP430_HAS_PORT1_R__
    if (ste_port == PORT_1)
    {
        P1OUT |= ste_pin;                                     // Active Low, so set high
        P1DIR |= ste_pin;
        SpiRegisterStore[SpiRegisterIndex].PortPtr = &P1OUT;
        SpiRegisterStore[SpiRegisterIndex].bitNum  = ste_pin;
        return (SpiRegisterIndex);
    }
#endif

#if defined __MSP430_HAS_PORT2_R__
    if (ste_port == PORT_2)
    {
        P2OUT |= ste_pin;                                     // Active Low, so set high
        P2DIR |= ste_pin;
        SpiRegisterStore[SpiRegisterIndex].PortPtr = &P2OUT;
        SpiRegisterStore[SpiRegisterIndex].bitNum  = ste_pin;
        return (SpiRegisterIndex);
    }
#endif

#if defined __MSP430_HAS_PORT3_R__
    if (ste_port == PORT_3)
    {
        P3OUT |= ste_pin;                                     // Active Low, so set high
        P3DIR |= ste_pin;
        SpiRegisterStore[SpiRegisterIndex].PortPtr = &P3OUT;
        SpiRegisterStore[SpiRegisterIndex].bitNum  = ste_pin;
        return (SpiRegisterIndex);
    }
#endif

#if defined __MSP430_HAS_PORT4_R__
    if (ste_port == PORT_4)
    {
        P4OUT |= ste_pin;                                     // Active Low, so set high
        P4DIR |= ste_pin;
        SpiRegisterStore[SpiRegisterIndex].PortPtr = &P4OUT;
        SpiRegisterStore[SpiRegisterIndex].bitNum  = ste_pin;
        return (SpiRegisterIndex);
    }
#endif

    return (0);
}

/*************************************************************************************************************************************************
*  SPI_Write                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief Perform a SPI communication sequence to Write/Read data on SPI bus
*
* The SPI bus uses one clock line and 2 data lines that transfer data at the same time. The master will read data from the MISO
* (Master In Slave Out) line on one edge of the clock. The slave device will read on the MOSI (Master Out Slave In) on the other edge of the clock.
* As such, reads and writes happen together. If there is no data to send to the slave, the master must write out dummy data on the bus to get the
* clock to toggle for the slave device to communciate.
* 
* Note that many SPI devices do not communicate back to the master at all, and have no MISO line connected. 
* 
* @param[in]   spiHandle            Indicates the SPI device we will communicate with. Value from RegisterSpiDevice()
* @param[in]   outData              Pointer to an array with the data that will be written on the SPI bus
* @param[in]   inData               Pointer to an array to place the data returned from the slave device. One byte for EVERY byte sent,
*                                       even the command byte
* @param[in]   length               Number of bytes to send/receive 
* 
* @return  error status   SPI_INVALID_COMMAND or SPI_SUCCESS
**************************************************************************************************************************************************/
unsigned char SPI_Write (unsigned char spiHandle, unsigned char *outData, unsigned char *inData, unsigned char length)
{
unsigned char i;

    if ((spiHandle < 1) || (spiHandle > 10) || (SpiRegisterStore[spiHandle].bitNum == 0xff))
        return (SPI_INVALID_COMMAND);

    if (SpiInUseFlag)
        return (SPI_IN_USE);
    else
        SpiInUseFlag = 1;

    // CS Active Low
    *SpiRegisterStore[spiHandle].PortPtr &= ~SpiRegisterStore[spiHandle].bitNum;
    __delay_cycles(20);          // Pause a bit

    for (i=0; i<length; i++)
    {
        inData[i] = SPI_Write_Byte (outData[i]);
__delay_cycles(20);          // Pause a bit
    }

    *SpiRegisterStore[spiHandle].PortPtr |= SpiRegisterStore[spiHandle].bitNum;

    SpiInUseFlag = 0;

    return (SPI_SUCCESS);

}

/*************************************************************************************************************************************************
*  SPI_Write_Byte                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function is used to place a single byte on the SPI bus. This function is for expert mode only.
*
* This function can be used by someone managing their own communications on the SPI bus to place one byte on the bus. It is assumed that the user
* will manage the STE pin themselves when using this command. The received byte from the MISO pin will be returned by this function.
* 
* This function only exists to support users who have non-standard SPI devices on their bus. Some devices require excessive time between bytes or
* a delay between the command byte and subsequent data bytes.
* 
* @param[in]   data              Pointer to an array with the data that will be written on the SPI bus
* 
* @return  in_data               Character sent from SPI device on MISO line
**************************************************************************************************************************************************/
unsigned char SPI_Write_Byte (unsigned char data)
{

    IFG2 &= ~SPIRXIFG;             // Clear RX Interrupt flag

    *UCxxTXBUF = data;              // Place data in TX Buffer
    while (!(IFG2 & SPIRXIFG));    // Wait for end of data receive
    return ((unsigned char)*UCxxRXBUF);            // Return the received byte from RX Buffer
}

/*************************************************************************************************************************************************
*  SPI_Write_Shutdown_Byte                                                                                                                         
**************************************************************************************************************************************************/
/*!
* @brief This function is used to place a single byte on the SPI bus with NO return byte. This function is for expert mode only.
*
* This function can be used by someone managing their own communications on the SPI bus to place one byte on the bus. It is assumed that the user
* will manage the STE pin themselves when using this command. The received byte from the MISO pin will be returned by this function.
* 
* This function only exists to support users who have non-standard SPI devices on their bus. Some devices require excessive time between bytes or
* a delay between the command byte and subsequent data bytes.
* 
* There are some devices that will not return a valid byte on a shutdown command. Ths function does not wait for, nor return, a read byte
* 
* @param[in]   data              Pointer to an array with the data that will be written on the SPI bus
* 
* @return  none
**************************************************************************************************************************************************/
void SPI_Write_Shutdown_Byte (unsigned char data)
{

    IFG2 &= ~SPIRXIFG;
    *UCxxTXBUF = data;
}


