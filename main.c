#include <msp430.h> 
#include "stdint.h"
#include "ADS1220.h"
#include "usci_spi.h"
#include "TempTransmitter.h"
#include "usci_uart.h"

unsigned char ReadConversionData = 0;
extern long OffsetCalibrateValue;

void Setup_IO(void)
{
    P1DIR |= ADS_CS_N;
    P1OUT |= ADS_CS_N;

    P2DIR &= ~ADS_DRDY_N;
    P2IES |= ADS_DRDY_N;
    P2IFG &= ~ADS_DRDY_N;
    P2IE |= ADS_DRDY_N;
}

int main(void)
{
    volatile static unsigned char tempData[3];

	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer
	
	BCSCTL1 = CALBC1_1MHZ;              // BCSCTL1 Calibration Data for 1MHz
    DCOCTL = CALDCO_1MHZ;               // DCOCTL Calibration Data for 1MHz
    BCSCTL2 &= ~SELS;                   // Select DCOCLK as SMCLK source

	Setup_IO();

	__enable_interrupt();

	Setup_SPI_Master (SPI_PORT, ADS_DAC_DOUT, ADS_DAC_DIN, ADS_DAC_CLK,
                      SECONDARY_PERIPHERAL, USCI_B_SIDE, SMCLK_1_MHZ, SPI_1_MHZ,
                      SPI_INACTIVE_LOW_POL, SPI_CHANGE_ON_FIRST_EDGE);

	Setup_ADS1220_CS (ADS_CS_PORT, ADS_CS_N);

	Setup_UART(UART_PORT, UART_TX_PIN, UART_RX_PIN, SECONDARY_PERIPHERAL, SMCLK_1_MHZ, UART_9600_BAUD);

	ADS1220_Reset();

	Setup_ADS1220 (ADS1220_MUX_SHORTED, ADS1220_OP_MODE_NORMAL,
                   ADS1220_CONVERSION_SINGLE_SHOT, ADS1220_DATA_RATE_20SPS, ADS1220_GAIN_16, ADS1220_USE_PGA,
                   ADS1220_IDAC1_AIN3, ADS1220_IDAC2_AIN2, ADS1220_IDAC_CURRENT_250_UA);

//	Setup_ADS1220 (ADS1220_MUX_AIN0_AIN1, ADS1220_OP_MODE_NORMAL,
//                   ADS1220_CONVERSION_CONTINUOUS, ADS1220_DATA_RATE_20SPS, ADS1220_GAIN_16, ADS1220_USE_PGA,
//                   ADS1220_IDAC1_AIN3, ADS1220_IDAC2_AIN2, ADS1220_IDAC_CURRENT_250_UA);

	ReadConversionData = 0;
    ADS1220_Start();

//    while (calibrateCount < 8)
//    {
//        while (!ReadConversionData);   // Wait for Data Ready interrupt
//        ReadConversionData = 0;
//        ADS1220_Get_Conversion_Data ((unsigned char *)tempData);   // Get the raw data
//        ADS1220_Offset_Calibrate_Data ((unsigned char *)tempData);        // Send results to calibration function
//        calibrateCount++;
//        uart_puts("ADSuccess");
//
//        // Start next calibration reading?
//        if (calibrateCount < 8)
//            ADS1220_Start();
//    }

    while(1)
    {
        if (ReadConversionData)
        {
            ReadConversionData = 0;

            ADS1220_Get_Conversion_Data((unsigned char *)tempData);

            ADS1220_Start();
        }
    }

}

#pragma vector = PORT2_VECTOR
__interrupt void PORT2_ISR (void)
{
unsigned char interrupts = P2IFG;

    P2IFG = 0;

    // Has a Data Ready occurred from the ADS1220?
    if (interrupts & ADS_DRDY_N)
    {
        ReadConversionData = 1;
    }

    __bic_SR_register_on_exit(CPUOFF);      // Exit LPM0

}
