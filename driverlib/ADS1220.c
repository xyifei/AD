/*************************************************************************************************************************************************/
/*!     ADS1220.c
*
*       This code is designed to perform standard command and control operations on the ADS1220 over a SPI bus. Functions exist to setup, configure,
*       and read conversion data from the ADS1220.
*
*       The software is specifically written to execute on an MSP430G2413 on the SATxxxx board.
*
*       October 2013
*
*       \note that the functions in this file are not re-entrant. It is the user's responsibility to assure that these functions
*       are not called until the previous function has completed.
*/
/**************************************************************************************************************************************************
*       Copyright � 2013 Texas Instruments Incorporated - http://www.ti.com/                                                                      *
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
#include "usci_spi.h"
#include "ADS1220.h"


static unsigned char SpiHandle;
static unsigned char RcvData[8];

static long OffsetCalibrateData = 0;
static unsigned char OffsetCalibrateCount = 0;
static long OffsetCalibrateValue = 0;

float PgaGainLevel;

long GainCalibrateData = 0;
unsigned char GainCalibrateCount = 0;
long GainCalibrateValue = 0;

long RawVoltage;
long CalVoltage;
long GainVoltage;

extern float FlashGainCorrection;

/*************************************************************************************************************************************************
*  Setup_ADS1220
**************************************************************************************************************************************************/
/*!
* @brief Performs the setup of the ADS1220.
*
* This function will configure the ADS1220.
*
* @param[in]     ste_port        The MSP430 port that contains the CS or STE pin that is connected to the ADS1220. (PORT_1, PORT_2, ...)
* @param[in]     ste_pin         The MSP430 pin that contains the CS or STE pin that is connected to the ADS1220.  (BIT0, BIT1, ...)
* @param[in]     inputMux        Mux settings for the AIN pins (ADS1220_MUX_AIN0_AIN1, ADS1220_MUX_AIN0_AIN2, ADS1220_MUX_AIN0_AIN3,
*                                                               ADS1220_MUX_AIN1_AIN2, ADS1220_MUX_AIN1_AIN3, ADS1220_MUX_AIN2_AIN3,
*                                                               ADS1220_MUX_AIN1_AIN0, ADS1220_MUX_AIN3_AIN2, ADS1220_MUX_AIN0_AVSS,
*                                                               ADS1220_MUX_AIN1_AVSS, ADS1220_MUX_AIN2_AVSS, ADS1220_MUX_AIN3_AVSS,
*                                                               ADS1220_MUX_REFP-REFN, ADS1220_MUX_AVDD-AVSS, ADS1220_MUX_SHORTED)
* @param[in]     opMode          ADS1220 operating mode (ADS1220_OP_MODE_NORMAL, ADS1220_OP_MODE_DUTY, ADS1220_OP_MODE_TURBO)
* @param[in]     conversionMode  Identifies single shot or continuous conversions (ADS1220_CONVERSION_SINGLE_SHOT or ADS1220_CONVERSION_CONTINUOUS)
* @param[in]     dataRate        ADS1220 Data Rate (ADS1220_DATA_RATE_20SPS, ADS1220_DATA_RATE_45SPS, ADS1220_DATA_RATE_90SPS,
*                                                   ADS1220_DATA_RATE_175SPS, ADS1220_DATA_RATE_330SPS, ADS1220_DATA_RATE_600SPS, or
*                                                   ADS1220_DATA_RATE_1000SPS)
* @param[in]     gainLevel       ADS1220 Gain Level (ADS1220_GAIN_1, ADS1220_GAIN_2, ADS1220_GAIN_4, ADS1220_GAIN_8, ADS1220_GAIN_16,
*                                                    ADS1220_GAIN_32, ADS1220_GAIN_64, or ADS1220_GAIN_128)
* @param[in]     pgaBypass       Bypass PGA for gain settings - Note the PGA will be used whenever the gain setting is ADS1220_GAIN_8 or greater,
*                                no matter what this value is set to. (ADS1220_PGA_BYPASS or ADS1220_USE_PGA)
* @param[in]     routeIDAC1      The output pin used by IDAC1 (ADS1220_IDAC1_DISABLED, ADS1220_IDAC1_AIN0, ADS1220_IDAC1_AIN1, ADS1220_IDAC1_AIN2,
*                                                              ADS1220_IDAC1_AIN3, ADS1220_IDAC1_REFP, or ADS1220_IDAC1_REFN
* @param[in]     routeIDAC2      The output pin used by IDAC2 (ADS1220_IDAC2_DISABLED, ADS1220_IDAC2_AIN0, ADS1220_IDAC2_AIN1, ADS1220_IDAC2_AIN2,
*                                                              ADS1220_IDAC2_AIN3, ADS1220_IDAC2_REFP, or ADS1220_IDAC2_REFN
* @param[in]     idacCurrent     The IDAC Biasing Current (ADS1220_IDAC_CURRENT_OFF, ADS1220_IDAC_CURRENT_10_UA, ADS1220_IDAC_CURRENT_50_UA,
*                                                          ADS1220_IDAC_CURRENT_100_UA, ADS1220_IDAC_CURRENT_250_UA, ADS1220_IDAC_CURRENT_500_UA,
*                                                          ADS1220_IDAC_CURRENT_1000_UA, ADS1220_IDAC_CURRENT_1500_UA
*
* @return  None
*
* @sa Setup_ADS1220_CS()
**************************************************************************************************************************************************/
void Setup_ADS1220 (unsigned char inputMux, unsigned char opMode,
                    unsigned char conversionMode, unsigned char dataRate, unsigned char gainLevel, unsigned char pgaBypass,
                    unsigned char routeIDAC1, unsigned char routeIDAC2, unsigned char idacCurrent)
{
unsigned char config[4];

    config[0] = inputMux + gainLevel + pgaBypass;
    config[1] = dataRate + opMode + conversionMode + ADS1220_TEMP_SENSOR_OFF + ADS1220_BURN_OUT_CURRENT_OFF;
    config[2] = ADS1220_FIR_50_60 + ADS1220_VREF_EXT_REF0_PINS + ADS1220_LOW_SIDE_POWER_OPEN + idacCurrent;
    config[3] = routeIDAC1 + routeIDAC2 + ADS1220_DRDY_ON_DRDY_ONLY;

    switch (gainLevel)
    {
        case ADS1220_GAIN_1:
            PgaGainLevel = 1.0;
            break;
        case ADS1220_GAIN_2:
            PgaGainLevel = 2.0;
            break;
        case ADS1220_GAIN_4:
            PgaGainLevel = 4.0;
            break;
        case ADS1220_GAIN_8:
            PgaGainLevel = 8.0;
            break;
        case ADS1220_GAIN_16:
            PgaGainLevel = 16.0;
            break;
        case ADS1220_GAIN_32:
            PgaGainLevel = 32.0;
            break;
        case ADS1220_GAIN_64:
            PgaGainLevel = 64.0;
            break;
        case ADS1220_GAIN_128:
            PgaGainLevel = 128.0;
            break;

    }
    ADS1220_Write_Regs (config, ADS1220_CONFIG_0_REG, 4);
}

/*************************************************************************************************************************************************
*  Setup_ADS1220_CS
**************************************************************************************************************************************************/
/*!
* @brief Identifies the GPIO pin that will be used for CS on the SPI Bus
*
* This function will configure the CS pin on the SPI bus for the ADS1220.
*
* @param[in]     ste_port        The MSP430 port that contains the CS or STE pin that is connected to the ADS1220. (PORT_1, PORT_2, ...)
* @param[in]     ste_pin         The MSP430 pin that contains the CS or STE pin that is connected to the ADS1220.  (BIT0, BIT1, ...)
*
* @return  None
*
* @sa Setup_ADS1220()
**************************************************************************************************************************************************/
void Setup_ADS1220_CS (Port_t ste_port, unsigned char ste_pin)
{

    SpiHandle = RegisterSpiDevice (ste_port, ste_pin);
}

/*************************************************************************************************************************************************
*  ADS1220_Offset_Calibrate_Data
**************************************************************************************************************************************************/
/*!
* @brief Performs a calibration step from data collected by the ADS1220.
*
* In order to perform a calibration, the ADS1220 AIN lines are shorted together (using the Setup_ADS1220() function). A number of conversion are
* then performed and the conversion data is sent to this function.
*
* This function will take the conversion data and average the results which will be used in the ADS1220_Get_Conversion_Data_Calibrated() function
* to return a calibrated conversion result.
*
* @param[in]    *tempData        Pointer to raw conversion data that will be included in calibration calculations.
*
* @return  None
*
* @sa Setup_ADS1220()
* @sa ADS1220_Get_Conversion_Data_Calibrated()
**************************************************************************************************************************************************/
void ADS1220_Offset_Calibrate_Data (unsigned char *tempData)
{
    long temp;

    temp = tempData[0];
    temp <<= 8;
    temp |= tempData[1];
    temp <<= 8;
    temp |= tempData[2];

    // Was temp negative?
    if (tempData[0] & 0x80)
        temp |= ((long)0xff << 24);              // Sign extend

    OffsetCalibrateData += temp;
    OffsetCalibrateCount++;
    OffsetCalibrateValue = OffsetCalibrateData / OffsetCalibrateCount;

}

void ADS1220_Gain_Calibrate_Data (unsigned char *tempData)
{
    long temp;

    temp = tempData[0];
    temp <<= 8;
    temp |= tempData[1];
    temp <<= 8;
    temp |= tempData[2];

    // Was temp negative?
    if (tempData[0] & 0x80)
        temp |= ((long)0xff << 24);              // Sign extend

    GainCalibrateData += temp;
    GainCalibrateCount++;
    GainCalibrateValue = GainCalibrateData / GainCalibrateCount;


    // To calculate fraction, take the expected answer from the precision multimeter reading and
    //   Code = RTDmeas * PgaGainLevel * (2^23 - 1) / (Rref * 2)

}

/*************************************************************************************************************************************************
*  ADS1220_Reset
**************************************************************************************************************************************************/
/*!
* @brief Sends a Reset Command to the ADS1220.
*
* This function sends a Reset command to the ADS1220 on the SPI bus.
*
* @return  None
*
**************************************************************************************************************************************************/
void ADS1220_Reset (void)
{
    unsigned char cmd = ADS1220_RESET_CMD;
    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, &cmd, RcvData, 1);
}

/*************************************************************************************************************************************************
*  ADS1220_Start
**************************************************************************************************************************************************/
/*!
* @brief Sends a Start Conversion Command to the ADS1220.
*
* This function sends a Start Conversion command to the ADS1220 on the SPI bus. Conversions will be completed when the Data Ready interrupt occurs.
*
* @return  None
*
**************************************************************************************************************************************************/
void ADS1220_Start (void)
{
    unsigned char cmd = ADS1220_START_CMD;
    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, &cmd, RcvData, 1);
}

/*************************************************************************************************************************************************
*  ADS1220_Powerdown
**************************************************************************************************************************************************/
/*!
* @brief Sends a Power Down Data Command to the ADS1220.
*
* This function sends a Power Down command to the ADS1220 on the SPI bus.
*
* @return  None
*
* @note A Start Conversion command is required to bring the ADS1220 out of Power Down. Since the Power Down turns off the IDACs, care must be
*       taken to allow them sufficient time to settle after waking up from a power down. Depending on the specific system, it is possible that
*       initial readings after a power down may not be valid.
*
* @sa ADS1220_Start()
**************************************************************************************************************************************************/
void ADS1220_Powerdown (void)
{
    unsigned char cmd = ADS1220_POWERDOWN_CMD;
    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, &cmd, RcvData, 1);
}

/*************************************************************************************************************************************************
*  ADS1220_Write_Regs
**************************************************************************************************************************************************/
/*!
* @brief Writes registers on the ADS1220.
*
* This function will execute a write register command to the ADS1220. This function can be used to update one or more registers on the ADS1220.
* No error checking is performed, so it is the user's responsibility to make sure they do not attempt to write past the end of the ADS1220 registers.
*
* @param[out]   *writeValues    Pointer to the list of 8 bit register values to place in the ADS1220
* @param[in]     startReg       Address of the first register to write
* @param[in]     length         Number of registers to write.
*
* @return  None
*
**************************************************************************************************************************************************/
void ADS1220_Write_Regs (unsigned char *writeValues, unsigned char startReg, unsigned char length)
{
    unsigned char outData[5];
    unsigned char i;

    outData[0] = ADS1220_WRITE_CMD(startReg,length);

    for (i=0; i<length; i++)
    {
        outData[i+1] = writeValues[i];
    }

    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, outData, RcvData, length+1);    // Add 1 to length for command byte

}

/*************************************************************************************************************************************************
*  ADS1220_Read_Regs
**************************************************************************************************************************************************/
/*!
* @brief Reads registers on the ADS1220.
*
* This function will execute a read register command to the ADS1220 and return the resultant data. This function can be used to read one or more
* registers from the ADS1220. No error checking is performed, so it is the user's responsibility to make sure they do not attempt to read past
* the end of the ADS1220 registers.
*
* @param[out]   *readValues     Pointer to place the 8 bit register values from the ADS1220
* @param[in]     startReg       Address of the first register to read
* @param[in]     length         Number of registers to read.
*
* @return  None
*
**************************************************************************************************************************************************/
void ADS1220_Read_Regs (unsigned char *readValues, unsigned char startReg, unsigned char length)
{
    unsigned char outData[5] = {0x55, 0x55, 0x55, 0x55, 0x55};

    outData[0] = ADS1220_READ_CMD(startReg,length);

    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, outData, readValues, length+1);    // Add 1 to length for command byte

}

/*************************************************************************************************************************************************
*  ADS1220_Send_Read_Data_Command
**************************************************************************************************************************************************/
/*!
* @brief Sends a Read Data Command to the ADS1220.
*
* This function sends a Read Data (RDATA) command to the ADS1220 on the SPI bus.
*
* @return  None
*
**************************************************************************************************************************************************/
void ADS1220_Send_Read_Data_Command (void)
{
    unsigned char cmd = ADS1220_RDATA_CMD;
    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, &cmd, RcvData, 1);
}


/*************************************************************************************************************************************************
*  ADS1220_Get_Conversion_Data
**************************************************************************************************************************************************/
/*!
* @brief Gets the raw conversion data from the ADS1220.
*
* This function gets the Conversion Data from the ADS1220. This function is the standard function used to gather the conversion data during
* calibration operations.
*
* @param[out]   *conversionData     Pointer to place the 24 bit Conversion Data from the ADS1220
*
* @return  None
*
* @note The ADS1220_Get_Conversion_Data_Calibrated() function is used to return calibrated conversion data, which is used during normal RTD logic
*
* @sa ADS1220_Get_Conversion_Data_Calibrated()
**************************************************************************************************************************************************/
void ADS1220_Get_Conversion_Data (unsigned char *conversionData)
{

    unsigned char outData[3] = {0xff, 0xff, 0xff};

    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, outData, conversionData, 3);    // 3 Bytes of Conversion Data

}

/*************************************************************************************************************************************************
*  ADS1220_Get_Conversion_Data_Calibrated
**************************************************************************************************************************************************/
/*!
* @brief Gets the conversion data from the ADS1220 and subtracts the calibration values to return a calibrated conversion data.
*
* This function gets the Conversion Data from the ADS1220. It will then subtract the Calibration Value previously calculated. This function
* is the standard function used to gather the conversion data during normal operations.
*
* The function requires the system to have performed calibration, otherwise the calibration value will be 0.
*
* @param[out]   *conversionData     Pointer to place the 24 bit Calibrated Conversion Data from the ADS1220
*
* @return  None
*
* @note The ADS1220_Get_Conversion_Data() function is used to return raw conversion data, which is used during calibration logic
*
* @sa ADS1220_Get_Conversion_Data()
**************************************************************************************************************************************************/
void ADS1220_Get_Conversion_Data_Calibrated (unsigned char *conversionData)
{
    unsigned char outData[3] = {0xff, 0xff, 0xff};
    unsigned char tempData[3];
    long temp_voltage;

    Change_SPI_Phase(SPI_CHANGE_ON_FIRST_EDGE);
    SPI_Write (SpiHandle, outData, tempData, 3);    // 3 Bytes of Conversion Data

    temp_voltage = (((long)tempData[0] << 16) + ((long)tempData[1] << 8) + (long)tempData[2]);

    RawVoltage = temp_voltage;

    temp_voltage = temp_voltage - OffsetCalibrateValue;

    CalVoltage = temp_voltage;

    // Make sure that there is a value in place for the Gain
    if (*(unsigned long *)&FlashGainCorrection != 0xffffffff)
        temp_voltage = (long) ((float)temp_voltage * FlashGainCorrection);

    GainVoltage = temp_voltage;

    conversionData[0] = (temp_voltage >> 16) & 0xff;
    conversionData[1] = (temp_voltage >> 8) & 0xff;
    conversionData[2] = temp_voltage & 0xff;
}
