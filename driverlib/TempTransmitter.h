/*
 * TempTransmitter.h
 *
 *  Created on: Sep 11, 2013
 *      Author: a0207881
 */

#ifndef TEMPTRANSMITTER_H_
#define TEMPTRANSMITTER_H_

#include <msp430.h>
#include "usci_digital_io.h"

#define  UART_PORT                   PORT_1
#define  SPI_PORT                    PORT_1

#define  DAC_CS_PORT                 PORT_3
#define  ADS_CS_PORT                 PORT_1

// Port 1 pins

#define UART_RX_PIN                  BIT1       // P1.1 - UART RX
#define UART_TX_PIN                  BIT2       // P1.2 - UART TX
#define USR_USER_INTERRUPT_N         BIT3       // P1.3
#define ADS_CS_N                     BIT4       // P1.4
#define ADS_DAC_CLK                  BIT5       // P1.5
#define ADS_DAC_DOUT                 BIT6       // P1.6 - SOMI
#define ADS_DAC_DIN                  BIT7       // P1.7 - SIMO

// Port 2 pins

#define DAC_ERROR_N                  BIT0       // P2.0
#define ERR_LVL                      BIT1       // P2.1
#define USR_GAIN_CAL                 BIT3       // P2.3
#define VCC_ISO_UART_EN              BIT4       // P2.4
#define ADS_DRDY_N                   BIT5       // P2.5

// Port 3 pins

#define DAC_CS_N                     BIT1       // P3.1

#endif /* TEMPTRANSMITTER_H_ */
