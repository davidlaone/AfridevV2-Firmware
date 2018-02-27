/** 
 * @file hal.c
 * \n Source File
 * \n Outpour MSP430 Firmware
 * 
 * \brief Hal support routines
 */

#include "outpour.h"

/*******************************************************************************
*
*  MSP430 GPIO Info
*
*  For ports:
*
*  PxDIR: Direction Registers PxDIR
*  Each bit in each PxDIR register selects the direction of the corresponding I/O
*  pin, regardless of the selected function for the pin. PxDIR bits for I/O pins that
*  are selected for other functions must be set as required by the other function.
*    Bit = 0: The port pin is switched to input direction
*    Bit = 1: The port pin is switched to output direction
*
*  PxOUT:
*  Each bit in each PxOUT register is the value to be output on
*  the corresponding I/O pin when the pin is configured as I/O function,
*  output direction, and the pull-up/down resistor is disabled.
*    Bit = 0: The output is low
*    Bit = 1: The output is high
*  If the pin’s pull-up/down resistor is enabled, the corresponding bit
*  in the PxOUT register selects pull-up or pull-down.
*    Bit = 0: The pin is pulled down
*    Bit = 1: The pin is pulled up
*
*  PxREN:  Pull-Up/Down Resistor Enable Registers
*  Each bit in each PxREN register enables or disables the pullup/pulldown
*  resistor of the corresponding I/O pin. The corresponding bit in the PxOUT
*  register selects if the pin is pulled up or pulled down.
*     Bit = 0: Pullup/pulldown resistor disabled
*     Bit = 1: Pullup/pulldown resistor enabled
*
*  PxSEL: PxSEL and PxSEL2 bit are used to select the pin function -
*         I/O port or peripheral module function (00 = I/O Port).
*
******************************************************************************/

/**
* \brief One time init of all clock related subsystems after 
*        boot up.
* \ingroup PUBLIC_API
*/
void hal_sysClockInit(void) {
    DCOCTL = 0;               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    BCSCTL1 |= DIVA_0;        // ACLK/1 [ACLK/(0:1,1:2,2:4,3:8)]
    BCSCTL2 = 0;              // SMCLK [SMCLK/(0:1,1:2,2:4,3:8)]
    BCSCTL3 |= LFXT1S_0;      // LFXT1S0 32768-Hz crystal on LFXT1
}

/**
* \brief One time init of all GPIO port pin related items after 
*        boot up.
* \ingroup PUBLIC_API
*/
void hal_pinInit(void) {

    P1DIR  |= VBAT_GND + _1V8_EN + GSM_DCDC; // Output
                                             // GSM_INT and GSM_STAT are GPIO inputs, set P1DIR to 0
    P1DIR  &= ~(GSM_INT + GSM_STATUS);

    // VBAT_GND controls VBAT sensing, default off (Active LOW)
    // GSM_DCDC controls (U4) default off (Active HIGH)
    // _1V8_EN controls (U5) default off (Active HIGH)
    P1OUT  |= VBAT_GND;

    // VBAT_MON adc pin initializtion should go here

    P2SEL  |= BIT6 + BIT7; // configuring crystal XIN for P2.6

    // BIT0 = ACLK out on pin 2.0
    // I2C_DRV is output GPIO (default hi-z) Controls I2C
    // GSM_PWR is output GPIO (default off) On when GPS on
    // LS_VCC is output GPIO (default off) On when modem on
    // BIT7 is for crystal XOUT on P2.7
    P2DIR  |= I2C_DRV + GSM_EN + LS_VCC + BIT7;


#ifdef USE_UART_SIGNALS_FOR_GPIO
    // If using UART signals for timing GPIO outputs
    P3DIR  |= RXD + TXD; // P1.1 = RXD, P1.2=TXD
#else
    // UART Tx - to debug UART in debug mode, to modem in modem mode
    P3SEL  |= TXD;
    // UART Rx
    P3SEL2 |= RXD;
#endif
    P3DIR  |= VBAT_GND + _1V8_EN;

    P4DIR  |= GPS_ON_OFF;
}

/**
* \brief One time init of the UART subsystem after boot up.
* \ingroup PUBLIC_API
*/
void hal_uartInit(void) {
    //  ACLK source for UART
    UCA0CTL1 |= UCSSEL_1;         // ACLK
    UCA0BR0 = 0x03;               // 32 kHz 9600
    UCA0BR1 = 0x00;               // 32 kHz 9600
    UCA0MCTL = UCBRS0 + UCBRS1;   // Modulation UCBRSx = 3

    UCA0CTL1 &= ~UCSWRST;         // **Initialize USCI state machine**
    UC0IE |= UCA0RXIE;            // Enable USCI_A0 RX interrupt
}

