/*
 * debugUart.c
 *
 *  Created on: Nov 17, 2016
 *      Author: robl
 */

#ifdef WATER_DEBUG

#include "outpour.h"
#include "debugUart.h"
#include "waterDetect.h"

/***************************
 * Module Data Definitions
 **************************/

#define TRUE true
#define FALSE false

/****************************
 * Module Data Declarations
 ***************************/

/**
 * \var isrCommBuf
 * \brief This buffer location is specified in the linker
 *        command file to live right below the stack space in
 *        RAM.
 */
#pragma SET_DATA_SECTION(".commbufs")
uint8_t isrCommBuf[ISR_BUF_SIZE];
uint8_t dbg_line[DEBUG_LINE_SIZE];
#pragma SET_DATA_SECTION()

uint8_t isrCommRecv[3];                                    // the latest byte received
uint8_t isrCommBufHead;
uint8_t isrCommBufTail;

#ifdef GPS_FUNCTIONALITY_ENABLED
extern signed char byteCtr;
extern unsigned char *TI_receive_field;
extern unsigned char *TI_transmit_field;
#endif

/*************************
 * Module Prototypes
 ************************/

static inline void enable_UART_tx(void);
static inline void enable_UART_rx(void);
static inline void disable_UART_tx(void);
static inline void disable_UART_rx(void);

/***************************
 * Module Public Functions
 **************************/


static inline void enable_UART_tx(void)
{
    UC0IE |= UCA0TXIE;
}

static inline void enable_UART_rx(void)
{
    UC0IE |= UCA0RXIE;
}

static inline void disable_UART_tx(void)
{
    UC0IE &= ~UCA0TXIE;
}

static inline void disable_UART_rx(void)
{
    UC0IE &= ~UCA0RXIE;
}

#ifdef DBG_DETAILS
static uint8_t add_wordval(uint8_t *dest, uint16_t meas, uint8_t digits)
{
    uint8_t i;
    uint8_t bitmove;

    // make hex representation of data
    for (i = 0; i < digits; i++)
    {
        bitmove = (((digits - 1) - i) * 4);
        dest[i] = ((0xf << bitmove) & meas) >> bitmove;
        if (dest[i] > 9)
        {
            dest[i] -= 10;
            dest[i] += 'a';
        }
        else
            dest[i] += '0';
    }
    return (i);
}

static int debug_pad_meas(uint8_t *dest, uint8_t pad)
{
    uint8_t i = 0;
    uint8_t state;
    uint8_t num_samp;
    uint16_t mean;

    mean = waterDetect_getPadState(pad, &state, &num_samp);
    dest[i++] = pad + '0';
    dest[i++] = '(';                                       //2
    dest[i++] = state;                                     //3
    dest[i++] = ')';                                       //4
    i += add_wordval(&dest[i], mean, 4);                   //5-8
    if (num_samp != SAMPLE_COUNT)
        dest[i++] = '*';                                   //9
    else
        dest[i++] = ' ';                                   //9

    return (i);
}


static int debug_level(uint8_t *dest, uint8_t level)
{
    uint8_t i = 0;

    dest[i++] = 'L';
    dest[i++] = level + '0';


    return (i);
}

static int debug_flow_out(uint8_t *dest, uint8_t level, uint8_t unknowns)
{
    uint8_t i = 0;
    uint8_t percentile;

    uint16_t flow_rate = waterDetect_get_flow_rate(level,&percentile);

    dest[i++] = 'F';
    dest[i++] = ((flow_rate / 100) % 10) + '0';
    dest[i++] = ((flow_rate / 10) % 10) + '0';
    dest[i++] = (flow_rate % 10) + '0';
    dest[i++] = unknowns ? 'u' : ' ';
    dest[i++] = 'P';
    dest[i++] = ((percentile / 100) % 10) + '0';
    dest[i++] = ((percentile / 10) % 10) + '0';
    dest[i++] = (percentile % 10) + '0';
    dest[i++] = unknowns ? 'u' : ' ';
    dest[i++] = '\n';
    return (i);
}

static int debug_target_out(uint8_t *dest, uint8_t pad)
{
    uint8_t i = 0;
    uint16_t target = waterDetect_getPadTargetWidth(pad);

    dest[i++] = pad + '0';
    dest[i++] = ':';
    dest[i++] = ((target / 10000) % 10) + '0';
    dest[i++] = ((target / 1000) % 10) + '0';
    dest[i++] = ((target / 100) % 10) + '0';
    dest[i++] = ((target / 10) % 10) + '0';
    dest[i++] = (target % 10) + '0';
    dest[i++] = ' ';

    return (i);
}

#endif

static int debug_time(uint8_t *dest, uint32_t sys_time)
{
    uint8_t i = 0;

    dest[i++] = 'T';
    i += add_wordval(&dest[i], sys_time & 0xffff, 4);
    dest[i++] = ' ';

    return (i);
}

#ifdef DBG_TEMP
static int debug_temp_out(uint8_t *dest, uint16_t temp)
{
    uint8_t i = 0;

    dest[i++] = 't';
    dest[i++] = '=';
    dest[i++] = ((temp / 100) % 10) + '0';
    dest[i++] = ((temp / 10) % 10) + '0';
    dest[i++] = (temp % 10) + '0';
    dest[i++] = 'C';
    dest[i++] = '\n';
    return (i);
}
#endif

#ifdef DBG_POUR
static int debug_pour_out(uint8_t *dest, uint32_t pour)
{
    uint8_t i = 0;

    dest[i++] = 'p';
    dest[i++] = '=';
    dest[i++] = ((pour / 10000) % 10) + '0';
    dest[i++] = ((pour / 1000) % 10) + '0';
    dest[i++] = ((pour / 100) % 10) + '0';
    dest[i++] = ((pour / 10) % 10) + '0';
    dest[i++] = (pour % 10) + '0';
    dest[i++] = 'm';
    dest[i++] = 'l';
    dest[i++] = '\n';
    return (i);
}
#endif

#ifdef DBG_SAMPLES
static int debug_sample_out(uint8_t *dest, uint8_t pad_number)
{
    uint8_t i = 0;
    uint16_t sample;

    sample = waterDetect_getCurrSample(pad_number);

    dest[i++] = ((sample / 10000) % 10) + '0';
    dest[i++] = ((sample / 1000) % 10) + '0';
    dest[i++] = ((sample / 100) % 10) + '0';
    dest[i++] = ((sample / 10) % 10) + '0';
    dest[i++] = (sample % 10) + '0';
    dest[i++] = ',';
    dest[i++] = pad_number + '0';
    dest[i++] = ',';

    return (i);
}
#endif

#ifdef DBG_DETAILS
//called by waterSense.c
void debug_padSummary(uint32_t sys_time, uint8_t level, uint8_t unknowns, uint8_t pump_active)
{
    uint8_t pad_number;
    uint8_t dbg_len = 0;

	dbg_line[dbg_len++]=pump_active?'$':'X';
    dbg_len = debug_time(&dbg_line[dbg_len], sys_time);
    for (pad_number = 0; pad_number < NUM_PADS; pad_number++) dbg_len += debug_pad_meas(&dbg_line[dbg_len], pad_number);
    dbg_len += debug_level(&dbg_line[dbg_len], level);
    dbg_len += debug_flow_out(&dbg_line[dbg_len], level, unknowns);
    dbg_uart_write(dbg_line, dbg_len);
}


void debug_padTargets(void)  {
    uint8_t pad_number;
    uint8_t dbg_len = 0;

	dbg_line[dbg_len++]='<';
	dbg_line[dbg_len++]='.';
	dbg_line[dbg_len++]='>';
    for (pad_number = 0; pad_number < NUM_PADS; pad_number++)
    	dbg_len += debug_target_out(&dbg_line[dbg_len], pad_number);
	dbg_line[dbg_len++]='\n';
    dbg_uart_write(dbg_line, dbg_len);
}


void debug_sampProgress(void)
{
	dbg_line[0]='@';
    dbg_uart_write(dbg_line, 1);
}
#endif

#ifdef DBG_TEMP
//called by waterSense.c
void debug_internalTemp(uint32_t sys_time, uint16_t temp)
{
    uint8_t dbg_len = 0;

    dbg_len = debug_time(&dbg_line[dbg_len], sys_time);
    dbg_len += debug_temp_out(&dbg_line[dbg_len], temp);

    dbg_uart_write(dbg_line, dbg_len);
}
#endif

#ifdef DBG_POUR
//called by waterSense.c
void debug_pour_total(uint32_t sys_time, uint32_t total_pour)
{
    uint8_t dbg_len = 0;

    dbg_len = debug_time(&dbg_line[dbg_len], sys_time);
    dbg_len += debug_pour_out(&dbg_line[dbg_len], total_pour);

    dbg_uart_write(dbg_line, dbg_len);
}
#endif

#ifdef DBG_SAMPLES
//called by sysExec.c
void debug_sample_dump(void)
{
    uint8_t pad_number;
    uint8_t dbg_len = 0;

    for (pad_number = 0; pad_number < NUM_PADS; pad_number++) dbg_len += debug_sample_out(&dbg_line[dbg_len], pad_number);
    dbg_line[dbg_len++] = '\n';
    dbg_uart_write(dbg_line, dbg_len);
}
#endif
/**
* \brief One time initialization for module.  Call one time
*        after system starts or (optionally) wakes.
* \ingroup PUBLIC_API
*/
void dbg_uart_init(void)
{
    uint8_t __attribute__((unused)) garbage;

    memset(isrCommBuf, 0, ISR_BUF_SIZE);
    isrCommBufHead = 0;
    isrCommBufTail = ISR_BUF_SIZE - 1;
    memset(isrCommRecv, 0xff, 3);

    // clear out receive buffer
    disable_UART_rx();
    garbage = UCA0RXBUF;
    enable_UART_rx();
}

void dbg_uart_write(uint8_t *writeCmdP, uint8_t len)
{
    uint8_t i;

    disable_UART_tx();
    disable_UART_rx();

    for (i = 0; i < len; i++)
    {
        // if the queue isn't full
        if (isrCommBufHead != isrCommBufTail)
        {
            isrCommBuf[isrCommBufHead] = writeCmdP[i];
            if (isrCommBufHead + 1 < ISR_BUF_SIZE)
                isrCommBufHead++;
            else
                isrCommBufHead = 0;
        }
        else
            break;                                         // we sent all we could
    }

    enable_UART_tx();
    enable_UART_rx();
}

uint8_t dbg_uart_read(void)
{
    uint8_t answer;

    disable_UART_rx();
    answer = isrCommRecv[0];
    enable_UART_rx();

    return (answer);
}

uint8_t dbg_uart_txqempty(void)
{
    uint8_t next;

    if (isrCommBufTail + 1 >= ISR_BUF_SIZE)
        next = 0;
    else
        next = isrCommBufTail + 1;

	if (next == isrCommBufHead)
		return(TRUE);
	else
		return(FALSE);
}

uint8_t dbg_uart_txpend(void)
{
	return(UC0IFG & UCA0TXIFG);
}

/*****************************
 * UART Interrupt Functions
 ****************************/

/**
* \brief Uart Transmit Interrupt Service Routine.
* \ingroup ISR
*/
extern uint8_t CAPSENSE_ACTIVE;
#ifndef FOR_USE_WITH_BOOTLOADER
#pragma vector=USCIAB0TX_VECTOR
#endif
__interrupt void USCI0TX_ISR(void)
{
#ifdef GPS_FUNCTIONALITY_ENABLED
	// I2C Rx
	if ((IFG2 & UCB0RXIFG)&&(IE2 & UCB0RXIE)){
	if ( byteCtr == 0 ){
	  UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	  *TI_receive_field = UCB0RXBUF;
	  TI_receive_field++;
	}
	else {
	  *TI_receive_field = UCB0RXBUF;
	  TI_receive_field++;
	  byteCtr--;
	}
	}

	if (UCB0STAT & UCNACKIFG){            // send STOP if slave sends NACK
	UCB0CTL1 |= UCTXSTP;
	UCB0STAT &= ~UCNACKIFG;
	}

	//(needed?) added to support i2c humid sensor, interrupt problems when debug uart disabled?
	// I2C Tx
	if ((IFG2 & UCB0TXIFG) && (IE2 & UCB0TXIE)) {
	if (byteCtr == 0){
	  UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	  IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
	}
	else {
	  UCB0TXBUF = *TI_transmit_field;
	  TI_transmit_field++;
	  byteCtr--;
	}
	}
#endif
	//UART Tx
	if ((IFG2 & UCA0TXIFG) && (IE2 & UCA0TXIE)) {

    uint8_t next;

    if (isrCommBufTail + 1 >= ISR_BUF_SIZE)
        next = 0;
    else
        next = isrCommBufTail + 1;

    // do we have data to send?
    if (next != isrCommBufHead)
    {
        isrCommBufTail = next;

        UCA0TXBUF = isrCommBuf[next];
    }
    else
        disable_UART_tx();
	}

	// Catch to go back into LPM in case we try to process UART/I2C while taking capsense data
    if (CAPSENSE_ACTIVE){
    	__bis_SR_register_on_exit(LPM3_bits + GIE); // continue waiting for the WDT interrupt
    }
}

/**
* \brief Uart Receive Interrupt Service Routine.
* \ingroup ISR
*/
#ifndef FOR_USE_WITH_BOOTLOADER
#pragma vector=USCIAB0RX_VECTOR
#endif
__interrupt void USCI0RX_ISR(void)
{
	// uart Rx
	if ((IFG2 & UCA0RXIFG)&&(IE2 & UCA0RXIE)){
    // only save the last byte received
    isrCommRecv[0] = UCA0RXBUF;
	}

    if (CAPSENSE_ACTIVE){
    	__bis_SR_register_on_exit(LPM3_bits + GIE); // continue waiting for the WDT interrupt
    }
}

#endif

