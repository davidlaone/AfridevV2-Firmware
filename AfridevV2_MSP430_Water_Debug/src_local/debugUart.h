/*
 * debugUart.h
 *
 *  Created on: Nov 17, 2016
 *      Author: robl
 */

#ifndef SRC_LOCAL_DEBUGUART_H_
#define SRC_LOCAL_DEBUGUART_H_

#define DEBUG_LINE_SIZE 72
/**
 * \def ISR_BUF_SIZE
 * \brief Define the size of the UART receive buffer.
 */
#define ISR_BUF_SIZE ((uint8_t)200)

#define DBG_DETAILS
#define DBG_SAMPLES
#define DBG_TEMP
#define DBG_POUR

extern uint8_t dbg_line[DEBUG_LINE_SIZE];

extern void dbg_uart_init(void);
extern void dbg_uart_write(uint8_t *writeCmdP, uint8_t len);
extern uint8_t dbg_uart_txqempty(void);
extern uint8_t dbg_uart_txpend(void);
extern uint8_t dbg_uart_read(void);

#ifdef DBG_DETAILS
void debug_padSummary(uint32_t sys_time, uint8_t level, uint8_t unknowns, uint8_t pump_active);
void debug_sampProgress(void);
void debug_padTargets(void);
#endif

#ifdef DBG_SAMPLES
void debug_sample_dump(void);
#endif

#ifdef DBG_TEMP
void debug_internalTemp(uint32_t sys_time, uint16_t temp);
#endif

#ifdef DBG_POUR
void debug_pour_total(uint32_t sys_time, uint32_t total_pour);
#endif


#endif /* SRC_LOCAL_DEBUGUART_H_ */
