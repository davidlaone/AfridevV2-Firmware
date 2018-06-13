/** 
 * @file sysExec.c
 * \n Source File
 * \n Afridev-V2 MSP430 Firmware
 * 
 * \brief main system exec that runs the top level loop.
 */

#include "outpour.h"
#include "waterDetect.h"
#include "waterSense.h"
#include "debugUart.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \def NO_WATER_HF_TO_LF_TIME_IN_SECONDS
 * \brief  Specify how long to wait before switching to
 *         the low-frequency (LF) water measurement interval in
 *         order to save power. By default, the system is in the
 *         high-frequency (HF) water measurement interval where
 *         water measurements are take every main loop
 *         iterataion. If water is detected while in the LF
 *         mode, the system immediately switches back to HF
 *         mode.
 */
#define NO_WATER_HF_TO_LF_TIME_IN_SECONDS ((uint16_t)TIME_60_SECONDS*5)

/**
 * \def LOW_FREQUENCY_MEAS_TIME_IN_SECONDS
 * \brief Specify how often to take the batch water measurements
 *        when in the low-frequency water measurement mode. The
 *        system transitions to the low-frequency interval if no
 *        water has been detected over a
 *        NO_WATER_HF_TO_LF_TIME_IN_SECONDS window of time.
 */
#define LOW_FREQUENCY_MEAS_TIME_IN_SECONDS ((uint8_t)TIME_60_SECONDS)

/**
 * \def WATER_LF_MEAS_BATCH_COUNT
 * This controls the number of water measurements taken per 
 * LOW_FREQUENCY_MEAS_TIME_IN_SECONDS while in the low-frequency 
 * water measurement mode.
 */
#define WATER_LF_MEAS_BATCH_COUNT 4

/**
 * \typedef sysExecData_t
 * \brief Container to hold data for the sysExec module.
 */
typedef struct sysExecData_s {
    uint8_t noWaterMeasCount;         /**< Maintain a count of sequential no water detected measurements */
    uint8_t waterMeasDelayCount;      /**< Delay next water measurement count-down */
} sysExecData_t;

/****************************
 * Module Data Declarations
 ***************************/

sysExecData_t sysExecData;

/*************************
 * Module Prototypes
 ************************/
static uint16_t analyzeWaterMeasurementData(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief This function contains the main software loop
*        for the MSP430 firmware.  After calling the
*        initialization routines, it drops into an infinite
*        while loop.  In the while loop, it goes into a low
*        power mode (i.e. sleep) waiting for the timer interrupt
*        to wake it up. The timer interrupt occurs once every .5
*        seconds. Once awake, it performs a water capacitance
*        measurement. Every 4th time through the main loop, it
*        performs the water detection algorithm and system tasks
*        by calling the exec routines of the different
*        sub-modules.
*
* \ingroup EXEC_ROUTINE
*/
void sysExec_exec(void) {

    uint8_t exec_main_loop_counter = 0;

    // Restart the one-second watchdog timeout
    WATCHDOG_TICKLE();

    memset(&sysExecData, 0, sizeof(sysExecData_t));

    // Initialize the date for Jan 1, 2017
    // h, m, s, am/pm (must be in BCD)
    setTime(0x00, 0x00, 0x00, 0x00);
    // y, m, d
    setDate(2017, 1, 1);

    // Call the module init routines
    dbg_uart_init();
    waterSense_init();

    // Start the timer interrupt
    timerA0_init();

    // Enable the global interrupt
    enableGlobalInterrupt();

    // Restart the one-second watchdog timeout
    WATCHDOG_TICKLE();

    // Start the infinite exec main loop
    while (1) {

        // Go into low power mode.
        // Wake on the exit of the TimerA0 interrupt.
        // Wakes every .5 seconds to run the main loop
        __bis_SR_register(LPM3_bits);

        // Restart the one-second watchdog timeout
        WATCHDOG_TICKLE();

        // Take a water measurement
        // Don't take a measurement if the modem or GPS is in use or the water
        // measurement delay count is above the batch count. The waterMeasDelay
        // count is always zero in HF water measurement mode. In LF water measurement
        // mode, the counter is used to control how often the water measurements are
        // performed. In LF water measurement mode, we want to take a batch of
        // measurements periodically.
        if (sysExecData.waterMeasDelayCount < WATER_LF_MEAS_BATCH_COUNT) {
            waterSense_takeReading();

#ifdef DBG_DETAILS
            debug_sampProgress();  // type a @
#endif

#ifdef DBG_SAMPLES
            // print current readings
            // debug messages are selected/deselected in debugUart.h
            debug_sample_dump();
#endif

        }

        // Increment main loop counter
        exec_main_loop_counter++;

        // Perform system tasks every SECONDS_PER_TREND (i.e. every 2 seconds)
        // which is every fourth time that the exec main loop runs.
        if (exec_main_loop_counter >= TICKS_PER_TREND) {

            uint16_t __attribute__((unused)) currentFlowRateInMLPerSec = 0;

            // zero the main loop counter
            exec_main_loop_counter = 0;

            currentFlowRateInMLPerSec = analyzeWaterMeasurementData();

            // about to sleep for 20 seconds
            while(!dbg_uart_txqempty());  // send the rest of the debug data
            while(dbg_uart_txpend());     // wait for the last character to be consumed
            _delay_cycles(2000);          // wait one character time afterwards so the terminal is not confused
        }
    }
}

/***************************
 * Module Private Functions
 **************************/

/**
 * \brief Call the water data analysis routine. Check if the 
 *        system should move to the low-frequency (LF)
 *        measurement mode in order to save power. This
 *        happens if no water has been detected for a certain
 *        amount of time. By default, the system is in the
 *        high-frequency (HF) measurement mode.  When in LF
 *        measurement mode, the water analysis function is
 *        not performed on every call. When in HF measurement
 *        mode, it is.
 * 
 * @return uint16_t Returns the current flow rate measured in 
 *         milliliters per second.
 */
static uint16_t analyzeWaterMeasurementData(void) {
    uint16_t currentFlowRateInMLPerSec = 0;
    bool waterDetectedFlag = false;
    bool didWaterAnalysisFlag = false;

    // When in LF measurement mode, a batch of water data analysis calls will be
    // performed  before deciding if the LF measurement mode should continue. If
    // in HF measurement mode, then waterMeasDelayCount will be zero and the water
    // analysis is performed on every call. If water is detected on any measurement,
    // then the system will always return to HF measurement mode.

    // In LF water measurement mode, take a batch of measurements periodically.
    // In HF water measurement mode, the waterMeasDelayCount will always be zero.
    if (sysExecData.waterMeasDelayCount < WATER_LF_MEAS_BATCH_COUNT) {

        // Perform algorithm to analyze water data samples.
        waterSense_analyzeData();

        // Read the flow rate in milliliters per second
        currentFlowRateInMLPerSec = waterSense_getLastMeasFlowRateInML();

        if (currentFlowRateInMLPerSec) {
            waterDetectedFlag = true;
        }
        didWaterAnalysisFlag = true;
    }

    if (didWaterAnalysisFlag && waterDetectedFlag) {
        // If water flow is detected, then clear all LF water measurement counters.
        sysExecData.noWaterMeasCount = 0;
        sysExecData.waterMeasDelayCount = 0;
    }

    // Perform checks to determine if system should go into LF measurement mode.
    else {

        // Once the noWaterMeasCount reaches the timeout count, it will no
        // longer change until water has been detected, keeping the water
        // measurement interval in the LF mode.

        // For the compare, the NO_WATER_HF_TO_LF_TIME_IN_SECONDS is divided by
        // two (SECONDS_PER_TREND) because the noWaterMeasCount count is incremented
        // once every two seconds.

        if (sysExecData.noWaterMeasCount < ((NO_WATER_HF_TO_LF_TIME_IN_SECONDS >> SECONDS_PER_TREND_SHIFT)-1)) {
            // The noWaterMeasCount is incremented once every two seconds.
            sysExecData.noWaterMeasCount++;
        } else {
            // No water has been detected for at least NO_WATER_HF_TO_LF_TIME_IN_SECONDS.
            // Stay in the LF water measurement mode in order to save power.
            // If the waterMeasDelayCount has expired, re-start the LF delay countdown.
            if (!sysExecData.waterMeasDelayCount) {
                // Set the delay counter based on how often the waterMeasDelayCount is
                // decremented within the exec main loop. It is decremented once every
                // two seconds (SECONDS_PER_TREND)
                sysExecData.waterMeasDelayCount = (LOW_FREQUENCY_MEAS_TIME_IN_SECONDS >> SECONDS_PER_TREND_SHIFT);
            }
        }
    }

    // If the LF delay count is non-zero, then decrement it.
    if (sysExecData.waterMeasDelayCount) {
        sysExecData.waterMeasDelayCount--;
    }

    return currentFlowRateInMLPerSec;
}

