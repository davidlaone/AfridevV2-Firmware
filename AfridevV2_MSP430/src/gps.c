/** 
 * @file gpsMsg.c
 * \n Source File
 * \n Outpour MSP430 Firmware
 * 
 * \brief GPS module responsible for controlling the GPS power 
 *        and message modules.
 *
 * \brief When the Start GPS function is called, it will power
 *        on the GPS device and wait for a valid satellite fix.
 *        The GPS will be powered off as soon as a satellite fix
 *        is obtained. If no satellite fix occurs within
 *        MAX_ALLOWED_GPS_FIX_TIME_IN_SEC seconds, the GPS is
 *        powered off.
 *  
 * \brief The gps_start function should be called from the upper
 *        layers to perform GPS processing. After this module
 *        has completed its GPS processing, the last valid RMC
 *        string can be retrieved using the API from the gpsMsg
 *        module. Regardless of whether the GPS was able to
 *        determine a satellite fix, the last RMC message is
 *        saved by the gpsMsg module for return to the cloud.
 */

#include "outpour.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \def MAX_ALLOWED_GPS_FIX_TIME_IN_SEC
 * Max time to have the GPS device on waiting for a satellite 
 * fix. 
 */
#define MAX_ALLOWED_GPS_FIX_TIME_IN_SEC ((10*60)*TIME_SCALER)

/**
 * \def MAX_GPS_RETRY_ON_ERROR
 * If an error occurs powering on the GPS device or a timeout 
 * occurs waiting for a RMC message, how many times the state 
 * machine will retry to get a valid satellite fix. 
 */
#define MAX_GPS_RETRY_ON_ERROR 4

/**
 * \typedef gpsState_t
 * \brief Define the different states to manage on the GPS.
 */
typedef enum {
    GPS_STATE_IDLE,
    GPS_STATE_POWER_UP,
    GPS_STATE_POWER_UP_WAIT,
    GPS_STATE_MSG_RX_START,
    GPS_STATE_MSG_RX_WAIT,
    GPS_STATE_RETRY,
    GPS_STATE_DONE,
} gpsState_t;

/**
 * \typedef gpsData_t
 * \brief Module data structure.
 */
typedef struct gpsData_s {
    bool active;
    gpsState_t state;
    uint8_t gpsOnRetryCount;
    uint8_t retryCount;
    sys_tick_t startGpsTimestamp;
} gpsData_t;

/****************************
 * Module Data Declarations
 ***************************/

/**
 * \var gpsData
 * \brief Declare the object that contains the module data.
 */
gpsData_t gpsData;

/*************************
 * Module Prototypes
 ************************/

static void gps_stateMachine(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief Module init routine. Call once at startup.
* 
* \ingroup PUBLIC_API
*/
void gps_init(void) {
}

/**
* \brief GPS MSG executive. Called every 2 seconds from the main
*        loop to manage the GPS.
*
* \ingroup EXEC_ROUTINE
*/
void gps_exec(void) {
    // printf("%s\n", __func__);
    if (gpsData.active) {
        gps_stateMachine();
    }
}

/**
* \brief Start GPS processing. This will power on the GPS device 
*        and wait for a valid satellite fix. The GPS will be
*        powered off as soon as a satellite fix is obtained. If
*        no satellite fix occurs within
*        MAX_ALLOWED_GPS_FIX_TIME_IN_SEC seconds, the GPS is
*        powered off.
*  
* \note This function should be called from the upper layers to 
*       perform GPS processing. After this module has completed
*       its GPS processing, the last valid RMC string can be
*       retrieved using the API from the gpsMsg module.
*       Regardless of whether the GPS was able to determine a
*       satellite fix, the last RMC message is saved by the
*       gpsMsg module for return to the cloud.
* 
* @return bool Returns true if processing was started. Returns 
*         false if the processing had already been started.
* 
* \ingroup PUBLIC_API
*/
void gps_start(void) {
    if (gpsData.active) {
        return;
    }
    // printf("%s\n", __func__);
    gpsData.active = true;
    gpsData.state = GPS_STATE_POWER_UP;
    gpsData.gpsOnRetryCount = 0;
    gps_stateMachine();
}

/**
* \brief Stop managing the GPS. This will power off the GPS 
*        device and stop any current GPS message processing.
* 
* \ingroup PUBLIC_API
*/
void gps_stop(void) {
    gpsPower_powerDownGPS();
    gpsMsg_stop();
    gpsData.active = false;
}

/**
* \brief Identify if the gps module is currently busy managing 
*        the GPS.
* 
* @return bool Returns true if busy, false otherwise.
*/
bool gps_isActive(void) {
    return gpsData.active;
}

/*************************
 * Module Private Functions
 ************************/

/**
* \brief Statemachine to sequence through the steps of powering 
*        on the GPS and waiting for a valid satellite fix. The
*        gpsPower and gpsMsg API's are called to manage the
*        operations.
*/
static void gps_stateMachine(void) {
    switch (gpsData.state) {
    case GPS_STATE_IDLE:
        break;

    case GPS_STATE_POWER_UP:
        gpsPower_restart();
        gpsData.state = GPS_STATE_POWER_UP_WAIT;
        gpsData.startGpsTimestamp = GET_SYSTEM_TICK();
        break;

    case GPS_STATE_POWER_UP_WAIT:
        if (gpsPower_isGpsOn()) {
            // printf("%s: GPS IS UP\n", __func__);
            gpsData.state = GPS_STATE_MSG_RX_START;
        } else if (gpsPower_isGpsOnError()) {
            // Houston - we have a problem
            gpsData.state = GPS_STATE_RETRY;
        }
        break;

    case GPS_STATE_MSG_RX_START:
        gpsMsg_start();
        gpsData.state = GPS_STATE_MSG_RX_WAIT;
        break;

    case GPS_STATE_MSG_RX_WAIT:
        if (gpsMsg_gotRmcMessage()) {
            // printf("Got RMC Message @ %d\n", sysTime);
            // We have a RMC message available.
            if (gpsMsg_gotDataValidRmcMessage()) {
                // printf("Got Satellite Fix @ %d\n", sysTime);
                // The RMC message contains a valid satellite fix. We are done.
                gpsData.state = GPS_STATE_DONE;
            } else if (GET_ELAPSED_TIME_IN_SEC(gpsData.startGpsTimestamp) > MAX_ALLOWED_GPS_FIX_TIME_IN_SEC) {
                // printf("TIMEOUT WAITING FOR SATELLITE FIX\n");
                // Timeout waiting for a satellite fix - so we are done.
                gpsData.state = GPS_STATE_DONE;
            } else {
                // printf("Get another RMC message, elapsed: %d\n", GET_ELAPSED_TIME_IN_SEC(gpsData.startGpsTimestamp));
                // Keep trying for a satellite fix
                gpsData.state = GPS_STATE_MSG_RX_START;
            }
        } else if (gpsMsg_isError()) {
            // Houston - we have a problem
            gpsData.state = GPS_STATE_RETRY;
        }
        break;

    case GPS_STATE_RETRY:
        gpsPower_powerDownGPS();
        gpsData.gpsOnRetryCount++;
        if (gpsData.gpsOnRetryCount < MAX_GPS_RETRY_ON_ERROR) {
            gpsData.state = GPS_STATE_POWER_UP;
        } else {
            gpsData.state = GPS_STATE_DONE;
        }
        break;

    default:
    case GPS_STATE_DONE:
        gps_stop();
        gpsData.state = GPS_STATE_IDLE;
        // exit(0);
        break;
    }
}

