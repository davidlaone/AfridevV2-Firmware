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

static const char no_gps_data_string[] = "NO DATA\n";

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
    // Always set UART mux for use with modem after GPS is done
    MODEM_UART_SELECT_ENABLE();
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
            // We have a RMC message available.
            if (gpsMsg_gotDataValidRmcMessage()) {
                // The RMC message contains a valid satellite fix. We are done.
                gpsData.state = GPS_STATE_DONE;
            } else if (GET_ELAPSED_TIME_IN_SEC(gpsData.startGpsTimestamp) > MAX_ALLOWED_GPS_FIX_TIME_IN_SEC) {
                // Timeout waiting for a satellite fix - so we are done.
                gpsData.state = GPS_STATE_DONE;
            } else {
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
        // Always schedule a GPS message to be sent after the
        // GPS measurement is complete.
        msgSched_scheduleGpsLocationMessage();
        break;
    }
}

/**
* \brief Utility function to send the GPS message to the modem.
*/
void gps_sendGpsMessage(void) {
    uint8_t *ptr = NULL;
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    // Fill in the buffer with the standard message header
    uint8_t payloadSize = storageMgr_prepareMsgHeader(payloadP, MSG_TYPE_GPS_LOCATION);
    // Add the GPS data if available
    ptr = &payloadP[payloadSize];
    if (gpsMsg_gotRmcMessage()) {
        payloadSize += gpsMsg_getRmcMessage(ptr);
    } else {
        // If a RMC string is not available, put the default no-data string.
        strcpy((char *)ptr, no_gps_data_string);
        payloadSize += sizeof(no_gps_data_string);
    }
    // Initiate sending the GPS message
    dataMsgMgr_sendDataMsg(MSG_TYPE_GPS_LOCATION, payloadP, payloadSize);
}

/**
* \brief Build the GPS Location message for transmission. The 
*        shared buffer is used to hold the message. The standard
*        msg header is added first followed by the GPS data.
* 
* @param payloadPP Pointer to fill in with the address of the 
*                  message to send.
* 
* @return uint16_t Length of the message in bytes.
*/
uint16_t gps_getGpsMessage(uint8_t **payloadPP) {
    uint8_t *ptr = NULL;
    // Get the shared buffer (we borrow the ota buffer)
    uint8_t *payloadP = modemMgr_getSharedBuffer();
    // Fill in the buffer with the standard message header
    uint8_t payloadSize = storageMgr_prepareMsgHeader(payloadP, MSG_TYPE_GPS_LOCATION);
    // Add the GPS data if available
    ptr = &payloadP[payloadSize];
    if (gpsMsg_gotRmcMessage()) {
        payloadSize += gpsMsg_getRmcMessage(ptr);
    } else {
        // If a RMC string is not available, put the default no-data string.
        strcpy((char *)ptr, no_gps_data_string);
        payloadSize += sizeof(no_gps_data_string);
    }
    // For the GPS message, we always return 128 bytes. This 
    // includes the 16 byte header and 112 bytes of data payload.
    // Not all of the data payload will contain GPS data - so junk
    // is returned for those unused bytes.
    payloadSize = 128;
    // Assign pointer
    *payloadPP = payloadP;
    // return payload size
    return payloadSize;
}

/**
* \brief Copy the last RMC message that was received into the 
*        buffer. If there was not a RMC message received, then
*        return the string "NO DATA". Copies at most 96 bytes
*        into ther buffer. The data is always a NULL terminated
*        ASCII string.
* 
* @param bufP Buffer to copy the RMC message string into.
* 
* @return uint16_t The number of bytes copied.
*/
uint16_t gps_getGpsData(uint8_t *bufP) {
    uint8_t length;
    if (gpsMsg_gotRmcMessage()) {
        length = gpsMsg_getRmcMessage(bufP);
    } else {
        // If a RMC string is not available, put the default no-data string.
        strcpy((char *)bufP, no_gps_data_string);
        length = sizeof(no_gps_data_string);
    }
    // return length of data put in the buffer
    return length;
}
