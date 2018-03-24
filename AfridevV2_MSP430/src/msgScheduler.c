/** 
 * @file msgScheduler.c
 * \n Source File
 * \n Outpour MSP430 Firmware
 * 
 * \brief Schedule a message to be sent to the modem. All 
 *        scheduled messages are always sent at 1:00AM. Messages
 *        that are scheduled include:
 *        \li Activated message
 *        \li Daily Water Log message
 *        \li Monthly Check-In message
 *        \li GPS Location message.
 *
 * \brief Schedule a GPS measurement. The GPS measurement is 
 *        always performed at 12:00AM (if scheduled).
 *  
 * \note There are two ways to send a message to the modem: 
 *       immediate and scheduled. To send a message immediately,
 *       usd the dataMsgMgr_sendDataMsg function. To schedule a
 *       message to be transmitted at 1:00AM, use the scheduler
 *       API's. The scheduler kicks off the message transmission
 *       session at 1:00AM if there are messages scheduled. The
 *       scheduler calls the dataMsgMgr_startSendingScheduled
 *       function to kick off the transmission session.
 */

#include "outpour.h"

/***************************
 * Module Data Definitions
 **************************/

/**
 * \typedef msgSchedData_t 
 * \brief Define a structure to hold data for this module.
 */
typedef struct msgSchedData_s {
    bool msgScheduled;          /**< Flag to indicate there is at least one message scheduled */
    bool sendDailyWaterLogs;    /**< Flag to indicate the daily water log message is scheduled */
    bool sendActivated;         /**< Flag to indicate the Activated message is scheduled */
    bool sendMonthlyCheckIn;    /**< Flag to indicate the Monthly Check-In message is scheduled */
    bool sendGpsLocation;       /**< Flag to indicate the GPS Location message is scheduled */
    bool performGpsMeasurement; /**< Flag to indicate the a GPS measurement is scheduled */
} msgSchedData_t;

/****************************
 * Module Data Declarations
 ***************************/

/**
* \var msgSchedData
* \brief Declare the data msg object.
*/
// static
msgSchedData_t msgSchedData;

/*************************
 * Module Prototypes
 ************************/

/***************************
 * Module Public Functions
 **************************/

/**
* \brief One time initialization for module.  Call one time 
*        after system starts or (optionally) wakes.
* \ingroup PUBLIC_API
*/
void msgSched_init(void) {
}

/**
* \brief Executive that manages this module. Called every 2 
*        seconds from the main processing loop.
* \ingroup EXEC_ROUTINE
*/
void msgSched_exec(void) {

    // Check if any message is scheduled to be transmitted
    if (msgSchedData.msgScheduled) {
        // Get time from the storage module and check against 1:00AM
        if (storageMgr_getStorageClockHour() == 1) {
            // Start the transmission cycle
            dataMsgMgr_startSendingScheduled();
            // Clear flag
            msgSchedData.msgScheduled = false;
        }
    }

    // Check if there a GPS measurement to perform
    if (msgSchedData.performGpsMeasurement) {
        // Get time from the storage module and check against 12:00AM
        if (storageMgr_getStorageClockHour() == 0) {
            // Start a GPS measurement
            gps_start();
            msgSchedData.performGpsMeasurement = false;
        }
    }
}

/**
* \brief Retrieve the next message to transmit (if any). This 
*        function is used by the msgData.c module to retrieve a
*        scheduled message.
* 
* @param cmdWriteP Modem command object that is filled in by 
*                  this function with info on the payload to
*                  send. If the length field is set to zero on
*                  return, it indicates there is no message to
*                  send.
*/
void msgSched_getNextMessageToTransmit(modemCmdWriteData_t *cmdWriteP) {
    uint8_t *payloadP;
    uint16_t payloadLength = 0;
    MessageType_t payloadMsgId;
    if (msgSchedData.sendDailyWaterLogs) {
        // Start the process of sending the daily logs.
        // Send the oldest daily log that is ready.
        payloadLength = storageMgr_getNextDailyLogToTransmit(&payloadP);
        payloadMsgId = MSG_TYPE_DAILY_LOG;
        if (!payloadLength) {
            msgSchedData.sendDailyWaterLogs = false;
        }
    } else if (msgSchedData.sendActivated) {
        // Retrieve the activated message payload
        payloadLength = storageMgr_getActivatedMessage(&payloadP);
        payloadMsgId = MSG_TYPE_ACTIVATED;
        msgSchedData.sendActivated = false;
    } else if (msgSchedData.sendMonthlyCheckIn) {
        // Retrieve the monthly check-in message payload
        payloadLength = storageMgr_getMonthlyCheckinMessage(&payloadP);
        payloadMsgId = MSG_TYPE_CHECKIN;
        msgSchedData.sendMonthlyCheckIn = false;
    } else if (msgSchedData.sendGpsLocation) {
        // Retrieve the monthly GPS message payload
        payloadLength = gps_getGpsMessage(&payloadP);
        payloadMsgId = MSG_TYPE_GPS_LOCATION;
        msgSchedData.sendGpsLocation = false;
    }

    // Initialize the command object 
    cmdWriteP->cmd             = OUTPOUR_M_COMMAND_SEND_DATA;
    cmdWriteP->payloadMsgId    = payloadMsgId;  /* the payload type */
    cmdWriteP->payloadP        = payloadP;   /* the payload pointer */
    cmdWriteP->payloadLength   = payloadLength;  /* size of the payload in bytes */
}

/**
* \brief Schedule the daily water logs to be sent when the 
*        Storage clock hour is set to 1:00 AM.
*/
void msgSched_scheduleDailyWaterLogMessage(void) {
    msgSchedData.msgScheduled = true;
    msgSchedData.sendDailyWaterLogs = true;
}

/**
* \brief Schedule the Activated message to be sent when the 
*        Storage clock hour is set to 1:00 AM.
*/
void msgSched_scheduleActivatedMessage(void) {
    msgSchedData.msgScheduled = true;
    msgSchedData.sendActivated = true;
}

/**
* \brief Schedule the Monthly Check-In message to be sent when 
*        the Storage clock hour is set to 1:00 AM.
*/
void msgSched_scheduleMonthlyCheckInMessage(void) {
    msgSchedData.msgScheduled = true;
    msgSchedData.sendMonthlyCheckIn = true;
}

/**
* \brief Schedule the GPS Location message to be sent when the 
*        Storage clock hour is set to 1:00 AM.
*/
void msgSched_scheduleGpsLocationMessage(void) {
    msgSchedData.msgScheduled = true;
    msgSchedData.sendGpsLocation = true;
}

/**
* \brief Schedule a GPS measurement to be performed when the 
*        Storage clock hour is set to 12:00 AM (i.e. 0 hours).
*/
void msgSched_scheduleGpsMeasurement(void) {
    msgSchedData.performGpsMeasurement = true;
}

