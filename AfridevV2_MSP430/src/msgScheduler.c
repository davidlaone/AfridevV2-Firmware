/** 
 * @file msgScheduler.c
 * \n Source File
 * \n Outpour MSP430 Firmware
 * 
 * \brief Schedule a message to be sent to the modem, and then 
 *        send the message at the scheduled time which is always
 *        at 1:00AM. Messages that are scheduled include the
 *        Activated message, Daily Water Log message and the
 *        Monthly Check-In message.
 * \brief Schedule a GPS measurement, and then start the GPS 
 *        measurement at the the schedule time which is always
 *        at 12:00AM.
 */

#include "outpour.h"

/***************************
 * Module Data Definitions
 **************************/

typedef struct msgSchedData_s {
    bool msgScheduled;
    bool sendDailyWaterLogs;
    bool sendActivated;
    bool sendMonthlyCheckIn;
    bool sendGps;
    bool performGpsMeasurement;
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
            // These message are mutually exclusive, and send one of the message types
            // negates any other message types that may be schedule. There is a hierarchy of
            // which message types take precedence.
            if (msgSchedData.sendDailyWaterLogs) {
                // Start the process of sending the daily logs.
                // This will send the oldest daily log that is ready.
                dataMsgMgr_sendDailyLogs();
            } else if (msgSchedData.sendActivated) {
                // Send the activated status message
                storageMgr_sendActivatedMessage();
            } else if (msgSchedData.sendMonthlyCheckIn) {
                // Send the monthly check-in message
                storageMgr_sendMonthlyCheckin();
            } else if (msgSchedData.sendGps) {
                gps_sendGpsMessage();
            }
            // Clear all scheduled message types
            // Sending one types negates any others that may be scheduled.
            msgSchedData.msgScheduled = false;
            msgSchedData.sendDailyWaterLogs = false;
            msgSchedData.sendActivated = false;
            msgSchedData.sendMonthlyCheckIn = false;
            msgSchedData.sendGps = false;
        }
    }

    // Chedk if there a GPS measurement to perform
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
* \brief Schedule the GPS message to be sent when the Storage 
*        clock hour is set to 1:00 AM.
*/
void msgSched_scheduleGpsMessage(void) {
    msgSchedData.msgScheduled = true;
    msgSchedData.sendGps = true;
}

/**
* \brief Schedule a GPS measurement to be performed when the 
*        Storage clock hour is set to 12:00 AM (i.e. 0 hours).
*/
void msgSched_scheduleGpsMeasurement(void) {
    msgSchedData.performGpsMeasurement = true;
}

