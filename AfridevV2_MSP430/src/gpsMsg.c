/** 
 * @file gpsMsg.c
 * \n Source File
 * \n Outpour MSP430 Firmware
 * 
 * \brief GPS module responsible for receiving and processing 
 *        GPS messages (from the UART).
 */

// Example valid RMC message
// $GPRMC,020730.000,A,3716.1771,N,12156.0343,W,0.00,73.54,260218,,,A*58
// $GPRMC,051506.000,A,3716.1770,N,12156.0483,W,0.13,213.58,180318,,,A*7B

#include "outpour.h"

/***************************
 * Module Data Definitions
 **************************/

#define RX_BUF_SIZE 96

#define MAX_RMC_WAIT_TIME_IN_SEC (10 * TIME_SCALER)

/**
 * \typedef gps_data_t
 * Structure containing module data
 */
typedef struct gpsMsgdata_s {
    bool busy;                      /**< Flag to indicate module is waiting for a RMC msg */

    bool rmcMsgAvailable;           /**< Flag to indicate a valided RMC msg is ready */
    uint8_t rmcMsgLength;           /**< The length of the validated RMC msg */
    bool rmcDataIsValid;            /**< We have a valid satellite location fix */

    sys_tick_t waitForRmcTimestamp; /**< The time we we started waiting for an RMC msg */
    bool noRmcMsgError;             /**< Flag indicating timeout occurred waiting for RMC msg */

    bool rmcMsgFromIsrReady;        /**< ISR flag to indicate an RMC msg has been received */
    uint8_t isrRxIndex;             /**< ISR index into receive buffer */
    bool isrGotStart$;              /**< ISR processing flag that we got a '$' character */

} gpsMsgData_t;


/****************************
 * Module Data Declarations
 ***************************/

/**
* \var gpsRxBuf 
* Where we will put the bytes received from the GPS. This buffer
* location is specified in the linker command file to live right 
* below the stack space.
*/
#pragma SET_DATA_SECTION(".commbufs")
char gpsRxBuf[RX_BUF_SIZE];
#pragma SET_DATA_SECTION()

/**
* \var gpsMsgData
* Declare module data structure
*/
gpsMsgData_t gpsMsgData;

/**
* \var gnrmc_match_template
* Used to look for the RMC ASCII letters in a NMEA message 
* sentence.
*/
static const uint8_t gnrmc_match_template[] = { '$', 'G', 'P', 'R', 'M', 'C' };

#if 1
/**
* \var rmcTestString
* Used for testing only. Mimic a valid RMC sentance.
*/
static const char rmcTestString[] =  "$GPRMC,051506.000,A,3716.1770,N,12156.0483,W,0.13,213.58,180318,,,A*7B";
#endif

/*************************
 * Module Prototypes
 ************************/

static void gpsMsg_isrRestart(void);
static void gpsMSg_processRmcSentence(void);
static bool gpsMsg_verifyChecksum(void);
static uint8_t asciiToHex(uint8_t asciiByte);
static bool gpsMsg_matchRmc(void);
static bool gpsMsg_checkForRmcDataValid(void);
// static void gpsMsg_dumpSentence(void);
// static void gpsMsg_isr(char rxByte);

static void enable_UART_tx(void);
static void enable_UART_rx(void);
static void disable_UART_tx(void);
static void disable_UART_rx(void);

/***************************
 * Module Public Functions
 **************************/

/**
* \brief Module init routine. Call once at startup.
* 
* \ingroup PUBLIC_API
*/
void gpsMsg_init(void) {
    gpsMsgData.isrRxIndex = 0;
    gpsMsgData.isrGotStart$ = false;
}

/**
* \brief GPS MSG executive. Called every 2 seconds from the main
*        loop to manage the GPS message processing.
*
* \ingroup EXEC_ROUTINE
*/
void gpsMsg_exec(void) {

    // printf("%s\n",__func__);

    // If we are NOT currently active - just return.
    if (!gpsMsgData.busy) {
        return;
    }

    if (gpsMsgData.rmcMsgFromIsrReady) {
        gpsMsgData.rmcMsgFromIsrReady = false;
        gpsMSg_processRmcSentence();
    }

    if (!gpsMsgData.rmcMsgAvailable  &&
        GET_ELAPSED_TIME_IN_SEC(gpsMsgData.waitForRmcTimestamp) > MAX_RMC_WAIT_TIME_IN_SEC) {
        gpsMsgData.noRmcMsgError = true;
    }

}

/**
* \brief Start GPS message processing
* 
* @return bool Returns true if message processing was started. 
*         Returns false if the processing had already been
*         started.
* 
* \ingroup PUBLIC_API
*/
bool gpsMsg_start(void) {

    // Return false if already busy.
    if (gpsMsgData.busy) {
        return false;
    }

    // printf("%s\n",__func__);

    // Mark module as busy
    gpsMsgData.busy = true;

    // Start processing messages from the GPS
    gpsMsg_isrRestart();

    return true;
}

/**
* \brief Stop GPS message processing
* 
* \ingroup PUBLIC_API
*/
void gpsMsg_stop(void) {
    disable_UART_tx();
    disable_UART_rx();
    gpsMsgData.busy = false;
}

/**
* \brief Get the message processing status
* 
* @return bool Returns true if message processing is in 
*         progress. False otherwise.
* 
* \ingroup PUBLIC_API
*/
bool gpsMsg_isActive(void) {
    return gpsMsgData.busy;
}

/**
* \brief Get the RMC message processing error status
* 
* @return bool Returns true if a timeout occured waiting for a 
*         RMC message from the GPS. False otherwise.
* 
* \ingroup PUBLIC_API
*/
bool gpsMsg_isError(void) {
    return gpsMsgData.noRmcMsgError;
}

/**
* \brief Identify if there is a RMC message available.
* 
* @return bool True if RMC message is available. False 
*         otherwise.
* 
* \ingroup PUBLIC_API
*/
bool gpsMsg_gotRmcMessage(void) {
    return gpsMsgData.rmcMsgAvailable;
}

/**
* \brief Return the length of the RMC message that is available.
* 
* @return uint8_t Length in bytes.
* 
* \ingroup PUBLIC_API
*/
uint8_t gpsMsg_getRmcMesssageLength(void) {
    return gpsMsgData.rmcMsgLength;
}

/**
* \brief Get the status of whether the RMC message that is 
*        available contains a valid satellite fix.
* 
* @return bool True if valid satellite fix. False otherwise.
* 
* \ingroup PUBLIC_API
*/
bool gpsMsg_gotDataValidRmcMessage(void) {
    return gpsMsgData.rmcDataIsValid;
}

/**
* \brief Retrieve the RMC message that is available.
* 
* @param bufP Buffer to copy the RMC message into.
* 
* \ingroup PUBLIC_API
*/
uint8_t gpsMsg_getRmcMessage(uint8_t *bufP) {
    uint8_t length = 0;
    // Copy the RMC string to the buffer
    memcpy(bufP, gpsRxBuf, gpsMsgData.rmcMsgLength);
    length = gpsMsgData.rmcMsgLength;
    return length;
}

/*************************
 * Module Private Functions
 ************************/

/**
* \brief Helper function to setup parameters and enable UART 
*        hardware interrupts to start a tx/rx transaction.
*/
static void gpsMsg_isrRestart(void) {
    uint8_t __attribute__((unused)) garbage;

    // For safety, disable UART interrupts
    disable_UART_rx();
    disable_UART_tx();

    gpsMsgData.noRmcMsgError = false;
    gpsMsgData.rmcMsgAvailable = false;
    gpsMsgData.rmcMsgLength = 0;
    gpsMsgData.rmcDataIsValid = false;

    gpsMsgData.isrRxIndex = 0;
    gpsMsgData.isrGotStart$ = false;
    gpsMsgData.rmcMsgFromIsrReady = false;

    // Clear out the UART receive buffer
    garbage = UCA0RXBUF;

    // Get time that we start waiting for RMC message
    gpsMsgData.waitForRmcTimestamp = GET_SYSTEM_TICK();

    // Enable interrupts
    enable_UART_rx();
    enable_UART_tx();
}

/**
* \brief Process a received RMC sentence. Verify the RMC message
*        against its checkusm. If the checksum matches, then we
*        are done until restarted. If not verified, restart the
*        sequence to receive another RMC message. If the data
*        valid is set in the RMC message, then set the
*        rmcDataIsValid flag.
*/
static void gpsMSg_processRmcSentence(void) {
    bool dataIsValid = false;
    if (gpsMsg_verifyChecksum()) {
        // printf("Got Valid RMC Message\n");
        gpsMsgData.rmcMsgAvailable = true;
        gpsMsgData.rmcMsgLength = gpsMsgData.isrRxIndex;

        dataIsValid = gpsMsg_checkForRmcDataValid();
        if (dataIsValid) {
            // printf("Got Satellite Fix\n");
            gpsMsgData.rmcDataIsValid = true;
        }
        // Only run until we receive one RMC message.
        gpsMsgData.busy = false;
    } else {
        // Keep running until we receive a RMC message
        gpsMsg_isrRestart();
    }
}

/**
* \brief Interrupt subroutine called when a character is 
*        received by the UART.
* 
* @param rxByte 
*/
void gpsMsg_isr(void) {

    uint8_t rxByte = UCA0RXBUF;

    // printf("%c", rxByte);
    // static int count;

    // Look for the start of a new NMEA sentence.
    if (rxByte == '$') {
        gpsMsgData.isrRxIndex = 0;
        gpsMsgData.isrGotStart$ = true;
    }

    // Continue processing the sentence if we found the starting delimiter.
    if (gpsMsgData.isrGotStart$) {

        // Add byte to the receive buffer
        if (gpsMsgData.isrRxIndex < RX_BUF_SIZE) {
            gpsRxBuf[gpsMsgData.isrRxIndex] = rxByte;
            gpsMsgData.isrRxIndex++;
        }

        // Check for LF character which marks the end of the NMEA sentence.
        if (rxByte == 0xA) {
            // printf("ISR: Got NMEA message %d\n", ++count);
            // gpsMsg_dumpSentence();
            // Reset flag
            gpsMsgData.isrGotStart$ = false;
            // Check if we might have a RMC message
            if (gpsMsg_matchRmc()) {
                // count = 0;
                // printf("ISR: Got RMC message\n");
                // gpsMsg_dumpSentence();
                // Disable interrutps and process the received NMEA sentence.
                disable_UART_rx();
                // Set flag to indicate RMC message is ready
                gpsMsgData.rmcMsgFromIsrReady = true;
            }
        }
    }
}

/**
* \brief Look for the data valid symbol in the RMC message. A 
*        data valid is noted by the 'A' at offset 18 from the
*        start of the message (base 0). The 'A' means we have
*        lock on enough satelites to get a good fix.
* 
* @return bool Returns true if the data-is-valid symbol is found 
*         in the RMC message.
*/
static bool gpsMsg_checkForRmcDataValid(void) {
#if 1
    //*******************************************************
    // For Test Only!!!! - Simulate RMC String
    static uint8_t testCount = 0;
    if (++testCount == 5) {
        testCount = 0;
        memcpy (gpsRxBuf, rmcTestString, sizeof(rmcTestString));
    }
    //*******************************************************
#endif
    return (gpsRxBuf[18] == 'A') ? true : false;
}

/**
* \brief Look for $GNRMC in the beginning of a NMEA message.
* 
* \notes Assumes an NMEA sentence is located in the rxBuf. 
* 
* @return bool Return true if found, false other wise.
*/
static bool gpsMsg_matchRmc(void) {
    int i;
    bool match = true;
    for (i = 0; i < sizeof(gnrmc_match_template); i++) {
        if (gpsRxBuf[i] != gnrmc_match_template[i]) {
            match = false;
        }
    }
    return match;
}

/**
* \brief Verify the checksum value in the NMEA sentence. The 
*        checksum field is an 8-bit exclusive OR of all the
*        bytes between the $ and the * (not including the
*        delimiters themselves). The checksum is calculated and
*        compared against the received value.
* 
* \notes Assumes the NMEA sentence is located in the rxBuf. 
*        Assumes the gps_data.index represents the length of the
*        NMEA sentence located in the rxBuf.
* 
* @return bool true if match, false otherwise.
*/
static bool gpsMsg_verifyChecksum(void) {
    int i;
    uint8_t calculatedChecksum = 0;
    uint8_t rxChecksumMSN = 0;
    uint8_t rxChecksumLSN = 0;
    uint8_t rxChecksum = 0;

    // Verify that we received a minimal length and the asterisk before the checksum.
    // The '*' (0x2A) will be at five characters from the end.
    // An example end of sentence: 0x2A,0x37,0x33,0xD,0xA
    if ((gpsMsgData.isrRxIndex < 20) || (gpsRxBuf[gpsMsgData.isrRxIndex - 5] != '*')) {
        // printf("\n\n**** INCOMPLETE MESSAGE ****\n\n");
        return false;
    }

    // Calculate the checksum over the NMEA sentence.
    // Exclude the first '$' and calculate up to the '*'.
    for (i = 1; i < (gpsMsgData.isrRxIndex - 5); i++) {
        calculatedChecksum ^= gpsRxBuf[i];
    }
    // printf("Calculated Checksum: 0x%x\n", calculatedChecksum);

    // Get the received checksum value as a hex number
    // The Most significant nibble of the checksum is 4 characters from the end.
    // The least significant nibble of the checksum is 3 characters from the end.
    rxChecksumMSN = asciiToHex(gpsRxBuf[gpsMsgData.isrRxIndex - 4]);
    rxChecksumLSN = asciiToHex(gpsRxBuf[gpsMsgData.isrRxIndex - 3]);
    rxChecksum = (rxChecksumMSN << 4) | rxChecksumLSN;
    // printf("Received Checksum: 0x%x\n", rxChecksum);

    // Check if they match
    if (calculatedChecksum != rxChecksum) {
        // printf("\n\n**** CHECKSUM MISMATCH ****\n\n");
    }

    return (calculatedChecksum == rxChecksum);
}

/**
* \brief Perform an ASCII to hex conversion for one ASCII 
*        character. Assumes the character is a hex number in
*        ASCII form. Returns the hex number.
*  
* \note Assumes the ASCII number will only be in uppercase form 
*       (A-F).
* 
* @param asciiByte An ASCII byte representing one hex number.
*  
* For reference - ASCII values 
* \li A = 65, 0x41
* \li G = 71, 0x47
* 
* @return uint8_t The hex number from the ASCII byte.
*/
static uint8_t asciiToHex(uint8_t asciiByte) {
    uint8_t val = 0;
    if (asciiByte > 0x2F && asciiByte < 0x40) {
        val = asciiByte - 0x30;
    } else if (asciiByte > 0x40 && asciiByte < 0x47) {
        val = 10 + (asciiByte - 0x41);
    }
    return val;
}

/*=============================================================================*/

static inline void enable_UART_tx(void) {
    UC0IE |= UCA0TXIE;
}

static inline void enable_UART_rx(void) {
    UC0IE |= UCA0RXIE;
}

static inline void disable_UART_tx(void) {
    UC0IE &= ~UCA0TXIE;
}

static inline void disable_UART_rx(void) {
    UC0IE &= ~UCA0RXIE;
}

/*=============================================================================*/

/*****************************
 * UART Interrupt Functions
 ****************************/
