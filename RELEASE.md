
Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.6
June 12, 2018

Firmware Changes In This Release:

Redflag Processing
===================
Revamp the redflag processing per the firmware specification.
-Update mapping to 28 days
-redflag if daily liter sum < 0.25% of map value, cleared if > 75% of map value
-Send one notification, then return back to the standard transmission rate

Minor Changes
==============
-Compile Bootloader with optimizer set to 4 (max)
-Set default transmission rate to seven days

Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.05
May 16, 2018

Firmware Changes In This Release:

Change GPS Location Processing
===============================
Now look for and parse the GGA message.
Add new GPS parsing module (minmea.c/minmea.h).
Change the way a valid GPS measurement fix is identified.
Add new OTA message: Set GPS Measurement Criteria (0xE).
Change the format of the GPS data that is returned in the GPS Location Message.


Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.04
April 24, 2018

Firmware Changes In This Release:

Change Daily Water Log Message Format
======================================
Rework message to remove some of the pad diagnostic data not relevant anymore.

Change low frequency measurement window
========================================
Change low frequency measurement time window from 60 seconds to 20 seconds

Change message transmission time to 1:05 AM
============================================
Messages scheduled for transmission will be sent at 1:05 AM instead of 1:00 AM.


Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.03
April 10, 2018

Firmware Changes In This Release:

Add OTA GPS Request Message Support
====================================
Implemented logic to support the new GPS Request message. This message is sent by the IoT platform to request that
the sensor return the last GPS measurment data or to request that the Sensor take a new GPS measurement.

Add Bootloader Support 
=======================
Create a new CCS project to support the Bootloader. Also add the image-builder scripts that are used to combine the 
bootloader hex file with the application hex file. The combined file is used to program the MSP430 flash. The scripts 
are also used to create the firmware upgrade message that can be sent as an OTA message to the sensor.


Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.02
March 25, 2018

Firmware Changes In This Release:

New AfridevV2_MSP430_Water_Debug Project 
=========================================
Create a new CCS project to support water debug. This is a separate project that creates links to files in the 
AfridevV2_MSP430/src folder. This project is used to support IPS UART output for their water debug.


Afridev-V2 MSP430G2955 Firmware Release Notes
Version 0.01
March 24, 2018

Firmware Changes In This Release:

Initial Port of Afridev-V1B and Cascade to Afridev-V2 Complete 
===============================================================
The base firmware started with the Afridev-V1B code base. The Cascade firmware base was ported/merged-in to provide the
support for the MSP4302955 and also to bring in some of the key features that were added to Cascade. Including the more
robust firmware update design and transmission rate support.

Support for New Board
======================
The MSP430G2955 pin mappings and control was updated to support the new IPS board.

GPS Support Added 
==================
Support for the GPS was added. The firmware controls the GPS (turn on/ turn off) and captures the RMC messages sent
by the GPS to the MSP430 over the UART.

Modem Working
==================
The Modem is successfully communicating with the BodyTrace server

Multi Message Scheduling Added
===============================
A new feature of this release is the support to send multiple messages back-to-back from the modem. This was needed to
support sending a GPS message right after other messages, which include the Activation, Monthly, and Daily Water Log 
messages.

