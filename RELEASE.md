
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

