/* 
 * file: general.h
 * author: J. van Hooydonk
 * comments: general variables, settings and routines 
 *
 * revision history:
 *  v1.0 Creation (13/07/2025)
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GENERAL_H
#define	GENERAL_H

// include libraries

#include "ln.h"
#include "hallSensorDetection.h"

// routines
void init(void);
void initIsr(void);
void initPorts(void);

void isrLow(void);
void isrHigh(void);
void lnRxMessageHandler(lnQueue_t*);
void hallSensorDetectionHandler(uint8_t, bool, bool);
void sendAxlCounterReport(uint8_t, uint8_t, bool);
void sendSensorStateReport(uint8_t, uint8_t, bool);
uint8_t getDipSwitchAddress(void);

// variables
lnQueue_t lnTxMsg;

#endif	/* GENERAL_H */

