/* 
 * file: hallSensorDetection.h
 * author: J. van Hooydonk
 * comments: hall sensor detection for PCB LocoNet GPIO board v1.0
 *
 * revision history:
 *  v1.0 Creation (11/07/2025)
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef HALLSENSORDETECTION_H
#define	HALLSENSORDETECTION_H

#include "config.h"

// hall sensor detection callback definition (as function pointer)
typedef void (*hallSensorDetectionCallback_t)(uint8_t, bool, bool);

// variables
// hall sensor detection used variables
hallSensorDetectionCallback_t hallSensorDetectionCallback;
uint8_t hallSensorDetectionInputIndex;
uint8_t sectionDetector;
uint8_t freeSensorStates;

// routines
void hallSensorDetectionInit(hallSensorDetectionCallback_t);
void hallSensorDetectionInitInputs(void);
void hallSensorDetectionInitIsr(void);
void checkHallSensorDetectionInput(uint8_t);
void pollFreeSensors(void);

#endif	/* HALLSENSORDETECTION_H */
