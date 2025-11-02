/* 
 * file: ln.h
 * author: J. van Hooydonk
 * comments: LocoNet driver
 *
 * revision history:
 *  v0.1 Creation (14/01/2024)
 *  v1.0 Merge PIC18F2525/2620/4525/4620 and PIC18F24/25/26/27/45/46/47Q10 microcontrollers (20/07/2024)
 *  v1.1 Remove PIC18F2525/2620/4525/4620 (obsolete processor)
 *  v2.0 complete rework of LocoNet driver after some major bugs
 */

// this is a guard condition so that contents of this file are not included
// more than once
#ifndef LN_H
#define	LN_H

#include "circular_queue.h"

// definitions
// a linebreak must take 15 bit times (see LocoNet specs)
// with a bautrate of 16.666 is this 60usec * 15 = 900usec
#define LINEBREAK_LONG 2500U
#define TIMER1_IDLE 2000U

typedef enum {
    IDLE,
    CMP,
    LINEBREAK,
    TX
} lnMode;

// LN flag register

typedef struct {
    lnMode LN_MODE;
} LNCON_t;
LNCON_t LNCON;

// LN RX message callback definition (as function pointer)
typedef void (*lnRxMsgCallback_t)(lnQueue_t*);

// LN init routines
void lnInit(lnRxMsgCallback_t);
void lnInitCmp1(void);
void lnInitEusart1(void);
void lnInitTmr1(void);
void lnInitIsr(void);
void lnInitLeds(void);

// LN timer 1 routines
void lnIsrTmr1(void);
void startIdleDelay(void);
void startCmpDelay(void);
void startLinebreak(void);
uint16_t getRandomValue(uint16_t);

// LN RX routines
void lnIsrRc(void);
void rxHandler(uint8_t);

// LN TX routines
void lnIsrTx(void);
void lnTxMessageHandler(lnQueue_t*);
void startLnTxMessage(void);
void sendTxByte(void);
void setTxMode(void);

// LN aux. routines
bool isLnFree(void);
void enableEusartTxPort(void);
void disableEusartTxPort(void);
bool isChecksumCorrect(lnQueue_t*);
void removeLastLnMessageFromQueue(lnQueue_t*);

// LN mode (IDLE, CMP, Linebreak, TX, ...)

// LN used variables
lnRxMsgCallback_t lnRxMsgCallback;
uint16_t lastRandomValue; // initial value for the random generator
uint8_t _; // dummy variable

lnQueue_t lnTxQueue;
lnQueue_t lnTxTempQueue;
lnQueue_t lnTxCompQueue;
lnQueue_t lnRxQueue;
lnQueue_t lnRxTempQueue;

#endif	/* LN_H */

