/* 
 * file: circular_queue.h
 * author: J. van Hooydonk
 * comments: LocoNet driver
 *
 * revision history:
 *  v1.0 Creation (14/01/2024)
 */

#ifndef CIRCULAR_QUEUE_H
#define	CIRCULAR_QUEUE_H

#include "config.h"

// 128 bytes is the theoretical maximum length of a LN message
#define QUEUE_SIZE 128

typedef struct lnQueue_t {
    uint8_t head;
    uint8_t tail;
    uint8_t numEntries;
    uint8_t size;
    uint8_t values[QUEUE_SIZE];
} lnQueue_t;

void initQueue(lnQueue_t*);
bool isQueueEmpty(lnQueue_t*);
bool isQueueFull(lnQueue_t*);
bool enQueue(lnQueue_t*, uint8_t);
bool deQueue(lnQueue_t*);
void clearQueue(lnQueue_t*);

#endif	/* CIRCULAR_QUEUE_H */

