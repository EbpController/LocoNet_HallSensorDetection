/*
 * file: ln.c
 * author: J. van Hooydonk
 * comments: LocoNet driver
 *
 * revision history:
 *  v0.1 Creation (14/01/2024)
 *  v1.0 Merge PIC18F2525/2620/4525/4620 and PIC18F24/25/26/27/45/46/47Q10 microcontrollers (20/07/2024)
 *  v1.1 Remove PIC18F2525/2620/4525/4620 (obsolete processor)
 *  v2.0 complete rework of LocoNet driver after some major bugs
 */

#include "ln.h"

// <editor-fold defaultstate="collapsed" desc="LN init routines">

/**
 * LN initialisation
 * @param fptr: the function pointer to the (callback) LN RX message handler
 */
void lnInit(lnRxMsgCallback_t fptr)
{
    // init LN RX message callback function (function pointer)
    lnRxMsgCallback = fptr;

    // declaration and initialisation of the RX and TX queue
    // essentially the queue is just a pointer to the instance of the struct
    initQueue(&lnTxQueue);
    initQueue(&lnTxTempQueue);
    initQueue(&lnRxQueue);
    initQueue(&lnRxTempQueue);
    initQueue(&lnTxCompQueue);

    // init of the other elements (clock, comparator, EUSART, timer, ISR, leds)
    lnInitCmp1();
    lnInitEusart1();
    lnInitTmr1();
    lnInitIsr();
    lnInitLeds();

    // set LN at startup in IDLE mode
    startIdleDelay();
}

/**
 * LN initialisation of the comparator input circuit
 */
void lnInitCmp1(void)
{
    // set pins for CMP 1
    ANSELAbits.ANSELA3 = true; // PORT A, pin 0 = (analog) input, CMP 1 IN+
    TRISAbits.TRISA3 = true;
    TRISAbits.TRISA4 = false; // PORT A, pin 4 = output, CMP 1 OUT
    // use the fixed voltage reference to feed the Vin-
    // refer to PIC18FxxQ10 datasheet 'FVR (fixed voltage reference)'
    FVRCON = 0x0c; // CDAFVR buffer gain is 4x (4.096V))
    FVRCONbits.FVREN = true; // enable FVR
    // wait till FVR is ready to use
    while (!FVRCONbits.FVRRDY)
    {
        NOP();
    }
    // refer to PIC18FxxQ10 datasheet 'pin allocation tables' and 'CMP module'
    CM1NCH = 0x06; // CMP 1 Vin- = FVR
    CM1PCH = 0x01; // CMP 1 Vin+ = RA3 (CxIN1+)
    // refer to PIC18FxxQ10 datasheet 'PPS module' and 'CMP module'
    RA4PPS = 0x0d; // CMP 1 Vout = RA4 (CxOUT))
    // refer to PIC18FxxQ10 datasheet 'slew rate control'
    SLRCONAbits.SLRA4 = true; // set pin to limited slew rate

    CM1CON0bits.EN = true; // enable CMP1
}

/**
 * LN initialisation of the EUSART to baudrate of 16.666
 */
void lnInitEusart1(void)
{
    // set pins for EUSART 1 RX and TX
    TRISCbits.TRISC6 = false; // PORT C, pin 6 = LN TX
    ANSELCbits.ANSELC6 = false;
    PORTCbits.RC6 = false;
    TRISCbits.TRISC7 = true; // PORT C, pin 7 = LN RX
    ANSELCbits.ANSELC7 = false;
    // refer to PIC18FxxQ10 datasheet 'PPS module' and 'CMP module'
    // connect EUSART 1 TX to RC6 only if we have something to send (in TX mode)
    // connect EUSART 1 RX to RC7
    RX1PPS = 0x17;
    // configure EUSART 1
    BAUD1CONbits.SCKP = true; // invert TX output signal
    BAUD1CONbits.BRG16 = false; // 16-bit baudrate generator
    TX1STAbits.BRGH = false; // low speed    
    TX1STAbits.SYNC = false; // asynchronous mode

    // set the BRG
    // desired baudrate = 16.666
    // BRG value = (64.000.000 / (64 x 16.666)) - 1 = 59 (0x3B)
    // calculated baudrate = 64.000.000 / (64 x (59 + 1)) = 1.666,666667
    // error = (1.666,666667 - 1.666) / 1.666 = 0.04 %
    SP1BRG = 59U;
    // enable the BRG
    TX1STAbits.TXEN = true;
    RC1STAbits.CREN = false; // first clear bit CREN to clear the OERR bit
    RC1STAbits.CREN = true; // enable receiver
    RC1STAbits.SPEN = true; // enable serial port
    // read the receive register to clear his content and to clear the FERR bit
    _ = RC1REG;
}

/**
 * LN initialisation of the timer 1
 */
void lnInitTmr1(void)
{
    TMR1H = 0x00; // reset timer 1
    TMR1L = 0x00;
    TMR1CLK = 0x01; // clock source to Fosc / 4
    T1CON = 0b00110000; // T1CKPS = 0b11 (1:8 prescaler), RD16 = 0 (8 bit mode)
}

/**
 * LN initialisation of the interrupt service routine
 */
void lnInitIsr(void)
{
    IPR3bits.RC1IP = false; // EUSART 1 RXD interrupt low priority
    IPR3bits.TX1IP = false; // EUSART 1 TXD interrupt low priority
    IPR4bits.TMR1IP = false; // timer 1 interrupt low priority
    INTCONbits.IPEN = true; // enable priority levels on iterrupt
    INTCONbits.GIEH = true; // enable all high priority interrupts
    INTCONbits.GIEL = true; // enable all low priority interrupts
    PIE3bits.RC1IE = true; // enable EUSART 1 RXD interrupt
    // by init disable these interrupts
    PIE3bits.TX1IE = false; // disable EUSART 1 TXD interrupt
    PIE4bits.TMR1IE = false; // disable timer 1 overflow interrupt
    T1CONbits.TMR1ON = false; // disabled timer 1
}

/**
 * LN initialisation of the leds
 */
void lnInitLeds(void)
{
    TRISAbits.TRISA5 = false; // A5 as output
    LATAbits.LATA5 = false; // led 'data on LN' off (active high)
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LN timer 1 routines">

/**
 * interrupt routine for timer 1
 */
void lnIsrTmr1(void)
{
    switch (LNCON.LN_MODE)
    {
        case IDLE:
            // LN driver is in idle mode (no TX or RX)
            if (isLnFree())
            {
                // LN is free
                if (!isQueueEmpty(&lnTxQueue))
                {
                    // if LN TX queue has a LN message 
                    startLnTxMessage();
                }
                else
                {
                    // LN is free but has nothing to transmit
                    // restart timer 1 with idle delay
                    startIdleDelay();
                }
            }
            else
            {
                // LN is not free, so start timer 1 with CMP delay
                startCmpDelay();
            }
            break;
        case CMP:
            // after the CMP delay
            if (isLnFree())
            {
                // if LN line is free start timer 1 with idle delay
                startIdleDelay();
            }
            else
            {
                // if LN line is not free restart timer 1 with CMP delay
                startCmpDelay();
            }
            break;
        case LINEBREAK:
            // after the linebreak reset the output pin (no linebreak)
            PORTCbits.RC6 = false;
            // and start CMP delay
            startCmpDelay(); // start the timer 1 with CMP delay
            break;
        case TX:
            break;
        default:
            break;
    }
}

/**
 * start of the idle delay (1000탎)
 */
void startIdleDelay(void)
{
    // delay = 1000탎 (timer 1 in idle mode)
    TMR1H = (uint8_t) (~TIMER1_IDLE >> 8); // set delay in timer 1
    TMR1L = (uint8_t) (~TIMER1_IDLE & 0x00ff);
    // WRITETIMER1(~TIMER1_IDLE); // set delay in timer 1
    // disable TX and enable timer interrupts
    PIE3bits.TX1IE = false; // disable EUSART 1 TXD interrupt
    PIE4bits.TMR1IE = true; // enable timer 1 overflow interrupt
    T1CONbits.TMR1ON = true; // enable timer 1
    // set device in IDLE mode
    LNCON.LN_MODE = IDLE;
    // in idle mode, the leds on LN (RX + TX) can be turned off (active high)
    LATAbits.LATA5 = false;
}

/**
 * start of the carrier + master + priority delay
 */
void startCmpDelay(void)
{
    // disable EUSART (TX) port
    disableEusartTxPort();
    // delay CMP = 1200탎 + 360탎 + random (between 0탎 and 1023탎)
    uint16_t delay = getRandomValue(lastRandomValue);
    lastRandomValue = delay; // store last value of random generator
    delay &= 2047U; // get random value between 0 and 1023
    delay += 3120U; // add C + M delay (= 1560탎)
    TMR1H = (uint8_t) (~delay >> 8); // set delay in timer 1
    TMR1L = (uint8_t) (~delay & 0x00ff);
    // WRITETIMER1(~delay); // set delay in timer 1
    // disable TX and enable timer interrupts
    PIE3bits.TX1IE = false; // disable EUSART 1 TXD interrupt
    PIE4bits.TMR1IE = true; // enable timer 1 overflow interrupt
    T1CONbits.TMR1ON = true; // enable timer 1
    // set device in CMP mode
    LNCON.LN_MODE = CMP;
    // led 'data on LN' on (active high)
    LATAbits.LATA5 = true;
}

/**
 * start of the linebreak delay (with a well defined time)
 * @return 
 */
void startLinebreak(void)
{
    // linebreak detect by framing error
    // disable EUSART (TX) port and force a linebreak
    disableEusartTxPort();
    PORTCbits.RC6 = true;
    // setup the LN linebreak period 
    TMR1H = (uint8_t) (~LINEBREAK_LONG >> 8); // set delay in timer 1
    TMR1L = (uint8_t) (~LINEBREAK_LONG & 0x00ff);
    // WRITETIMER1(~LINEBREAK_LONG);
    // disable TX and enable timer interrupts
    PIE3bits.TX1IE = false; // disable EUSART 1 TXD interrupt
    PIE4bits.TMR1IE = true; // enable timer 1 overflow interrupt
    T1CONbits.TMR1ON = true; // enable timer 1
    // set device in LINEBREAK mode
    LNCON.LN_MODE = LINEBREAK;
}

/**
 * random generator with Galois shift register
 * @param lfsr: initial value for the shift register
 * @return a 16 bit random vaule
 */
uint16_t getRandomValue(uint16_t startState)
{
    // with the help of a Galois LFSR (linear-feedback shift register)
    // we can get a random number. The LFSR is a shift register whose
    // input bit is a linear function of its previous state
    // refer: https://en.wikipedia.org/wiki/Linear-feedback_shift_register

    if (startState == 0)
    {
        startState = 0xace1; // start state
    }

    uint16_t lfsr = startState;
    do
    {
        uint16_t lsb = lfsr & 1u; // get LSB (i.e. the output bit)

        lfsr >>= 1; // shift register to right
        if (lsb) // if the output bit is 1
        {
            lfsr ^= 0xb400u; // apply toggle mask
        }
    }
    while (lfsr == startState);

    return lfsr;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LN RX routines">

/**
 * interrupt routine for EUSART RX
 */
void lnIsrRc(void)
{
    // read the data from RX buffer
    uint8_t lnRxData = RC1REG;
    
    if (LNCON.LN_MODE == TX)
    {
        // device is in TX mode
        // check if received byte = last transmitted byte
        if (lnRxData == lnTxCompQueue.values[lnTxCompQueue.head])
        {
            deQueue(&lnTxCompQueue);
            if (isQueueEmpty(&lnTxCompQueue))
            {
                // now we are sure that the LN message is well transmitted
                // at this point we could remove the last transmitted LN
                // message from the TX queue
                removeLastLnMessageFromQueue(&lnTxQueue);
                // restart CMP delay
                startCmpDelay();
            }
            // handle the received byte
            rxHandler(lnRxData);
        }
        else
        {
            // if LN RX data is not equal to LN TX data send linebreak
            startLinebreak();
        }
    }
    else if (LNCON.LN_MODE != LINEBREAK)
    {
        // restart CMP delay
        startCmpDelay();
        // device is in RX mode (receive LN message)
        rxHandler(lnRxData);
    }
}

/**
 * EUSART RX handler
 * @param lnRxData: the received databyte
 */
void rxHandler(uint8_t lnRxData)
{
    // start testing if msb = 1 (this is the startbyte of the LN message)
    if ((lnRxData & 0x80) == 0x80)
    {
        clearQueue(&lnRxTempQueue);
        enQueue(&lnRxTempQueue, lnRxData);
    }
    else
    {
        enQueue(&lnRxTempQueue, lnRxData);

        // determine length of LN message
        uint8_t lnMessageLength;
        lnMessageLength = (lnRxTempQueue.values[lnRxTempQueue.head] & 0x60);
        lnMessageLength = (lnMessageLength >> 4) + 2;
        if (lnMessageLength > 6)
        {
            lnMessageLength = lnRxTempQueue.values[(lnRxTempQueue.head + 1) % lnRxTempQueue.size];
        }

        // has LN message reached the end the test checksum
        if (lnMessageLength == lnRxTempQueue.numEntries)
        {
            if (isChecksumCorrect(&lnRxTempQueue))
            {
                // if checksum is correct then copy LN RX temp queue to
                // LN RX queue
                while (!isQueueEmpty(&lnRxTempQueue))
                {
                    enQueue(&lnRxQueue, lnRxTempQueue.values[lnRxTempQueue.head]);
                    deQueue(&lnRxTempQueue);
                }
                // handle LN RX message (in the callback function)
                (*lnRxMsgCallback)(&lnRxQueue);
            }
        }
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LN TX routines">

/**
 * interrupt routine for EUSART TX
 */
void lnIsrTx(void)
{
    // send ln data as long queue is not empty
    if (!isQueueEmpty(&lnTxTempQueue))
    {
        sendTxByte();
    }
}

/**
 * start routine for transmitting a LN message
 * @param lnTxMsg: the message to transmit
 */
void lnTxMessageHandler(lnQueue_t* lnTxMsg)
{
    // copy the LN message into the LN TX queue
    // and add the calculated checksum
    uint8_t checksum = 0x00;

    while (!isQueueEmpty(lnTxMsg))
    {
        checksum ^= lnTxMsg->values[lnTxMsg->head];
        enQueue(&lnTxQueue, lnTxMsg->values[lnTxMsg->head]);
        deQueue(lnTxMsg);
    }
    enQueue(&lnTxQueue, (checksum ^ 0xff));
}

/**
 * begin of routine for transmitting a LN message
 */
void startLnTxMessage(void)
{
    // this routine is driven by (timer) interrupt, so don't call it directly !
    // set the device in TX mode
    setTxMode();
    // clear temporary & comparator queue
    clearQueue(&lnTxTempQueue);
    clearQueue(&lnTxCompQueue);
    // copy the LN message from LN TX queue into the LN TX temporary queue
    // get bytes till queue is empty or next byte starts with 0x80 (msb = 1)
    uint8_t pointer = lnTxQueue.head;
    do
    {
        enQueue(&lnTxTempQueue, lnTxQueue.values[pointer]);
        pointer = (pointer + 1) % lnTxQueue.size;
    }
    while ((pointer != lnTxQueue.tail) &&
            ((lnTxQueue.values[pointer] & 0x80) != 0x80));
    // last check is LN bus is free
    if (isLnFree())
    {
        // setup EUSART TX mode
        enableEusartTxPort();
        // if free, start sending the first byte
        sendTxByte();        
     }
    else
    {
        // if not free, restart CMP delay
        startCmpDelay();
    }
}

/**
 * transmit next byte 
 */
void sendTxByte(void)
{
    // transmit the byte
    TX1REG = lnTxTempQueue.values[lnTxTempQueue.head];
    // the last transmitted byte must be stored in the comparator queue
    // this is necessary to check if the data is transmitted correctly
    // (see routine lnIsrRc)
    enQueue(&lnTxCompQueue, lnTxTempQueue.values[lnTxTempQueue.head]);
    // dequeue TX queue
    deQueue(&lnTxTempQueue);
}

/**
 * set EUSART in TX mode
 */
void setTxMode(void)
{
    // enable TX and disable timer interrupts
    PIE3bits.TX1IE = true; // enable EUSART 1 TXD interrupt
    PIE4bits.TMR1IE = false; // disable timer 1 overflow interrupt
    T1CONbits.TMR1ON = false; // disable timer 1
    // set device in LN TX mode
    LNCON.LN_MODE = TX;
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="LN aux. routines">

/**
 * check if the LocoNet is free (to use)
 * @return true: if the line is free, false: if the line is occupied
 */
bool isLnFree(void)
{
    // check if:
    //  RCIDL == 1 (receiver is idle when no data reception is progressing)
    //  RC1IF == 1 (receiver has no remaining data in RX buffer)
    return (BAUD1CONbits.RCIDL && !PIR3bits.RC1IF);
}

/**
 * enable the EUSART TX port
 */
void enableEusartTxPort(void)
{
    // connect EUSART 1 TX to RC6
    RC6PPS = 0x09;
}

/**
 * disable the EUSART TX port
 */
void disableEusartTxPort(void)
{
    // deconnect EUSART 1 TX from RC6, return to normal IO pin
    RC6PPS = 0x00;
}

/**
 * calculate the checksum
 * @param lnQueue: name of the queue (pass the address of the queue)
 * @return true: if checksum is correct 
 */
bool isChecksumCorrect(lnQueue_t* lnQueue)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < lnQueue->numEntries; i++)
    {
        checksum ^= lnQueue->values[(lnQueue->head + i) % lnQueue->size];
    }
    return (checksum == 0xff);
}

/**
 * remove the last LN message from the queue
 * @param lnQueue: the queue where the LN message has to be removed
 */
void removeLastLnMessageFromQueue(lnQueue_t* lnQueue)
{
    // remove bytes till queue is empty or next bytes starts with 0x80 (msb = 1)
    if (!isQueueEmpty(lnQueue))
    {
        do
        {
            deQueue(lnQueue);
        }
        while (!isQueueEmpty(lnQueue) &&
                ((lnQueue->values[lnQueue->head] & 0x80) != 0x80));
    }
}

// </editor-fold>
