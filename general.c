/*
 * file: general.c
 * author: J. van Hooydonk
 * comments: general variables, settings and routines
 *
 * revision history:
 *  v1.0 Creation (13/07/2025)
 */

#include "general.h"

// <editor-fold defaultstate="collapsed" desc="initialisation">

/**
 * initialisation
 */
void init()
{
    // init the LN driver and give the function pointer for the callback
    lnInit(&lnRxMessageHandler);
    // init a temporary LN message queue for transmitting a LN message
    initQueue(&lnTxMsg);
    // init hall sensor detection and give the function pointer for the callback
    hallSensorDetectionInit(&hallSensorDetectionHandler);
    // init of the hardware elements (ISR, ports, ...)
    initIsr();
    initPorts();
}

/**
 * initialisation of the interrupt service routine
 */
void initIsr(void)
{
    // set global interrupt parameters
    INTCONbits.IPEN = true; // enable priority levels on interrupt
    INTCONbits.GIEH = true; // enable all high priority interrupts
    INTCONbits.GIEL = true; // enable all low priority interrupts
}

/**
 * initialistaion of the IO pins (to read the DIP switch address) *
 */
void initPorts()
{
    // setup digital inputs to read the DIP switches
    // PORTA = A3 A2 -- --  -- -- A1 A0
    // PORTC = -- -- -- --  -- A6 A5 A4

    // we only need to read 7 DIP switches (A0 - A6)
    // this makes the adress A4 - A10 for the LN address selection
    // A11 is always = 0
    // A0 - A3 will be the index of the detectors (= 8 + 8 detectors)
    TRISA |= 0xc3; // disable output (= input) on pin A0 - A1, A6 - A7
    TRISC |= 0x07; // disable output (= input) on pin C0 - C2

    ANSELA &= 0x3c; // enable TTL input buffer on pin A0 - A1, A6 - A7
    ANSELC &= 0xf8; // enable TTL input buffer on pin C0 - C2

    WPUA |= 0xc3; // enable pull-up on pin A0 - A1, A6 - A7
    WPUC |= 0x07; // enabel pull-up on pin C0 - C2
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ISR high priority">

// there is one possible high interrupt triggers, coming from
// the interrupt on-change of the input pin edge detection

/**
 * high priority interrupt service routine
 */
void __interrupt(high_priority) isrHigh(void)
{
    if (PIR0bits.IOCIF)
    {
        // interrupt on-change input
        // IOCIF is a read-only bit; to clear the interrupt condition, 
        //  all bits in the IOCF register must be cleared 
        // check the hall sensor detection input(s)
        checkHallSensorDetectionInput(PORTB);
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="ISR low priority">

// there are two possible low interrupt triggers, coming from
// the EUSART data receiver and/or coming from the timer 1 overrun flag

/**
 * low priority interrupt service routine
 */
void __interrupt(low_priority) isrLow(void)
{
    if (PIE4bits.TMR1IE && PIR4bits.TMR1IF)
    {
        // timer 1 interrupt
        // clear the interrupt flag and handle the request
        PIR4bits.TMR1IF = false;
        lnIsrTmr1();
    }
    if (PIE3bits.TX1IE && PIR3bits.TX1IF)
    {
        // EUSART TX interrupt
        lnIsrTx();
    }
    if (PIE3bits.RC1IE && PIR3bits.RC1IF)
    {
         // EUSART RC interrupt
        if (RC1STAbits.FERR || RC1STAbits.OERR)
        {
            // OERR can be cleared by resetting the CREN bit
            RC1STAbits.CREN = false;
            RC1STAbits.CREN = true;
        }
        // handle the received data
        lnIsrRc();
    }
}

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="routines">

/**
 * callback function of the hall sensor detection handler
 * @param index: index of the sensor
 * @param value: value of the sensor
 * @param type: type of sensor (false = axle counter, true = free sensor)
 */
void hallSensorDetectionHandler(uint8_t index, bool value, bool type)
{
    // depending of the type create a 'wheel counter report' or
    // a 'general sensor input state report'
    // reference https://wiki.rocrail.net/doku.php?id=loconet:ln-pe-en
    //           https://wiki.rocrail.net/doku.php?id=loconet:lnpe-parms-en

    // get DIP switch address
    uint8_t address = getDipSwitchAddress();

    if (type)
    {
        // send 'wheel counter report'
        sendAxlCounterReport(index, address, value);
    }
    else
    {
        //
        // disable interrupts (comimg from axl detectors = interrupt driven)
        //  to prevent overwriting the LocoNet message
        di();
        // send 'sensor state report
        sendSensorStateReport(index, address, value);
        // reenable interrupts
        ei();
    }
}

/**
 * send the axle counter report over LocoNet
 * @param index: index of the axle counter
 * @param address: address of the DIP switch
 * @param value: value of the axle counter
 */
void sendAxlCounterReport(uint8_t index, uint8_t address, bool value)
{\
    // OPCODE = 0xE4 (OPC_AXLCNT_REP)
    // count = 0x08
    // Arg1 = 0x02
    // Arg2 = address high value
    // Arg3 = address low value
    // Arg4 = up count
    // Arg5 = down count

    // make Args
    uint16_t address16 = ((uint16_t)(address) << 3) + (uint16_t)(index) + 1;
    
    uint8_t opCode = 0xe4;
    uint8_t count = 0x08;
    uint8_t arg1 = 0x02;    
    uint8_t arg2 = (uint8_t)(address16 >> 7) & 0x07;
    uint8_t arg3 = (uint8_t)(address16) & 0x7f;
    uint8_t arg4 = (value) ? 0x01 : 0x00;
    uint8_t arg5 = (value) ? 0x00 : 0x01;
    
    // enqueue message
    enQueue(&lnTxMsg, opCode);
    enQueue(&lnTxMsg, count);
    enQueue(&lnTxMsg, arg1);
    enQueue(&lnTxMsg, arg2);
    enQueue(&lnTxMsg, arg3);
    enQueue(&lnTxMsg, arg4);
    enQueue(&lnTxMsg, arg5);
    // transmit the LN message
    lnTxMessageHandler(&lnTxMsg);    
}

/**
 * send the sensor state report over LocoNet
 * @param index: index of the sensor
 * @param address: address of the DIP switch
 * @param value: value of the sensor
 */
void sendSensorStateReport(uint8_t index, uint8_t address, bool value)
{
    // OPCODE = 0xB2 (OPC_INPUT_REP)
    // IN1 = sensor address
    //       0, A7, A6, A5, A4, A3, A2, A1
    //       (A1 - A2 = index bit 1 - index bit 2 of detection input)
    //       (A4 - A7 = DIP switches 1 - 4)
    // IN2 = sensor address and status
    //       0, X, I, L, A11, A10, A9, A8
    //       (A8 - A11 = DIP switches 5 - 8)
    //       (L: sensor level = state of gnd detection input)
    //       (I: A0 = index bit 0 of detection input)
    //       (X = 0)

    // make arguments IN1, IN2
    // the index can have a value between 0 and 7
    uint8_t opCode = 0xb2;
    uint8_t IN1 = (((address << 2) & 0x7c) + ((index >> 1) & 0x03)) & 0x7f;
    uint8_t IN2 = (address >> 5) & 0x03;
    // I: A0 (= index bit 0)
    if ((index & 0x01) == 0x01)
    {
        IN2 |= 0x20;
    }
    if (value)
    {
        IN2 |= 0x10;
    }

    // enqueue message
    enQueue(&lnTxMsg, opCode);
    enQueue(&lnTxMsg, IN1);
    enQueue(&lnTxMsg, IN2);
    // transmit the LN message
    lnTxMessageHandler(&lnTxMsg);
}


/**
 * callback function for the LN receiver
 * @param lnRxMsg: the lN message queue
 */
void lnRxMessageHandler(lnQueue_t* lnRxMsg)
{
    while (!isQueueEmpty(lnRxMsg))
    {
        // analyse the received LN message from queue
        switch (lnRxMsg->values[lnRxMsg->head])
        {
            case 0x82:
            {
                // global power OFF request
                break;
            }
            case 0x83:
            {
                // global power ON request
                break;
            }
        }
        // clear the received LN message from queue
        deQueue(lnRxMsg);
    }
}

/**
 * get the state of the DIP switches (0 - 9)
 * @return the address (or value of the DIP switches)
 */
uint8_t getDipSwitchAddress()
{
    // return the address
    // address = 0 0 0 0  0 0 A9 A8  A7 A6 A5 A4  A3 A2 A1 A0

    // we only need to read 7 DIP switches (A0 - A6)
    // this makes the address A4 - A10 for the complete LN address selection
    // A0 - A3 will be the index of the detector (= 8 + 8 detectors)
    uint8_t address;

    address = PORTA & 0x03; // A1 - A0 on port A, pin 0 - 1
    address += (PORTA >> 4) & 0x0c; // A3 - A2 on PORT A, pin 6 - 7
    address += (PORTC << 4) & 0x70; // A6 - A4 on PORT C, pin 0 - 3

    return address;
}

// </editor-fold>
