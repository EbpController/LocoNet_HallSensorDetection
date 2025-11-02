/* 
 * file: hallSensorDetection.c
 * author: J. van Hooydonk
 * comments: hall sensor detection for PCB LocoNet GPIO board v1.0
 *
 * revision history:
 *  v1.0 Creation (13/07/2025)
 */

/*
 * working principle
 * -----------------
 * 
 * in this project the detection of the train on the track will be performed
 *   with unipolar hall sensors
 * under the train/wagon, at the front and at the end, 2 magnets must be placed
 * 
 * a detection point is a group of 2 hall sensors
 * the hall sensors are placed in longitudinal direction and close together
 *   so that there is an overlap when a magnet affects the sensors
 * an interrupt is fired at the rising or falling edge of the first hall sensor
 * then the state of the second hall sensor is checked
 * the result is captured in a counter
 * there are 4 possible situation
 *  1. hall sensor 1 = falling edge and hall sensor 2 = true -> detector = 1
 *  2. hall sensor 1 = falling edge and hall sensor 2 = false -> detector = 0
 *  3. hall sensor 1 = rising edge and hall sensor 2 = false ->
 *      if detector = 0 then send LN sensor occupied message
 *  4. hall sensor 1 = rising edge and hall sensor 2 = true ->
 *      if detector = 1 then send LN sensor free message
 * 
 * all pins of port B and port D will be configurated as inputs
 * the pins of port B triggers an interrupt at the rising and falling edge
 *   of the input signal
 * 
 * pin 1 of port B works together with pin 1 of port D, this makes detector 1
 * pin 2 of port B works together with pin 2 of port D, this makes detector 2
 * ...
 * pin 8 of port B works together with pin 8 of port D, this makes detector 8
 */

#include "hallSensorDetection.h"
#include "ln.h"

/**
 * hall sensor detection initialisation
 * @param fptr: the function pointer to the (callback) ground detection handler
 */
void hallSensorDetectionInit(hallSensorDetectionCallback_t fptr)
{
    // initialise hall sensor detection callback function (function pointer)
    hallSensorDetectionCallback = fptr;
    // init of the hall sensor detection inputs
    hallSensorDetectionInitInputs();
    // init of interrupts to the hall sensor detection inputs
    hallSensorDetectionInitIsr();
}

/**
 * init of the hall detection input pins
 */
void hallSensorDetectionInitInputs()
{
    // for the hall sensors used as section-detection point we take
    // GPIO 0 ... GPIO 7 (= port D) and GPIO 8 ... GPIO 15 (= port B)
    // set all pins of PORTB to input with weak pull-up and disable analog mode
    TRISB = 0xff;
    WPUB = 0xff;
    ANSELB = 0x00;
    // set all pins of PORTD to input with weak pull-up and disable analog mode
    TRISD = 0xff;
    WPUD = 0xff;
    ANSELD = 0x00;
    
    // for the hall sensors used as free-sensor points we take
    // GPIO 18 and GPIO24 ... GPIO30
    // set the pins of the respondig port to input with weak pull-up and
    //  disable analog mode
    // GPIO18 = RA2
    TRISA |= 0x04;
    WPUA |= 0x04;
    ANSELA &= 0xfb;
    // GPIO24 = RC3 (SW7)
    // GPIO25 = RC4 (SW8)
    // GPIO26 = RC5 (SW9)
    TRISC |= 0x38;
    WPUC |= 0x38;
    ANSELC &= 0xc7;
    // GPIO27 = RE0
    // GPIO28 = RE1
    // GPIO29 = RE2
    // GPIO30 = RE3
    TRISE |= 0x0f;
    WPUE |= 0x0f;
    ANSELE &= 0xf0;
}

/**
 * init of the interrupt on-change of port B
 */
void hallSensorDetectionInitIsr()
{
    // set port B interrupt on-change and set it to high priority
    IPR0bits.IOCIP = true;      // interrupt on-change high priority
    PIE0bits.IOCIE = true;      // enable interrupt on-edge detection
    // enable interrupt on-change for negative and positive edges (all inputs)
    IOCBN = 0xff;
    IOCBP = 0xff;
 }

/**
 * check the hall sensor detection input(s) (after the interrupt on-change)
 */
void checkHallSensorDetectionInput(uint8_t portValues)
{
    // this function has been called after the (high priority) interrupt
    // from the on-change input was fired
    
    // determine which input has fired the interrupt
    for (uint8_t index = 0; index < 8; index++)
    {
        // scan inputs of port B
        if ((IOCBF & (1 << index)) != 0)
        {
            // check for a positive/negative edge
            if ((portValues & (1 << index)) != 0)
            {
                // on a positive edge we compare the input level at port D with
                //  the input level of port D (that was taken at the neg. edge) 
                if ((PORTD & (1 << index)) != (sectionDetector & (1 << index)))
                {
                    // if the result is false (the current status is different
                    //  from the previous status), then send the value of the
                    //  detector to the LocoNet bus
                    if ((sectionDetector & (1 << index)) == 0)
                    {
                        // send LocoNet message for a positive count
                        hallSensorDetectionCallback(index, true, true);
                    }
                    else
                    {
                        // send LocoNet message for a negative count
                        hallSensorDetectionCallback(index, false, true);
                    }
                }
            }
            else
            {
                // on a negative edge we store the input level of port D pin
                if ((PORTD & (1 << index)) == 0)
                {
                    sectionDetector &= ~(1 << index);
                }
                else
                {
                    sectionDetector |= (1 << index);
                }
            }
            // clear the interrupt flag
            IOCBF &= ~(1 << index);
        }
    }    
}

/**
 * poll the hall sensors used as free-sensor points
 */
void pollFreeSensors()
{
    uint8_t value = 0;
    
    // get the values of the hall sensors
    value |= (PORTC >> 3) & 0x07; // GPIO 24 - 26
    value |= (PORTE << 3) & 0x78; // GPIO 27 - 30
    value |= (PORTA << 5) & 0x80; // GPIO 18
    
    // compare value with previous state
    freeSensorStates ^= value;
    if (freeSensorStates != 0)
    {
        // if there is a difference determine which input has changed
        for (uint8_t index = 0; index < 8; index++)
        {
            if ((freeSensorStates & (1 << index)) != 0)
            {
                if ((value & (1 << index)) == 0)
                {
                    // send LocoNet message for a high level
                    hallSensorDetectionCallback(index, true, false);
                }
                else
                {
                    // send LocoNet message for a low level
                    hallSensorDetectionCallback(index, false, false);
                }
            }
        }
    }
    // store old state
    freeSensorStates = value;
}
