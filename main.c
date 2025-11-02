/* 
 * file: main.c
 * author: J. van Hooydonk
 * comments: main program
 *
 * revision history:
 *  v1.0 creation (22/06/2025)
 */

#include "config.h"
#include "general.h"

/**
 * main (start of program)
 */
void main(void)
{
    // init
    init();

    // main loop
    while (true)
    {
        // poll the free hall sensors on GPIO22 ... GPIO30 (not the detectors);
        pollFreeSensors();
    }
    return;
}
