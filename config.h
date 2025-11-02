/* 
 * file: config.h
 * author: J. van Hooydonk
 * comments: configuration bit settings
 *
 * revision history:
 *  v1.0 Creation (21/07/2024)
 */

// this is a guard condition so that contents of this file are not included
// more than once.
#ifndef CONFIG_H
#define	CONFIG_H

    // #include <pic18f46q10.h>
    #include <xc.h>                 // include processor files - each processor file is guarded.
    #include <stdbool.h>
    #include <stdint.h>

    unsigned long _XTAL_FREQ = 64000000UL;

    // PIC18F46Q10 Configuration Bit Settings

    // CONFIG1L
    #pragma config FEXTOSC = OFF    // External Oscillator mode Selection bits (Oscillator not enabled)
    #pragma config RSTOSC = HFINTOSC_64MHZ // Power-up default value for COSC bits (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

    // CONFIG1H
    #pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled)
    #pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
    #pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

    // CONFIG2L
    #pragma config MCLRE = INTMCLR  // Master Clear Enable bit (If LVP = 0, MCLR pin (RE3) is an input; If LVP =1, MCLR pin (RE3) is MCLR)
    #pragma config PWRTE = OFF      // Power-up Timer Enable bit (Power up timer disabled)
    #pragma config LPBOREN = OFF    // Low-power BOR enable bit (Low power BOR is disabled)
    #pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

    // CONFIG2H
    #pragma config BORV = VBOR_190  // Brown Out Reset Voltage selection bits (Brown-out Reset Voltage (VBOR) set to 1.90V)
    #pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
    #pragma config PPS1WAY = OFF    // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be set and cleared repeatedly (subject to the unlock sequence))
    #pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
    #pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

    // CONFIG3L
    #pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
    #pragma config WDTE = OFF       // WDT operating mode (WDT Disabled)

    // CONFIG3H
    #pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
    #pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

    // CONFIG4L
    #pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
    #pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
    #pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
    #pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

    // CONFIG4H
    #pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
    #pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
    #pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
    #pragma config SCANE = ON       // Scanner Enable bit (Scanner module is available for use, SCANMD bit can control the module)
    #pragma config LVP = OFF        // HV on MCLR/VPP must be used for programming

    // CONFIG5L
    #pragma config CP = OFF         // UserNVM Program Memory Code Protection bit (UserNVM code protection disabled)
    #pragma config CPD = OFF        // DataNVM Memory Code Protection bit (DataNVM code protection disabled)

    // CONFIG5H

    // CONFIG6L
    #pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
    #pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

    // CONFIG6H
    #pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

#endif /* CONFIG_H */
