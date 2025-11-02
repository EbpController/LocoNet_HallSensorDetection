The hardware used for this project (LocoNet_HallSensorDetection) is a 'LocoNet GPIO board'.

This project is build on the LocoNet driver software for the PIC18F24/25/26/27/45/46/47Q10 microcontroller family.
The code is written in C and is compatible to the "[LocoNet Personal Use Edition 1.0 SPECIFICATION](https://www.digitrax.com/static/apps/cms/media/documents/loconet/loconetpersonaledition.pdf)" from DigiTrax Inc.

The LocoNet driver uses the EUSART 1 and the Timer 1, both with a low priority interrupt.

The following hardware pins on the microcontroller are used:
 - RA3: comparator 1, non-inverting input (C1IN+)
 - RA4: comparator 1, output (C1OUT)
 - RC6: LocoNet transmitter (EUSART 1, TXD)
 - RC7: LocoNet receiver (EUSART 1, RXD)

The pins RA5 is used as indication LED: data trafic on LocoNet

The locoNet driver is built in the files: ln.h, ln.c, circular_queue.h and circular_queue.c
Include this library (files) into your (LocoNet) project.
 - To transmit a LocoNet message, the function lnTxMessageHandler(lnMessage*) can be invoked.
 - To receive a LocoNet message, a lnRxMessageHandler(lnMessage*) callback function must be included.

There are 2 functions available
 - counter function (with direction information)
 - position function

The counter function uses 2 hall sensors. There are maximum 8 counters. For this function, 2 hall sensors must be placed near each othor, so that an overlap of the pulse can be detected. The first sensor must be conncted to GPIO n (PORT D, pin n), the second sensor must be connected to GPIO n+8 (PORT B, pin n), where GPIO is the name of the GPIO pin on the 'LocoNet GPIO board' an n is the number of the counter (0 to 7). The LocoNet message for a counter starts with OpCode 0xE4 (OPC_AXLCNT_REP). The LocoNet message is as follow:
	- OPCODE = 0xE4 (OPC_AXLCNT_REP)
	- count = 0x08
	- Arg1 = 0x02
	- Arg2 = address high value
	- Arg3 = address low value
	- Arg4 = up count
	- Arg5 = down count

The position function makes the detection for just one hall sensor. There are maximum 8 position detectors. The sensor must be connected to GPIO 24 - GPIO 30 (= sensor 0 - 6) and GPIO 18 (= sensor 7). The LocoNet message is as follow:
    - OPCODE = 0xB2 (OPC_INPUT_REP)
    - IN1 = sensor address
      - 0, A7, A6, A5, A4, A3, A2, A1
      - (A1 - A2 = index bit 1 - index bit 2 of detection input)
      - (A4 - A7 = DIP switches 1 - 4)
    - IN2 = sensor address and status
      - 0, X, I, L, A11, A10, A9, A8
      - (A8 - A11 = DIP switches 5 - 8)
      - (L: sensor level = state of gnd detection input)
      - (I: A0 = index bit 0 of detection input)
      - (X = 0)
