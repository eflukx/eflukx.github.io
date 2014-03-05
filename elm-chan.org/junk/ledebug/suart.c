/*------------------------------------------------------------------------/
/  LPC2000 Software UART module
/-------------------------------------------------------------------------/
/
/  Copyright (C) 2011, ChaN, all right reserved.
/
/ * This software is a free software and there is NO WARRANTY.
/ * No restriction on use. You can use, modify and redistribute it for
/   personal, non-profit or commercial products UNDER YOUR RESPONSIBILITY.
/ * Redistributions of source code must retain the above copyright notice.
/
/-------------------------------------------------------------------------*/

#include "suart.h"
#include "interrupt.h"

/* I/O bit definitions for FRK-NXP-ARM */
#define SET_MARK()	FIO1SET2 = 0x04		/* Mark bit (LED OFF) */
#define SET_SPACE()	FIO1CLR2 = 0x04		/* Space bit (LED ON) */
#define	IS_MARK()	(FIO1PIN2 & 0x02)	/* Check if RXD is mark */

#define	INIT_PORT()	{ FIO1DIR2 |= 0x04; FIO1SET2 = 0x04; }	/* Set TXD port bit out */

#define	PCLK_TMR	18000000	/* PCLK freqency supplied to the timer module */
#define	BPS			115200		/* Bit rate (bps) */


void suart_init (void)
{
	INIT_PORT();

	_set_PCONP(PCTIM3, 1);					/* Enable TIMER3 module */
	_set_PCLKSEL(PCLK_TIMER3, PCLKDIV_4);	/* Set PCLK: 72MHz / 4 = 18MHz */

	/* Start TIMER3 as free running timer */
	T3IR = 0; T3PR = 0; T3PC = 0;
	T3MCR = 0; T3TCR = 1;
}



void suart_put (
	uint8_t chr
)
{
	uint32_t c;
	long t;


	c = (chr << 1) | 0x200;	/* <Stop>, <Data>, <Start> */
	IrqDisable();	/* Enter critical section */
	t = (long)T3TC;
	do {
		t += PCLK_TMR / BPS;		/* Delay a bit time */
		while (t >= (long)T3TC) ;
		if (c & 1) {				/* Set a bit to the LED */
			SET_MARK();
		} else {
			SET_SPACE();
		}
		c >>= 1;					/* Next bit */
	} while (c);
	IrqEnable();	/* Exit critical section */
}


