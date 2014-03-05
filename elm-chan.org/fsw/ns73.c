/*----------------------------------------------------------------------------/
/  NS73 control library                                 R0.01 (C)ChaN, 2008
/-----------------------------------------------------------------------------/
/  The NS73 control library is to control NS73M FM transmitter module.
/
/  Copyright (C) 2008, ChaN, all right reserved.
/
/ * The NS73 contorl library is a free software and there is no warranty.
/ * You can use, modify and/or redistribute it for personal, non-profit or
/   commercial use without restriction under your responsibility.
/ * Redistributions of source code must retain the above copyright notice.
/
/-----------------------------------------------------------------------------/
/ Aug 10,'08 R0.01  First release.
/-----------------------------------------------------------------------------*/

int ns73_start(
	unsigned long freq,		/* Frequency [Hz] */
	int stereo,				/* 0:Mono, 1:Stereo */
	int power				/* 1:1mW, 2:2mW */
);

void ns73_stop(void);



/*--------------------------------------------------------------------------*/
/* Module private functions and macros                                      */
/*--------------------------------------------------------------------------*/

static
unsigned long CEX[4];	/* PLL range: Highest, High, Mid and Checksum */

/*--------------------------------------------------------------------------*/
/* These macros are platform dependent. You must implement corresponding    */
/* function to the following macros.                                        */

#define	LOAD_CEX()			/* Fill CEX[] with saved data */
#define	SAVE_CEX()			/* Save CEX[] */
#define	DELAY_MS(ms)		/* Wait for ms [ms] */
#define NS73_TEB			/* TEB pin status, 0:Low, 1:High */
#define	NS73_PUT(reg,dat)	/* Write a byte into NS73 register */

/*--------------------------------------------------------------------------*/



static
int ns73_teb (void)
{
	int n, c;

	for (n = c = 0; c < 50; c++) {
		if (NS73_TEB) n++;
		DELAY_MS(10);
	}
	return (n >= 25) ? 1 : 0;
}



static
void ns73_setfreq (unsigned long f)
{
	f /= 8192;
	NS73_PUT(3, (unsigned char)f);
	NS73_PUT(4, (unsigned char)(f >> 8));
}



static
void ns73_chkfreqvalid (unsigned long f)
{
	if (f > 95000000UL) {
		ns73_setfreq(95000000UL);
		DELAY_MS(1);
	}
	ns73_setfreq(f);
}



static
int ns73_getpllrange (void)
{
	unsigned long f;
	int i;

	/* Load stored table */
	LOAD_CEX();
	for (f = 0x12345678, i = 0; i < 3; i++) f += CEX[i];
	if (f == CEX[i]) return 1;		/* Return if the stored table is valid */

	/* Create CEX table (This takes a time) */
	f = 87500000UL;					/* Lowest frequency for NS73M-US */
	i = 3; NS73_PUT(8, 0x18 | i);	/* Select low PLL range */

	do {							/* Course search for lock range */
		ns73_chkfreqvalid(f);
		if (!ns73_teb()) {			/* Found an upper limit in this range */
			f -= 1000000UL;
			do {					/* Fine search for upper limit */
				f += 100000UL;
				ns73_chkfreqvalid(f);
			} while (ns73_teb());
			ns73_chkfreqvalid(f - 2000000UL);	/* Check range overwrapping */
			i--; NS73_PUT(8, 0x18 | i);			/* Select next higher range */
			if (!ns73_teb()) return 0;			/* Error when insufficient overwrap */
			CEX[i] = f - 1000000UL;				/* Store the base frequency of the range */
		}
		f += 1000000UL;
	} while (i);

	/* Save created table */
	for (f = 0x12345678, i = 0; i < 3; i++) f += CEX[i];
	CEX[i] = f;
	SAVE_CEX();
	return 1;
}



/*--------------------------------------------------------------------------*/
/* Public functions                                                         */
/*--------------------------------------------------------------------------*/

int ns73_start (
	unsigned long freq,		/* Frequency [Hz] */
	int stereo,				/* 0:Mono, 1:Stereo */
	int power				/* 1:1mW, 2:2mW */
)
{
	unsigned long f;
	int i;


	/* Check frequency range */
	if (freq < 87500000UL || freq > 108000000UL) return 0;
	freq += 304000UL;

	NS73_PUT(14, 0x05);		/* Software reset */
	NS73_PUT(1, 0xB4);
	NS73_PUT(2, 0x06);
	NS73_PUT(5, 0);
	NS73_PUT(6, 0x1E);		/* CIA=3, CIB=3 */
	NS73_PUT(7, 0);
	NS73_PUT(8, 0x1A);		/* CEX=2 */
	NS73_PUT(9, 0);
	NS73_PUT(10, 0);
	NS73_PUT(11, 0);
	NS73_PUT(0, 0x05);		/* Power ON, Mute */
	NS73_PUT(14, 0x05);		/* Software reset */

	if (!ns73_getpllrange()) {	/* Create CEX table */
		NS73_PUT(0, 0x00);		/* Power OFF */
		return 0;				/* Function failed */
	}

	if (freq >= 95000000UL) {
		ns73_setfreq(95000000UL);
		NS73_PUT(8, 0x19);			/* Select high PLL range */
		DELAY_MS(500);
		if (freq >= 104000000UL) {
			f = 104000000UL;
			ns73_setfreq(f);
			NS73_PUT(8, 0x18);		/* Select highest PLL range */
			DELAY_MS(500);
			while (f < freq) {		/* Pull-up frequency slowly */
				f += 100000UL;
				ns73_setfreq(f);
				DELAY_MS(20);
			}
		}
	}

	/* Select true PLL range */
	ns73_setfreq(freq);
	for (i = 0; i < 3 && freq < CEX[i]; i++);
	NS73_PUT(8, 0x18 | i);
	DELAY_MS(100);

	NS73_PUT(6, 0x1A);						/* CIA=1, CIB=3 */
	NS73_PUT(1, stereo ? 0xB4 : 0xFC);		/* Select mono or stereo */
	NS73_PUT(2, power == 2 ? 0x07 : 0x06);	/* Select output power 1mW or 2mW */
	NS73_PUT(0, 0x01);						/* Release mute, 100mV input range */

	return 1;	/* Function succeeded */
}




void ns73_stop (void)
{
	NS73_PUT(0, 0x00); /* Power OFF */
}


