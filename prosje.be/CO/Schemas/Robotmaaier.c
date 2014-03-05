// Robotmaaier

// AANDACHT!
// Verwijder de "_NO" hieronder, als het programma voor de Arduino gecompileerd moet worden!
#define VoorArduino_NO

#ifndef P
#define P(s) ({static const char c[] __attribute__ ((progmem)) = s;c;})
#endif

#ifdef VoorArduino						// Broncode voor de Arduino-compiler
boolean                 debug = true;
#include <avr/io.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define delay_us(x) delayMicroseconds(x)
#else								// Gewone AVR-broncode
#define F_CPU 20000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#define delay(x) delay_ms(x)
#warning Not compiling for Arduino
#define delay_us(x) _delay_us(x)
void delay_ms(unsigned int ms)
{
    unsigned int            cnt;

    for (cnt = 0; cnt < ms; cnt++) {
	_delay_ms(0.99);
    }
}
#endif


// Uitgangen op PORTB
#define UIT_PORT	PORTB
#define UIT_DDR		DDRB
#define LEFT_DIR	_BV(0)					// Links vooruit of achteruit
#define LEFT_SPEED	_BV(1)					// OC1A - PWM links
#define RIGHT_SPEED	_BV(2)					// OC1B - PWM - rechts
#define RIGHT_DIR	_BV(3)					// Rechts vooruit of achteruit
#define MOWERBLADES	_BV(4)					// motor for the cutting blades
#define BEEPER		_BV(5)					// Beeper
#define UITPINNEN	LEFT_DIR | LEFT_SPEED | RIGHT_SPEED | RIGHT_DIR | MOWERBLADES | BEEPER

// Bedieningstoetsen op PORTC
// Enkel K_MANUEEL wordt nog als digitale ingangspin gebruikt - de overige define's gebruiken we als bitmask
#define KNOPPEN_PORT	PINC
#define K_VOORUIT	_BV(0)
#define K_ACHTERUIT	_BV(1)
#define K_LINKS		_BV(2)
#define K_RECHTS	_BV(3)
#define K_MESSEN	_BV(4)
#define K_MANUEEL	_BV(5)
#define K_AAN_UIT	_BV(6)
//#define PORTCPINNEN   K_VOORUIT | K_ACHTERUIT | K_LINKS | K_RECHTS | K_MESSEN | K_MANUEEL
#define PORTCPINNEN	K_MANUEEL

// Sensors op PORTD
#define SENSOR_PORT	PIND
#define FRONTBUMPER	_BV(0)					// Front bumper - aktief laag
#define REARBUMPER	_BV(1)					// Rear bumper - actief laag
#define P_RV		_BV(2)					// Perimeter RechtsVoor - aktief laag
#define P_LV		_BV(3)					// Perimeter LinksVoor - aktief laag
#define P_RA		_BV(4)					// Perimeter RechtAchter - aktief laag
#define P_LA		_BV(5)					// Perimeter LinksAchter - aktief laag
//#define AAN_UIT               _BV(6)                                  // AAN/UIT-knop
#define KANTELCONTACT	_BV(7)					// De kantel-schakelaars
#define PV_MASK		(P_RV | P_LV | P_RA | P_LA)
#define PORTDPINNEN	FRONTBUMPER | REARBUMPER | P_RV | P_LV | P_RA | P_LA | KANTELCONTACT

// Overige definities
#define START		1
#define VOORWAARTS	2
#define ACHTERWAARTS	3
#define	TURNLEFT	4
#define TURNRIGHT	5
#define ERROR		6
#define LINKS		7
#define RECHTS		8
#define OFF		9
#define ALL		10
#define ALLEBEI		11
#define TURN		12
#define UIT		13
#define AAN		14
#define NOP		15
#define OMKEER_DELAY	500
#define DEFAULT_SPEED	90
#define HALF_SPEED	60

#define MAAIER_ON()	PORTB |= MOWERBLADES
#define MAAIER_OFF()	PORTB &= ~(MOWERBLADES)

// Minimum en maximum turntime in (seconden * 10)
#define MAX_TURNTIME	654					// 65 seconden
#define MIN_TURNTIME	234					// 23 seconden
#define DEFAULT_TURNTIME 345
// De tijd die een draaiende motor nodig heeft om uit te bollen (in milliseconden),
// alvorens we de rij-richting omkeren
#define UITBOL_TIJD	3000					// 3 seconden

// Globale variabelen
//long                    turntime = 0;                         // Used for randomtime to turn mower from direction
unsigned int            turntime;
unsigned char           aan_uit = UIT, status = START;


// Subroutines


/*
  LCD-interface
  Software om de driedraads LCD-interface aan te sturen

  Pros  2007
*/

#define TXT 23
#define COM 45

// instruction register bit positions
#define LCD_CLR             0					// DB0: clear display
#define LCD_HOME            1					// DB1: return to home position
#define LCD_ENTRY_MODE      2					// DB2: set entry mode
#define LCD_ENTRY_INC       1					// DB1: 1=increment, 0=decrement
#define LCD_ENTRY_SHIFT     0					// DB2: 1=display shift on
#define LCD_ON              3					// DB3: turn lcd/cursor on
#define LCD_ON_DISPLAY      2					// DB2: turn display on
#define LCD_ON_CURSOR       1					// DB1: turn cursor on
#define LCD_ON_BLINK        0					// DB0: blinking cursor ?
#define LCD_MOVE            4					// DB4: move cursor/display
#define LCD_MOVE_DISP       3					// DB3: move display (0-> cursor) ?
#define LCD_MOVE_RIGHT      2					// DB2: move right (0-> left) ?
#define LCD_FUNCTION        5					// DB5: function set
#define LCD_FUNCTION_8BIT   4					// DB4: set 8BIT mode (0->4BIT mode)
#define LCD_FUNCTION_2LINES 3					// DB3: two lines (0->one line)
#define LCD_FUNCTION_10DOTS 2					// DB2: 5x10 font (0->5x7 font)
#define LCD_CGRAM           6					// DB6: set CG RAM address
#define LCD_DDRAM           7					// DB7: set DD RAM address
#define LCD_BUSY            7					// DB7: LCD is busy

// set entry mode: display shift on/off, dec/inc cursor move direction
#define LCD_ENTRY_DEC            0x04				// display shift off, dec cursor move dir
#define LCD_ENTRY_DEC_SHIFT      0x05				// display shift on,  dec cursor move dir
#define LCD_ENTRY_INC_           0x06				// display shift off, inc cursor move dir
#define LCD_ENTRY_INC_SHIFT      0x07				// display shift on,  inc cursor move dir

// display on/off, cursor on/off, blinking char at cursor position
#define LCD_DISP_OFF             0x08				// display off
#define LCD_DISP_ON              0x0C				// display on, cursor off
#define LCD_DISP_ON_BLINK        0x0D				// display on, cursor off, blink char
#define LCD_DISP_ON_CURSOR       0x0E				// display on, cursor on
#define LCD_DISP_ON_CURSOR_BLINK 0x0F				// display on, cursor on, blink char

// move cursor/shift display
#define LCD_MOVE_CURSOR_LEFT     0x10				// move cursor left  (decrement)
#define LCD_MOVE_CURSOR_RIGHT    0x14				// move cursor right (increment)
#define LCD_MOVE_DISP_LEFT       0x18				// shift display left
#define LCD_MOVE_DISP_RIGHT      0x1C				// shift display right

// function set: set interface data length and number of display lines
#define LCD_FUNCTION_4BIT_1LINE  0x20				// 4-bit interface, single line, 5x7 dots
#define LCD_FUNCTION_4BIT_2LINES 0x28				// 4-bit interface, dual line,   5x7 dots
#define LCD_FUNCTION_8BIT_1LINE  0x30				// 8-bit interface, single line, 5x7 dots
#define LCD_FUNCTION_8BIT_2LINES 0x38				// 8-bit interface, dual line,   5x7 dots

#define LCD_MODE_DEFAULT     (_BV(LCD_ENTRY_MODE) | _BV(LCD_ENTRY_INC))

// normally you do not change the following
#define LCD_LINES           2					// visible lines
#define LCD_LINE_LENGTH  0x40					// internal line length
#define LCD_START_LINE1  0x00					// DDRAM address of first char of line 1
#define LCD_START_LINE2  0x40					// DDRAM address of first char of line 2
#define LCD_START_LINE3  0x14					// DDRAM address of first char of line 3
#define LCD_START_LINE4  0x54					// DDRAM address of first char of line 4


// Stuur een karakter of een instructie naar de LCD-interface
#define LCD_PORT      	PORTC
#define LCD_DDR       	DDRC
#define LCD_DATA_PIN  	_BV(2)
#define LCD_CLK_PIN   	_BV(3)
#define LCD_LATCH_PIN  	_BV(0)
#define LCD_ENABLE	LCD_DDR |= LCD_DATA_PIN | LCD_CLK_PIN | LCD_LATCH_PIN
#define LCD_DISABLE	LCD_DDR &= ~(LCD_DATA_PIN | LCD_CLK_PIN)

// Dit is de bediening v/d 3-draads seriele interface.
// LCD_DATA_PIN wordt verbonden met de Data-ingang van een 74HC595 en met RS v/d LCD-module.
// LCD_CLK_PIN wordt verbonden met de schuifregisterklok v/d 74HC595
// LCD_LATCH_PIN wordt verbonden met de Latch-ingang v/d 74HC595 en met de E-pin v/d LCD-module.

// Werking:
// LCD_LATCH_PIN laag
// LCD_CLK_PIN laag
// Bit 7 klaarzetten
// LCD_CLK_PIN hoog en terug laag
// Bit 6 klaarzetten
// LCD_CLK_PIN hoog en terug laag
// ...
// Bit 0 klaarzetten
// LCD_CLK_PIN hoog en terug laag
// RS klaarzetten
// LCD_LATCH_PIN hoog en terug laag


#define char2LCD char2LCD3
#define lcd_puts_P(__s)    lcd_puts_p(P(__s))
#define lcd_delay(x)       _delay_loop_2(x)
#define lcd_command(x)     char2LCD3(x, COM)
#define lcd_char(x)        char2LCD3(x, TXT)

void char2LCD3(unsigned char karakter, unsigned char RS)
{
    volatile unsigned char  cnt;

    for (cnt = 0; cnt < 8; cnt++) {
	if ((karakter & 0x80) == 0x00) {			// een '0'
	    LCD_PORT &= ~(LCD_DATA_PIN);			// LCD_DATA_PIN laag
	} else {						// een '1'
	    LCD_PORT |= LCD_DATA_PIN;				// LCD_DATA_PIN hoog
	}
	lcd_delay(50);
	LCD_PORT |= LCD_CLK_PIN;				// Seriele klok hoog
	lcd_delay(50);
	LCD_PORT &= ~(LCD_CLK_PIN);				// Serieele klok terug laag
	karakter = karakter << 1;				// Bits een plaatsje naar rechts schuiven
    }
    if (RS == TXT) {
	LCD_PORT |= LCD_DATA_PIN;				// Tekst, RS moet hoog
    } else {
	LCD_PORT &= ~(LCD_DATA_PIN);				// Instructie, RS moet laag
    }
    lcd_delay(50);
    LCD_PORT |= LCD_LATCH_PIN;					// schuifregister -> par. uitgang
    lcd_delay(80);
    LCD_PORT &= ~(LCD_LATCH_PIN);				// par. uitgang -> LCD-module
    delay_us(42);
}


void lcd_init(void)
{
    LCD_ENABLE;							// Pinnen als uitgang schakelen
    LCD_PORT &= ~(LCD_CLK_PIN | LCD_DATA_PIN | LCD_LATCH_PIN);	// pinnen laag
    delay(20);							// 16 mSec wachten na power-on
    lcd_command(LCD_FUNCTION_8BIT_2LINES);			// RESET-procedure
    delay(10);							//   |
    lcd_command(LCD_FUNCTION_8BIT_2LINES);			//   |
    delay_us(150);						//   V
    lcd_command(LCD_FUNCTION_8BIT_2LINES);
    delay_us(50);
    lcd_command(LCD_FUNCTION_8BIT_2LINES);
    delay_us(50);
    lcd_command(LCD_DISP_OFF);
    delay_us(50);
    lcd_command(LCD_DISP_ON);
    delay_us(50);
    lcd_command(LCD_MODE_DEFAULT);
    delay_us(50);
    lcd_command(_BV(LCD_CLR));
    delay(2);
    lcd_command(LCD_DISP_ON);
    delay(2);
    LCD_DISABLE;						// PC2 en PC3 terug als INGANG schakelen!!!
}


// goto position (regel y, kolom x)
void lcd_gotoyx(unsigned char y, unsigned char x)
{
    unsigned char           linestart[5] = { 0, 0, 44, 20, 84 };

    LCD_ENABLE;
    if (x > 0) {
	x--;
    }
    x += linestart[y];
    lcd_command((1 << LCD_DDRAM) + x);				// goto position (x,y)
    LCD_DISABLE;
}


// clear lcd and set cursor to home position
void lcd_clrscr(void)
{
    LCD_ENABLE;
    LCD_PORT &= ~(LCD_CLK_PIN | LCD_DATA_PIN | LCD_LATCH_PIN);	// pinnen laag
    delay_us(2);
    lcd_command(1 << LCD_CLR);
    delay(2);
    LCD_DISABLE;
}


// Stuur string naar LCD
void lcd_puts(const char *s)
{
    LCD_ENABLE;
    LCD_PORT &= ~(LCD_CLK_PIN | LCD_DATA_PIN | LCD_LATCH_PIN);	// pinnen laag
    delay_us(2);
    while (*s) {
	lcd_char(*s);
	s++;
    }
    LCD_DISABLE;
}


// print string from program memory on lcd
void lcd_puts_p(const prog_char * progmem_s)
{
    register char           c;

    LCD_ENABLE;
    LCD_PORT &= ~(LCD_CLK_PIN | LCD_DATA_PIN | LCD_LATCH_PIN);	// pinnen laag
    delay_us(2);
    while ((c = pgm_read_byte(progmem_s++))) {
	lcd_char(c);
    }
    LCD_DISABLE;
}


// Een passieve (piezo) beeper en een LED op dezelfde pin.
// Piezo-beepers maken flink lawaai bij hoge frequenties.
// Bij lage frequenties (1Hz bijvoorbeeld) is het geluid miniem.
// Van die eigenschap kunnen we misbruik maken om een LED en zo'n beeper met dezelfde pin te sturen.
// De LED brand gedurende die tijd op halve kracht;
// Gebruik: beep(frequentie in Hz, beeptijd in mSec);
void beep(unsigned int frequentie, unsigned int beeptijd)
{
    unsigned int            wachttijd, times, count;

    // Voorbeeld: frequentie = 4000Hz, beeptijd = 300mSec
    // wachttijd in microseconden: ((1000000/2)/4000) = 125uS
    wachttijd = (unsigned int) ((1000000UL / 2) / (unsigned long) (frequentie));
    // times: (beeptijd * 500) / wachttijd = (300*500)/125 = 1200
    times = (unsigned int) (((unsigned long) (beeptijd) * 500UL) / (unsigned long) wachttijd);
    for (count = 0; count < times; count++) {
	PINB |= BEEPER;						// Toggle de beeper-uitgang
#ifdef VoorArduino
	delayMicroseconds(wachttijd);
#else
	_delay_us(wachttijd);
#endif
    }
}


// Een interrupt op PD7 (Arduino-pin 7, PCINT23)
ISR(PCINT2_vect)
{
    unsigned char           portb, tccr1a;

    portb = PORTB;						// Huidige toestand noteren
    tccr1a = TCCR1A;

    TCCR1A &= ~(_BV(COM1A1) | _BV(COM1B1));			// OC1A en OC1B afkoppelen
    PORTB = 0;							// Alle uitgangen laag
    delay(500);							// Contactdender vermijden
  REPEAT:
    while ((PIND & KANTELCONTACT) == 0) {			// Zolang er een kantelcontact gesloten is,
	delay(100);						// houden we ons stil
    }
    delay(1000);						// Nog even wachten - veiligheid eerst!
    if ((PIND & KANTELCONTACT) == 0) {				// Opnieuw controleren
	goto REPEAT;						// Contact nog gesloten? Terug naar de wachtlus!
    }

    TCCR1A = tccr1a;						// Toestand herstellen
    PORTB = portb;
}


unsigned int convertanalog(unsigned char channel)
{
    ADMUX = (channel & 0x07) | _BV(REFS0);			// Kanaal instellen; Ref = 5V
    delay(1);
    ADCSRA |= _BV(ADSC);					// start dummy conversion
    while ((ADCSRA & _BV(ADSC)) != 0) {
    }
    ADCSRA |= _BV(ADSC);					// start real conversion
    while ((ADCSRA & _BV(ADSC)) != 0) {				// Effe wachten tot ADSC laag is
    }
    return (ADC);
}


void adc_init(void)
{
    ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

    convertanalog(2);		// 1ste dummy-conversie
}



void InitTimer1(unsigned int frequentie)
{								// Init timer1 volgens mode 14 uit tabel 15.4, maar hou de PWM-uitgangen laag
    unsigned long           clk = F_CPU;

    if ((clk / (unsigned long) (frequentie)) > 0xFFFF) {	// Erg lage frequentie?
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);	// PWM-klok = F_CPU / 8
	ICR1 = ((clk / 8UL) / (unsigned long) frequentie) - 1;
    } else {							// PWM-klok = F_CPU
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	ICR1 = (clk / (unsigned long) frequentie) - 1;
    }
    TCCR1A = (1 << WGM11);
    TCNT1 = 0x00;
    OCR1A = 2;							// Minimum-waarde
    OCR1B = 2;
    PORTB &= ~(LEFT_SPEED | RIGHT_SPEED);			// Laag maken voor alle zekerheid
    DDRB |= (LEFT_SPEED | RIGHT_SPEED);				// Pas dan als uitgang schakelen
}


// Pas dit aan naar wens. Onder dit minimum wordt OC1A (OC1B) laag gemaakt.
#define minimum_PWM 25
void set_PWMA(unsigned char percent)
{								// Stel de puls/pause-verhouding van OC1A in
    if (percent < minimum_PWM) {				// Opgegeven percent lager dan minimum?
	TCCR1A &= ~(1 << COM1A1);				// OC1A afkoppelen
	PORTB &= ~(LEFT_SPEED);					// en laag maken
    } else if (percent > 100) {					// Meer dan 100% = maximum
	OCR1A = ICR1;
	TCCR1A |= (1 << COM1A1);
    } else {							// Percent is hoger dan minimum en lager dan 100
	OCR1A = ((unsigned long) (ICR1) * (unsigned long) percent) / 100UL;
	TCCR1A |= (1 << COM1A1);
    }
}

void set_PWMB(unsigned char percent)
{								// Stel de puls/pause-verhouding van OC1B in
    if (percent < minimum_PWM) {				// Opgegeven percent lager dan minimum?
	TCCR1A &= ~(1 << COM1B1);				// OC1B afkoppelen
	PORTB &= ~(RIGHT_SPEED);				// en laag maken
    } else if (percent > 100) {					// Meer dan 100% = maximum
	OCR1B = ICR1;
	TCCR1A |= (1 << COM1B1);
    } else {
	OCR1B = ((unsigned long) (ICR1) * (unsigned long) percent) / 100UL;
	TCCR1A |= (1 << COM1B1);
    }
}


// Stuur een motor: links, rechts, of allebei samen.
// Gebruik: motor(LINKS(RECHTS, ALLEBEI), VOORWAARTS(ACHTERWAARTS), snelheid(25% ... 100%));
// Een snelheid van minder dan minimum_PWM (zie hierboven) schakelt de motor uit.
void motor(unsigned char linksrechts, unsigned char richting, unsigned char snelheid)
{
    if (linksrechts == LINKS) {					// De linker motor
	if (richting == VOORWAARTS) {				// moet voorwaarts draaien
	    if ((UIT_PORT & LEFT_DIR) != 0) {			// Draaide hij achterwaarts?
		set_PWMA(5);					// Dan eerst de motor stoppen
		delay(UITBOL_TIJD);				// Laten uitbollen
		UIT_PORT &= ~(LEFT_DIR);			// DIR laag maken
		delay(500);
	    }
	} else {						// Motor moet achterwaarts
	    if ((UIT_PORT & LEFT_DIR) == 0) {			// Draaide hij voorwaarts?
		set_PWMA(5);					// Dan eerst de motor stoppen
		delay(UITBOL_TIJD);				// Laten uitbollen
		UIT_PORT |= LEFT_DIR;				// DIR hoog maken
		delay(500);
	    }
	}
	set_PWMA(snelheid);					// Snelheid aanpassen
    } else if (linksrechts == RECHTS) {				// De rechter motor
	if (richting == VOORWAARTS) {				// moet voorwaarts draaien
	    if ((UIT_PORT & RIGHT_DIR) != 0) {			// Draaide hij achterwaarts?
		set_PWMB(5);					// Dan eerst de motor stoppen
		delay(UITBOL_TIJD);				// Laten uitbollen
		UIT_PORT &= ~(RIGHT_DIR);			// DIR laag maken
		delay(500);
	    }
	} else {						// Motor moet achterwaarts
	    if ((UIT_PORT & RIGHT_DIR) == 0) {			// Draaide hij voorwaarts?
		set_PWMB(5);					// Dan eerst de motor stoppen
		delay(UITBOL_TIJD);				// Laten uitbollen
		UIT_PORT |= RIGHT_DIR;				// DIR hoog maken
		delay(500);
	    }
	}
	set_PWMB(snelheid);					// Snelheid aanpassen
    } else if (linksrechts == ALLEBEI) {			// Allebei vooruit of achteruit
	if (richting == VOORWAARTS) {				// Vooruit
	    if ((UIT_PORT & (LEFT_DIR | RIGHT_DIR)) != 0) {	// Er draait minstens 1 motor achterwaarts
		set_PWMA(5);					// Beide motoren stoppen
		set_PWMB(5);
		delay(UITBOL_TIJD);				// Laten uitbollen
		UIT_PORT &= ~(LEFT_DIR);			// DIR-pinnen laag maken
		UIT_PORT &= ~(RIGHT_DIR);
		delay(500);
	    }
	} else {						// Achteruit
	    if ((UIT_PORT & (LEFT_DIR | RIGHT_DIR)) != (LEFT_DIR | RIGHT_DIR)) {	// Er draait minstens 1 motor voorwaarts
		set_PWMA(5);					// Beide motoren stoppen
		set_PWMB(5);
		delay(UITBOL_TIJD);
		UIT_PORT |= LEFT_DIR;
		UIT_PORT |= RIGHT_DIR;
		delay(500);
	    }
	}
	set_PWMA(snelheid);					// Snelheid aanpassen
	set_PWMB(snelheid);
    }
}


// Omdat we vanuit verschillende routines moeten controleren of de Start/Stop knop is ingedrukt,
// plaatsen we dat controleren in een afzonderlijke routine.
unsigned char check_aan_uit(void)
{
    unsigned int            adc3 = convertanalog(3);		// Spanning op A3 meten

    if ((adc3 > 645) && (adc3 < 850)) {				// AAN/UIT-knop ingedrukt?
	if (aan_uit == AAN) {
	    aan_uit = UIT;
	    motor(ALLEBEI, VOORWAARTS, 5);			// Motors stoppen
	    MAAIER_OFF();					// Maaier stoppen
	    beep(2000, 200);					// Beepen en meteen contactdender onderdrukken
	    delay(200);
	    beep(1000, 200);
	    PORTB &= ~(BEEPER);					// Knipper-LED doven
	    return UIT;						// Terugkeren naar loop()
	} else {						// aan_uit was UIT
	    aan_uit = AAN;
	    beep(2000, 200);					// Beepen en meteen contactdender onderdrukken
	    delay(200);
	    beep(1000, 200);
	    PORTB |= BEEPER;					// Knipper-LED ontsteken
	    return AAN;
	}
    } else {
	return NOP;
    }
}


void setup(void)
{
    CLKPR = (1 << CLKPCE);
    CLKPR = 0;							// We willen op max. snelheid draaien
#ifdef VoorArduino
    randomSeed(analogRead(0));					//randomize
#endif
    UIT_DDR = UITPINNEN;
    lcd_init();
    PORTC = PORTCPINNEN;
    PORTD = PORTDPINNEN;					// Interne pull-up weerstanden activeren
    InitTimer1(18000);						// Stel de PWM-frequentie in op 18kHz
    adc_init();							// Initialiseer de ADC
    PCICR = _BV(PCIE2);						// Enable PinChange-Interrupt 2
    PCMSK2 = _BV(PCINT23);					// PD7 = interrupt-pin
    lcd_puts_P(" Motormaaier");
    lcd_gotoyx(2, 2);
    lcd_puts_P(" tot uw dienst!");
    sei();							// Enable global interrupts
}


#define aantal_samples 15
#define sample_delay	(500 / aantal_samples)
// Een afzonderlijke routine, om met de perimeter om te gaan
void perimeter(void)
{
    unsigned char           input[aantal_samples], cnt, hoek = 123;

    // We proberen eerst uit te zoeken, onder welke hoek we de draad naderen.
    // Daarvoor rijden we nog even door, en noteren ondertussen de perimeter-sensors
    // Op volle snelheid zou het ding 50cm/sec doen. We gaan dus nog 500mSec doorrijden.
    for (cnt = 0; cnt < aantal_samples; cnt++) {
	// TODO: bumpers controleren
	delay(sample_delay);					// Een stukje verder rijden
	input[cnt] = SENSOR_PORT & (PV_MASK);			// Huidige stand noteren
    }
    motor(ALLEBEI, VOORWAARTS, 5);				// Motors stoppen
    if (check_aan_uit() == UIT) {				// AAN/UIT-knop ingedrukt?
	return;							// Terugkeren naar automatisch()
    }
    // Laten we veronderstellen, dat we voorwaarts reden en dat de perimeter rechts-voor als eerste reageerde.
    // De lengte-as van de robotmaaier kan de perimeter-draad onder veschillende hoeken naderen:
    // Als links-voor kort na rechts-voor actief wordt, rijden we bijna haaks op de draad toe: 90 graden
    // Als rechts-achter kort na rechts-voor actief wordt, rijden we bijna parallel aan de perimeter: 0 graden
    if (status == VOORWAARTS) {					// We reden voorwaarts
	// Welke perimeter-sensor reageerde eerst?

	// Rechts-voor
	if ((input[0] & P_RV) == 0) {
	    for (cnt = 1; cnt < aantal_samples; cnt++) {
		if ((input[0] & P_LV) == 0) {			// Gevolgd door links-voor?
		    // We rijden min-of-meer haaks op de perimeter-draad toe
		    hoek = 90 - (cnt * 6);			// Hoe groter cnt, hoe meer de hoek afwijkt van 90 graden
		    break;					// We weten genoeg
		} else if ((input[0] & P_RA) == 0) {		// Gevolgd door rechts-achter?
		    // De hoek tov. de perimeter-draad is eerder klein.
		    hoek = cnt * 6;				// Hoe groter cnt, des te groter de hoek tov. de draad
		    break;
		}
	    }
	    if (hoek == 123) {					// Niets gevonden (123 was bij het ingaan van deze routine ingesteld)?
		goto WEETHETNIETMEER;				// Dan weg van hier!
	    } else {
		beep(3000, 300);
		delay(300);
		beep(3000, 300);
		// We gaan proberen, een eindje parallel aan de perimeter-draad te rijden...
		motor(ALLEBEI, ACHTERWAARTS, HALF_SPEED);	// Eerst een eindje achteruit
		delay(1000);
		// De draad zit min-of-meer rechts van ons, dus gaan we de linker-motor wat sneller laten draaien,
		// zodat we meer-en-meer parallel aan de draad komen te rijden
		motor(LINKS, ACHTERWAARTS, DEFAULT_SPEED);
		for (cnt = 0; cnt < hoek; cnt++) {		// Hoe groter de hoek,
		    delay(sample_delay);			// des te verder rijden we achteruit
		    if (check_aan_uit() == UIT) {		// AAN/UIT-knop ingedrukt?
			return;					// Terugkeren naar automatisch()
		    }
		}
		motor(ALLEBEI, VOORWAARTS, 5);			// Motors stoppen
		beep(3000, 300);
		delay(300);
		beep(3000, 300);
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);	// Volle kracht vooruit
		for (cnt = 0; cnt < 200; cnt++) {		// Gedurende 20 seconden
		    delay(100);
		    if ((SENSOR_PORT & (P_LV | P_LA)) != (P_LV | P_LA)) {	// P_LV of P_LA laag?
			goto END_RV;
		    } else if (check_aan_uit() == UIT) {	// AAN/UIT-knop ingedrukt?
			return;					// Terugkeren naar automatisch()
		    }
		}
	    }
	  END_RV:
	    motor(LINKS, VOORWAARTS, HALF_SPEED);		// Motor links vertragen = bocht naar links
	    delay(3000);					// Na drie seconden
	    motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);		// Volle kracht vooruit
	    return;
	}
	// Links-voor
	if ((input[0] & P_LV) == 0) {
	    for (cnt = 1; cnt < aantal_samples; cnt++) {
		if ((input[0] & P_RV) == 0) {			// Gevolgd door rechts-voor?
		    // We rijden min-of-meer haaks op de perimeter-draad toe
		    hoek = 90 - (cnt * 6);			// Hoe groter cnt, hoe meer de hoek afwijkt van 90 graden
		    break;					// We weten genoeg
		} else if ((input[0] & P_LA) == 0) {		// Gevolgd door links-achter?
		    // De hoek tov. de perimeter-draad is eerder klein.
		    hoek = cnt * 6;				// Hoe groter cnt, des te groter de hoek tov. de draad
		    break;
		}
	    }
	    if (hoek == 123) {					// Niets gevonden?
		goto WEETHETNIETMEER;				// Dan weg van hier!
	    } else {
		beep(3000, 300);
		delay(300);
		beep(3000, 300);
		// We gaan proberen, een eindje parallel aan de perimeter-draad te rijden...
		motor(ALLEBEI, ACHTERWAARTS, HALF_SPEED);	// Eerst een eindje achteruit
		delay(1000);
		// De draad zit min-of-meer links van ons, dus gaan we de linker-motor wat sneller laten draaien,
		// zodat we meer-en-meer parallel aan de draad komen te rijden
		motor(RECHTS, ACHTERWAARTS, DEFAULT_SPEED);
		for (cnt = 0; cnt < hoek; cnt++) {		// Hoe groter de hoek,
		    delay(sample_delay);			// des te verder rijden we achteruit
		    if (check_aan_uit() == UIT) {		// AAN/UIT-knop ingedrukt?
			return;					// Terugkeren naar automatisch()
		    }
		}
		motor(ALLEBEI, VOORWAARTS, 5);			// Motors stoppen
		beep(3000, 300);
		delay(300);
		beep(3000, 300);
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);	// Volle kracht vooruit
		for (cnt = 0; cnt < 200; cnt++) {		// Gedurende 20 seconden
		    delay(100);
		    if ((SENSOR_PORT & (P_RV | P_RA)) != (P_RV | P_RA)) {	// P_RV of P_RA laag?
			goto END_LV;				// Dan stoppen we ermee
		    } else if (check_aan_uit() == UIT) {	// AAN/UIT-knop ingedrukt?
			return;					// Terugkeren naar automatisch()
		    }
		}
	    }
	  END_LV:
	    motor(RECHTS, VOORWAARTS, HALF_SPEED);		// Motor rechts vertragen = bocht naar rechts
	    delay(3000);					// Na drie seconden
	    motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);		// Volle kracht vooruit
	    return;
	}

	goto WEETHETNIETMEER;

    }
    // Als we hierboven vastlopen, rest ons nog deze uitweg:
  WEETHETNIETMEER:
    if (status == VOORWAARTS) {					// Reden we voorwaarts?
	motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);		// Dan gaan we de andere kant op
	status = ACHTERWAARTS;
    } else {							// We reden achteruit
	motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);		// Dus gaan we nu vooruit
	status = VOORWAARTS;
    }
}


// Bereken een random-turntime
unsigned int get_turntime(void)
{								// Bereken een willekeurige turntime
    unsigned int            t;

    t = TCNT1;							// De tellerstand van Timer1
    if (t == 0) {						// Uitzonderlijk, maar niet uitgesloten
	t = DEFAULT_TURNTIME;					// Default
    } else if (t > MAX_TURNTIME) {				// Te groot?
	while (t > MAX_TURNTIME) {
	    t /= 2;						// Delen door 2, tot t klein genoeg is
	}
    } else if (t < MIN_TURNTIME) {				// Te klein?
	while (t < MIN_TURNTIME) {
	    t *= 2;						// Vermenigvuldigen met 2, tot t groot genoeg is
	}
    }
    return t;
}


void automatisch(void)
{
    unsigned int            cnt;
    unsigned char           afteller = 0;

    lcd_clrscr();						// LCD-scherm wissen
    lcd_puts_P(" MODE = automatisch");
    for (cnt = 1000; cnt > 200; cnt--) {
	PINB |= BEEPER;						// Een andere manier om te beeper
#ifdef VoorArduino
	delayMicroseconds(cnt);
#else
	_delay_us(cnt);
#endif
    }

    status = START;

    while ((KNOPPEN_PORT & K_MANUEEL) == 0) {			// Zolang de manueel-knop laag is
      BEGIN:
	if (check_aan_uit() == UIT) {				// AAN/UIT-knop ingedrukt?
	    return;						// Terugkeren naar loop()
	}

	afteller++;
	if (afteller > 10) {
	    PINB |= BEEPER;					// Toggle de beeper-uitgang
	    afteller = 0;					// zodat de LED knippert
	}

	if ((SENSOR_PORT & (PV_MASK)) != PV_MASK) {		// We naderen de perimeter-draad
	    perimeter();					// Dat handelen we niet zelf af...
	}

	switch (status) {					// Wat zijn we aan het doen?

	case START:						// We zijn pas opgestart
	    MAAIER_ON();					// Start de maaier
	    motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);		// Volle kracht vooruit
	    status = VOORWAARTS;				// Status aanpassen
	    turntime = get_turntime();				// Na x seconden gaan we andersom
	    break;

	case VOORWAARTS:					// We rijden voorwaarts 
	    if ((SENSOR_PORT & FRONTBUMPER) == 0) {		// De voorste bumper raakt iets!
		motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);	// Eerst terugtrekken
		for (cnt = 0; cnt < 50; cnt++) {		// 5 seconden achteruit rijden
		    if ((SENSOR_PORT & REARBUMPER) == 0) {	// En tussendoor de achterste bumper testen
			motor(ALLEBEI, VOORWAARTS, 5);		// Paniek! We zitten klem!
			MAAIER_OFF();				// Maaier stoppen
			status = ERROR;				// Status aanpassen
			goto BEGIN;				// Lus verlaten
		    }
		    delay(100);
		}
		// Na 5 seconden gaan we draaien
		motor(RECHTS, ACHTERWAARTS, HALF_SPEED);	// Rechtermotor vertragen
		for (cnt = 0; cnt < 30; cnt++) {		// 3 seconden draaien
		    if ((SENSOR_PORT & REARBUMPER) == 0) {	// Achterste bumper testen
			motor(ALLEBEI, VOORWAARTS, 5);		// Beide motors stoppen
			break;					// De for()-lus verlaten
		    }
		    delay(100);
		}
		// Nu kunnen we (hopelijk) terug vooruit rijden
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);
	    } else if (turntime == 0) {				// We gaan er een draai aan geven...
		motor(RECHTS, VOORWAARTS, HALF_SPEED);		// Rechtermotor stoppen
		beep(2000, 100);
		for (cnt = 0; cnt < 20; cnt++) {		// 2 seconden naar links draaien
		    // TODO: bumper controleren
		    delay(100);
		}
		motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);	// Achteruit rijden
		status = ACHTERWAARTS;				// Status aanpassen
		turntime = get_turntime();			// We gaan nu x seconden achteruit rijden
	    } else {
		turntime--;					// turntime met 1 verminderen
	    }
	    break;

	case ACHTERWAARTS:
	    if ((SENSOR_PORT & REARBUMPER) == 0) {		// De achterste bumper raakt iets
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);	// Dus rijden we even vooruit
		for (cnt = 0; cnt < 50; cnt++) {		// 5 seconden achteruit rijden
		    if ((SENSOR_PORT & FRONTBUMPER) == 0) {	// En tussendoor de voorste bumper testen
			motor(ALLEBEI, VOORWAARTS, 5);		// Paniek! We zitten klem!
			MAAIER_OFF();				// Maaier stoppen
			status = ERROR;				// Status aanpassen
			goto BEGIN;				// Lus verlaten
		    }
		    delay(100);
		}
		// Na 5 seconden gaan we draaien
		motor(RECHTS, VOORWAARTS, HALF_SPEED);		// Rechtermotor vertragen
		for (cnt = 0; cnt < 30; cnt++) {		// 3 seconden naar links draaien
		    if ((SENSOR_PORT & FRONTBUMPER) == 0) {	// Voorste bumper testen
			motor(LINKS, VOORWAARTS, 5);		// Linkermotor stoppen
			break;					// De for()-lus verlaten
		    }
		    delay(100);
		}
		// Nu kunnen we terug achteruit rijden
		motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);
	    } else if (turntime == 0) {				// We gaan er een draai aan geven...
		motor(RECHTS, ACHTERWAARTS, HALF_SPEED);	// Rechtermotor vertragen
		beep(2000, 100);
		for (cnt = 0; cnt < 20; cnt++) {		// 2 seconden draaien
		    // TODO: bumper controleren
		    delay(100);					// op de linker-motor
		}
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);	// Overschakelen op voorwaarts rijden
		status = VOORWAARTS;				// Status aanpassen
		turntime = get_turntime();			// We gaan nu x seconden voorruit rijden
	    } else {
		turntime--;					// turntime met 1 verminderen
	    }
	    break;

	case ERROR:						// Zaten we klem?
	    if ((SENSOR_PORT & FRONTBUMPER) != 0) {		// Voorste bumper vrij?
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);	// We proberen vooruit te rijden
		MAAIER_ON();					// En we starten de maaier terug
		status = VOORWAARTS;				// Status aanpassen
		turntime = get_turntime();
	    } else if ((SENSOR_PORT & REARBUMPER) != 0) {	// Achterste bumper vrij?
		motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);	// We proberen achteruit te rijden
		MAAIER_ON();					// En we starten de maaier terug
		status = ACHTERWAARTS;				// Status aanpassen
		turntime = get_turntime();
	    } else {
		beep(4000, 500);				// Maak wat kabaal
	    }
	    break;

	default:						// Hier horen we niet te belanden
	    status = START;
	    break;
	}
	delay(100);						// Zodat turntime = 200 ongeveer overeenkomt met 20 seconden
    }
    motor(ALLEBEI, VOORWAARTS, 5);				// Manueel-knop gesloten; motors stilleggen
    beep(2000, 200);
    delay(200);							// Even wachten om contactdender te onderdukken
    beep(1000, 200);
}


// Handbediening, zolang het "manueel"-contact gesloten is
// Theoretische uitlezingen van de toetsen:
// Open                 1023
// 12k ingedrukt         735
// 5k6 ingedrukt         556
// 2k2 ingedrukt         326
// 100R ingedrukt         21
void manueel(void)
{
    unsigned char           toetsen, vorige_toetsen = 0, richting = VOORWAARTS;
    unsigned int            adc2, adc3;

    lcd_clrscr();						// LCD-scherm wissen
    lcd_puts_P(" MODE = manueel");
    while ((KNOPPEN_PORT & K_MANUEEL) != 0) {			// Zolang K_MANUEEL hoog is...
	if (check_aan_uit() == UIT) {				// AAN/UIT-knop ingedrukt?
	    return;						// Terugkeren naar loop()
	}

	toetsen = 0;
	adc2 = convertanalog(2);				// Spanning op A2
	if (adc2 < 100) {					// 100R-toets ingedrukt
	    // Reserve
	} else if (adc2 < 400) {				// 2k2-toets ingedrukt
	    toetsen = K_ACHTERUIT;
	} else if (adc2 < 645) {				// 5k6-toets ingedrukt
	    toetsen = K_VOORUIT;
	} else if (adc2 < 850) {				// 12k-toets ingedrukt
	    toetsen = K_MESSEN;
	}

	adc3 = convertanalog(3);				// Spanning op A3
	if (adc3 < 100) {					// 100R-toets ingedrukt
	    // Reserve
	} else if (adc3 < 400) {
	    toetsen |= K_RECHTS;
	} else if (adc3 < 645) {
	    toetsen = K_LINKS;
	}

	if (toetsen != vorige_toetsen) {			// Gewijzigde toestand!
	    vorige_toetsen = toetsen;				// Noteren
	    delay(100);						// Even wachten ivm contactdender
	} else {						// Toestand = stabiel
	    switch (toetsen) {					// Welke is er ingedrukt?
	    case K_VOORUIT:					// Enkel de vooruit-knop
		motor(ALLEBEI, VOORWAARTS, DEFAULT_SPEED);
		richting = VOORWAARTS;
		break;
	    case K_ACHTERUIT:					// Enkel de achteruit-knop
		motor(ALLEBEI, ACHTERWAARTS, DEFAULT_SPEED);
		richting = ACHTERWAARTS;
		break;
	    case K_VOORUIT | K_LINKS:				// Vooruit en links samen
	    case K_LINKS:					// Enkel de links-knop
		motor(LINKS, VOORWAARTS, 5);
		motor(RECHTS, VOORWAARTS, DEFAULT_SPEED);
		richting = VOORWAARTS;
		break;
	    case K_VOORUIT | K_RECHTS:				// Vooruit en rechts samen
	    case K_RECHTS:					// Enkel de rechts-knop
		motor(RECHTS, VOORWAARTS, 5);
		motor(LINKS, VOORWAARTS, DEFAULT_SPEED);
		richting = VOORWAARTS;
		break;
	    case K_ACHTERUIT | K_LINKS:			// Achteruit en links samen
		motor(LINKS, VOORWAARTS, 5);
		motor(RECHTS, ACHTERWAARTS, DEFAULT_SPEED);
		richting = ACHTERWAARTS;
		break;
	    case K_ACHTERUIT | K_RECHTS:			// Achteruit en rechts samen
		motor(RECHTS, VOORWAARTS, 5);
		motor(LINKS, ACHTERWAARTS, DEFAULT_SPEED);
		richting = ACHTERWAARTS;
		break;
		// Voor de messen maken we een uitzondering.
		// Even de toest indrukken start of stopt de messen. Daarvoor schrijven we naar PINB, niet naar PORTB
		// Die toestand blijft behouden tot er weer op de knop gedrukt wordt.
	    case K_MESSEN:
		PINB |= MOWERBLADES;
		delay(400);					// Contactdender vermijden
		break;
	    default:						// Niets ingedrukt, of een verboden combinatie ingedrukt
		// We onthouden in welke richting de motor laatst draaide, zodat de motor()-routine weet
		// dat ze geen delay moet inlassen als we na het stoppen in dezelfde richting verder rijden
		if (richting == VOORWAARTS) {
		    motor(ALLEBEI, VOORWAARTS, 5);
		} else {
		    motor(ALLEBEI, ACHTERWAARTS, 5);
		}
		break;
	    }
	}
    }
    motor(ALLEBEI, VOORWAARTS, 5);				// Manueel = laag; we stoppen er mee
}



#ifdef VoorArduino
void loop()
{
#else
int main(void)
{
    setup();
#endif
    while (1) {
	check_aan_uit();
	if (aan_uit == AAN) {					// AAN/UIT-knop ingedrukt?
	    if ((KNOPPEN_PORT & K_MANUEEL) == 0) {		// Manueel-knop gesloten
		automatisch();
	    } else {
		manueel();
	    }
	}
    }
}
