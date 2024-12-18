// disable watchdog timer
#pragma config PERIOD = PERIOD_OFF, WINDOW = WINDOW_OFF

// set brown-out detect for 1.8V
#pragma config SLEEP = SLEEP_ENABLED, ACTIVE = ACTIVE_ENABLED
#pragma config SAMPFREQ = SAMPFREQ_1KHZ, LVL = LVL_BODLEVEL0

// set internal oscillator to 20 MHz 
#pragma config FREQSEL = FREQSEL_20MHZ, OSCLOCK = CLEAR

// set SYSCFG0 
#pragma config EESAVE = CLEAR, RSTPINCFG = RSTPINCFG_GPIO
#pragma config CRCSRC = CRCSRC_NOCRC

// set SYSCFG1
#pragma config SUT = SUT_64MS

// unlock memory
#pragma config LB = LB_NOLOCK



// defines for discrete transistors
#define USBfet	0b00100000
#define dig1en	0b00010000
#define dig2en	0b00001000
#define dig3en	0b00000100
#define dig4en	0b00000010
#define dig5en	0b00000001

// defines for buttons
#define b_mask  0b00000111
#define down    0b00000100
#define ok      0b00000010
#define up      0b00000001
#define none    0b00000000

// define timing diagnostic pins
#define rtc_pulse 0b10000000    // PD7
#define pit_pulse 0b01000000    // PD6
#define BOD_pulse 0b00100000    // PD5

// defines for LEDs
#define off     0b00000000
#define dpt     get_segs('.')
#define alrLED  0b00001000

// define for buzzer
#define buzz 0b01000000

// duration of current learning in seconds
#define learn_loops 120

// number of consecutive tries before terminating charge
#define term_loops 150

// multiplier to find end current from average current
#define curr_thresh 0.65 

// circuit dependent constants        
#define adc_conversion 4.3/(1023.0*64.0) 
#define AVDD_volt_div (11800.0+4750.0)/11800.0
#define transconductance (4750.0/(150000.0+4750.0))/0.07

// general
#define tick 125



// includes
#include <avr/cpufunc.h>    // eeprom constants and change protection
#include <avr/eeprom.h>     // eeprom 
#include <avr/io.h>         // declares variables for the ATmega4809 processor
#include <avr/sleep.h>      // sleep functions
#include <limits.h>         // max int
#include <math.h>           // for the log function
#include <stdlib.h>         // assorted number functions
#include <string.h>         // strlen
#include <xc.h>             // interrupt macros



// global variables
enum adc_type {init, volt, cur, vccn};
volatile enum adc_type adc_pending = init; 
volatile unsigned char digit_cnt = 1;
volatile unsigned char pwr_ok = 1; 
volatile signed char sec = 0;
volatile signed char min = 0;
volatile signed char hr = 0;
volatile unsigned int day = 0;
volatile unsigned int avdd_cnt = 0;
volatile unsigned int curr_cnt = 0; 
volatile unsigned int loop_count = 0; 
volatile unsigned long int last_set = 0;  
volatile float total_energy = 0; 
volatile float avdd = 0;
volatile float avdd_acc = 0; 
volatile float current = 0;
volatile float curr_acc = 0;
volatile float qavdd = 0;
volatile float vcc1 = 0;
volatile float vcc2 = 0;
float adc2AVDD = 0;
float adc2curr = 0; 
char dig1 = off; // all off
char dig2 = off; // all off
char dig3 = off; // all off
char dig4 = off; // all off
char dig5 = off; // all off



// setup sets up the microcontroller 
void setup (void)
{
    // setup clocks      
    // enable 32768 Hz external oscillator
    ccp_write_io((void *) &(CLKCTRL.XOSC32KCTRLA), 0b00000011);
    
    // 20MHz clock
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLA), 0b00000000);
   
    // set prescaler to divide by 4 (5 MHz clock)
    ccp_write_io((void *) &(CLKCTRL.MCLKCTRLB), 0b00000011);

	// porta is used for segment drivers
    PORTA_OUT = 0b11111111; // all LEDs off
	PORTA_DIR = 0b11111111;
    PORTA_PIN0CTRL = 0b10000000; // invert output pin
    PORTA_PIN1CTRL = 0b10000000; // invert output pin
    PORTA_PIN2CTRL = 0b10000000; // invert output pin
    PORTA_PIN3CTRL = 0b10000000; // invert output pin   
    PORTA_PIN4CTRL = 0b10000000; // invert output pin
    PORTA_PIN5CTRL = 0b10000000; // invert output pin
    PORTA_PIN6CTRL = 0b10000000; // invert output pin
    PORTA_PIN7CTRL = 0b10000000; // invert output pin  
    
    // portb is unavailable in the 40 pin dip
    PORTB_PIN0CTRL = 0b00000100; // disable input
    PORTB_PIN1CTRL = 0b00000100; // disable input
    PORTB_PIN2CTRL = 0b00000100; // disable input
    PORTB_PIN3CTRL = 0b00000100; // disable input
    PORTB_PIN4CTRL = 0b00000100; // disable input
    PORTB_PIN5CTRL = 0b00000100; // disable input
       
    // portc is used to drive the digit select transistors and the USB switch
    // portc bits 6 and 7 are not connected in the 40 pin dip and are disabled
    PORTC_PIN6CTRL = 0b00000100;    // disable input
    PORTC_PIN7CTRL = 0b00000100;    // disable input
    PORTC_OUT = 0b00111111;         // all off
    PORTC_DIR = 0b11111111;         // unavailable pins output
    
    // portd is for expansion, the diagnostic outputs can be eliminated if desired
    PORTD_DIR = rtc_pulse | pit_pulse | BOD_pulse; // configure diagnostic outputs
    PORTD_OUTCLR = rtc_pulse;       // initialize low
    PORTD_OUTCLR = pit_pulse;       // initialize low
    PORTD_OUTCLR = BOD_pulse;       // initialize low
    
    // porte is used for switch inputs and alarm LED
    PORTE_PIN0CTRL = 0b10001000;	// invert, enable pullup
	PORTE_PIN1CTRL = 0b10001000;	// invert, enable pullup
    PORTE_PIN2CTRL = 0b10001000;	// invert, enable pullup
    PORTE_OUT = 0b00000000;         // all off
    PORTE_DIR = alrLED;             // alarm LED is output
	
    // portf is used for the crystal oscillator, analog inputs, and the buzzer
    PORTF_OUT = 0b00000000;         // all off
    PORTF_DIR = buzz;               // enable buzzer output
	PORTF_PIN2CTRL = 0b00000100;    // disable digital input for analog inputs
    PORTF_PIN3CTRL = 0b00000100;    // disable digital input for analog inputs
    PORTF_PIN4CTRL = 0b00000100;    // disable digital input for analog inputs
    PORTF_PIN5CTRL = 0b00000100;    // disable digital input for analog inputs 

	// voltage reference setup
	VREF_CTRLA = 0b00110011;		// 4.3 V ADC reference, 4.3V AC reference
	VREF_CTRLB = 0b00000011;		// keep references enabled
	
	// analog to digital converter setup
	ADC0_CTRLA = 0b10000000;		// run standby enabled, ADC disabled
	ADC0_CTRLB = 0b00000110;		// 64 results accumulated
	ADC0_CTRLC = 0b01000001;		// small capacitor, internal reference, prescale 4
	ADC0_CTRLA = 0b10000001;		// ADC enabled
    
    // setup analog to digital constants
    adc2AVDD = adc_conversion * AVDD_volt_div * AVDD_cal; 
    adc2curr = adc_conversion * transconductance * curr_cal;
    
    // real time counter setup
    while (RTC_STATUS) {}           // wait until not busy
    RTC_CLKSEL = 0b00000010;        // crystal clock source
    RTC_CTRLA = 0b10001101;         // run standby, prescale by 2, correct, enable
    RTC_PER = (32768 / 2) - 1;      // lowest prescale that allows correction
    
    // read RTC calibration factor from EEPROM
    unsigned int rtc_word;
    rtc_word = eeprom_read_word((unsigned int *)&USERROW_USERROW0);
    // verify RTC calibration factor. Low byte is calibration factor,
    // high byte should be inverted copy of low byte
    if ((~(rtc_word & 0xFF00))>>8 == (rtc_word & 0x00FF))
        RTC_CALIB = rtc_word & 0xFF;   // read successful, use eeprom constant
    else
        RTC_CALIB = 0b00000000;     // not programmed or read failed, use zero
    
    // setup RTC interrupts
    RTC_INTCTRL = 0b00000001;       // enable overflow interrupt
    RTC_PITCTRLA = 0b00101001;      // period 32 (64 before prescale), enable
    RTC_PITINTCTRL = 0b00000001;    // interrupt enabled
    
    // setup brownout interrupts 
    BOD_VLMCTRLA = 0b00000000;      // threshold BOD + 5%    
    BOD_INTCTRL = 0b00000000;       // no interrupts
}



// this function delays for approximately d microseconds. The argument
// must be in the range 50<d<65420. It uses a _NOP n operation
// to avoid complier optimization
void delay_us (unsigned int d)
{
		d = (d-50) * 1.0025; // constant experimentally determined for 5MHz clock
	while (d > 0)
	{
        _NOP ();  // no operation, just wastes time
        _NOP ();  // no operation, just wastes time
        _NOP ();  // no operation, just wastes time
        _NOP ();  // no operation, just wastes time
		d--;
	}
}



// this function delays for t milliseconds. It uses delay_us.
// The duration is limited by the integer size to 65.535 seconds.
void delay_ms (unsigned int t)
{
	for (; t!=0; t--)
	{
		delay_us (1000); 
	}
}



// debounce by waiting for all buttons to be released 
void wait_up (void)
{
    do 
    {
        
    } while ((PORTE_IN & b_mask) != none);
    delay_ms (50); // brief debounce
}



// get segs returns the segments to display numchar
unsigned char get_segs (unsigned char numchar)
{
    unsigned char segs;
	// find the appropriate segments
	switch (numchar)
	{
		case 0:{segs = 0b11111100; break;}
		case 1:{segs = 0b00101000; break;}
		case 2:{segs = 0b10011110; break;}
		case 3:{segs = 0b00111110; break;}
		case 4:{segs = 0b01101010; break;}
		case 5:{segs = 0b01110110; break;}
		case 6:{segs = 0b11110110; break;}
		case 7:{segs = 0b00101100; break;}
		case 8:{segs = 0b11111110; break;}
		case 9:{segs = 0b01101110; break;}
		case 10:{segs = 0b11101110; break;} // for hex output A
		case 11:{segs = 0b11110010; break;} // for hex output b
		case 12:{segs = 0b11010100; break;} // for hex output C
		case 13:{segs = 0b10111010; break;} // for hex output d
		case 14:{segs = 0b11010110; break;} // for hex output E
		case 15:{segs = 0b11000110; break;} // for hex output F
        case ' ':{segs = 0b00000000; break;}
        case '"':{segs = 0b01001000; break;}
        case '\'':{segs = 0b00001000; break;}
		case '-':{segs = 0b00000010; break;}
        case '.':{segs = 0b00000001; break;}
        case '0':{segs = 0b11111100; break;}
		case '1':{segs = 0b00101000; break;}
		case '2':{segs = 0b10011110; break;}
		case '3':{segs = 0b00111110; break;}
		case '4':{segs = 0b01101010; break;}
		case '5':{segs = 0b01110110; break;}
		case '6':{segs = 0b11110110; break;}
		case '7':{segs = 0b00101100; break;}
		case '8':{segs = 0b11111110; break;}
		case '9':{segs = 0b01101110; break;}
        case '=':{segs = 0b00010010; break;}
		case 'A':{segs = 0b11101110; break;}
        case 'C':{segs = 0b11010100; break;}
		case 'E':{segs = 0b11010110; break;}
		case 'F':{segs = 0b11000110; break;}
        case 'G':{segs = 0b11110110; break;}
        case 'H':{segs = 0b11101010; break;}
		case 'I':{segs = 0b11000000; break;}
        case 'J':{segs = 0b00111000; break;}
		case 'L':{segs = 0b11010000; break;}
        case 'N':{segs = 0b11101100; break;}
        case 'O':{segs = 0b11111100; break;}
		case 'P':{segs = 0b11001110; break;}
		case 'S':{segs = 0b01110110; break;}
        case 'U':{segs = 0b11111000; break;}
        case 'Y':{segs = 0b01111010; break;}
        case 'Z':{segs = 0b10011110; break;}
        case '[':{segs = 0b11010100; break;}
        case ']':{segs = 0b00111100; break;}
        case '_':{segs = 0b00010000; break;}
		case 'b':{segs = 0b11110010; break;}
        case 'c':{segs = 0b10010010; break;}
        case 'd':{segs = 0b10111010; break;}
        case 'g':{segs = 0b01111110; break;}
        case 'h':{segs = 0b11100010; break;}
        case 'i':{segs = 0b00100000; break;}
        case 'l':{segs = 0b00101000; break;}
        case 'n':{segs = 0b10100010; break;}
        case 'o':{segs = 0b10110010; break;}
		case 'r':{segs = 0b10000010; break;}
        case 't':{segs = 0b11010010; break;}
        case 'u':{segs = 0b10110000; break;}
		default:segs = 0b00010000;  // underscore for undefined digits
	}
    return (segs);
}



// display_hex displays numbers from 0 to FFFF inclusive on the 7 segment display
void disp_hex (unsigned int number)
{
	char dig;
	dig = number / 0x1000;				// integer division: get first digit
	dig1 = get_segs (dig);              // display 16^4 digit
	number %= 0x1000;					// compute remainder
	dig = number / 0x100;				// get second digit
	dig2 = get_segs (dig);              // display 16^3 digit
	number %= 0x100;					// compute remainder
	dig = number / 0x10;				// gets third digit
	dig3 = get_segs (dig);              // display 16 digit
	number %= 0x10;						// compute remainder
	dig4 = get_segs (number);           // display ones digit
}



// disp_4char displays a four character string
void disp_4char (char str[])
{
    dig1 = get_segs (str[0]);
    dig2 = get_segs (str[1]);
    dig3 = get_segs (str[2]);
    dig4 = get_segs (str[3]);
    dig5 = off; // off
}



// disp_string displays a string of any length
void disp_string (char string[])
{
    int i; 
    
    // check to see if string is less than 4 char
    if (strlen (string) <= 4)
    {
        while (strlen (string) < 4)
            strcat (string, " ");       // pad it with spaces if required
        disp_4char (string);            // display first four characters
    }
    else
    {
        // string is longer that 4 characters
        // walk it on leading with blanks
        dig1 = get_segs (' '); 
        dig2 = get_segs (' '); 
        dig3 = get_segs (' ');
        dig4 = get_segs (string[0]); 
        delay_ms (tick);
        
        dig1 = get_segs (' '); 
        dig2 = get_segs (' '); 
        dig3 = get_segs (string[0]); 
        dig4 = get_segs (string[1]); 
        delay_ms (tick);
        
        dig1 = get_segs (' '); 
        dig2 = get_segs (string[0]); 
        dig3 = get_segs (string[1]); 
        dig4 = get_segs (string[2]); 
        delay_ms (tick);
        
        // display the rest of the string
        for (i = 0; string[i+3] != '\0'; i++)
        {
            disp_4char (string + i);
            delay_ms (tick);
        }
        
        // walk it off with trailing blanks
        dig1 = get_segs (string [i]); 
        dig2 = get_segs (string [i+1]); 
        dig3 = get_segs (string [i+2]);
        dig4 = get_segs (' '); 
        delay_ms (tick);
        
        dig1 = get_segs (string [i+1]); 
        dig2 = get_segs (string [i+2]);
        dig3 = get_segs (' '); 
        dig4 = get_segs (' '); 
        delay_ms (tick);
        
        dig1 = get_segs (string [i+2]);
        dig2 = get_segs (' '); 
        dig3 = get_segs (' '); 
        dig4 = get_segs (' ');
        delay_ms (tick);
        
        disp_4char ("    ");        // blank
        delay_ms (tick);
    }
}



// display_int displays numbers from 0 to 9999 inclusive on the 7 segment display.
// Leading zeros are suppressed if suppress is true.
void disp_int (int number, char suppress)
{
    char dig; 
    dig5 = off; 
    if (number > 9999) // out of range 
    {
        dig1 = get_segs ('-') | dpt;
        dig2 = get_segs ('-') | dpt;
        dig3 = get_segs ('-') | dpt;
        dig4 = get_segs ('-') | dpt;
    }
    else
    {
        dig = number / 1000;				// integer division: get thousands digit
        if (dig != 0)
            suppress = 0;					// cancel suppression
        if (suppress)
            dig1 = get_segs (' ');
        else
            dig1 = get_segs (dig);          // display thousands digit
        number %= 1000;						// compute remainder
        dig = number / 100;					// get hundreds digit
        if (dig != 0)
            suppress = 0;					// cancel suppression
        if (suppress)
            dig2 = get_segs (' ');
        else
            dig2 = get_segs (dig); 
        number %= 100;						// compute remainder
        dig = number / 10;					// gets tens digit
        if (dig != 0)
            suppress = 0;					// cancel suppression
        if (suppress)
            dig3 = get_segs (' ');
        else
            dig3 = get_segs (dig); 
        dig = number % 10;                  // compute remainder
        dig4 = get_segs (dig); 
    }
}



// disp_signed_int displays numbers from -999 to 9999 inclusive on the 7 segment display.
// Leading zeros are suppressed if suppress is true.
void disp_signed_int (int number, char suppress)
{
    if (number >= 0)
        disp_int (number, suppress);            // use disp_int to display positive numbers
    else
    {
        number = - number;                      // convert to positive
        char dig; 
        dig5 = off;                             // turn off seconds
        if (number > 999) // out of range 
        {
            dig1 = get_segs ('-') | dpt;
            dig2 = get_segs ('-') | dpt;
            dig3 = get_segs ('-') | dpt;
            dig4 = get_segs ('-') | dpt;
        }
        else
        {
            dig1 = get_segs ('-');
            dig = number / 100;					// get hundreds digit
            if (dig != 0)
                suppress = 0;					// cancel suppression
            if (suppress)
                dig2 = get_segs (' ');
            else
                dig2 = get_segs (dig); 
            number %= 100;						// compute remainder
            dig = number / 10;					// gets tens digit
            if (dig != 0)
                suppress = 0;					// cancel suppression
            if (suppress)
                dig3 = get_segs (' ');
            else
                dig3 = get_segs (dig); 
            dig = number % 10;                  // compute remainder
            dig4 = get_segs (dig); 
        }
    }
}



void disp_signed_char (signed char x)
{
    dig5 = off;                 // turn off the individual LEDs
    
    if (x < 0)                  // trap negative values
    {
        x = -x;                 // make it positive
        dig1 = get_segs ('-');  // display negative sign
    }
    else                        // positive 
        dig1 = get_segs (' ');  // blank first digit
    dig2 = get_segs (x/100);    // hundreds
    x = x % 100;                // remainder
    dig3 = get_segs (x/10);     // tens
    x = x % 10;                 // remainder
    dig4 = get_segs (x);        // ones
}



void get_secs (char colon)
{
    char sec_LEDs = 0; 
#ifdef binary       // binary seconds
    sec_LEDs = sec; // colon on, seconds displayed on individual LEDs
#endif
#ifdef bar_graph    // sequential seconds
    switch (sec*7/60)
    {
        case 0:     // 0 to 8
            sec_LEDs = 0b00000000; 
            break;
        case 1:     // 9 to 17
            sec_LEDs = 0b00100000; 
            break; 
        case 2:     // 18 to 25
            sec_LEDs = 0b00110000; 
            break;
        case 3:     // 26 to 34
            sec_LEDs = 0b00111000; 
            break;
        case 4:     // 35 to 42
            sec_LEDs = 0b00111100; 
            break; 
        case 5:     // 43 to 51
            sec_LEDs = 0b00111110; 
            break; 
        case 6:     // 52 to 59
            sec_LEDs = 0b00111111; 
            break;
    }  
#endif
#ifdef all              // all LEDs on 
    sec_LEDs = 0b00111111;  // all on
#endif  
    if (colon)
        sec_LEDs |= 0b11000000; 
    dig5 = sec_LEDs;
}



// This is a function to display floating point
// numbers in the range of 0.001 to 9999
void disp_float (float number, char sec_ok)
{
    int temp; 
    di ();                      // disable interrupts
    if (number > 9999.5)        // out of range 
    {
        dig1 = get_segs ('-') | dpt;
        dig2 = get_segs ('-') | dpt;
        dig3 = get_segs ('-') | dpt;
        dig4 = get_segs ('-') | dpt;
    }
    else if (number >= 1000)
    {
        temp = number;          // coerce to integer
        disp_int (temp, 1);     // display as integer
    }
    else if (number >= 100) 
    {
        temp = 10 * number;     // multiply by 10 and coerce
        disp_int (temp, 1);     // display as integer
        dig3 = dig3 | dpt;      // turn on decimal point
    }
    else if (number >= 10)
    {
        temp = 100 * number;    // multiply by 100 and coerce
        disp_int (temp, 1);     // display as integer
        dig2 = dig2 | dpt;      // turn on decimal point
    }
    else 
    {
        temp = 1000 * number;   // multiply by 1000 and coerce
        disp_int (temp, 0);     // display as integer
        dig1 = dig1 | dpt;      // turn on decimal point
    }
    if (sec_ok)
        get_secs (0);           // display seconds without colon
    ei ();
}



// This is a stripped down function to display floating point
// numbers in the range of 0.000 to 9.999 
void disp_fast_float (float number)
{
    int temp; 
    char dig; 
    {
        temp = 1000 * number;           // multiply by 1000 and coerce
        dig = temp / 1000;				// integer division: get thousands digit
        dig1 = get_segs (dig) | dpt;    // display thousands digit with decimal point
        temp %= 1000;                   // compute remainder
        dig = temp / 100;				// get hundreds digit
        dig2 = get_segs (dig); 
        temp %= 100;					// compute remainder
        dig = temp / 10;				// gets tens digit
        dig3 = get_segs (dig); 
        dig = temp % 10;                // compute remainder
        dig4 = get_segs (dig);          // get ones digit
    }
    get_secs(0);                        // update seconds     
}



// disp_time computes the digits to display the time
void disp_time (char time_mode)
{ 
    unsigned char hr_disp = hr; 
    
    // check for 12 hour format
    if (time_mode == 12)
    {
        // compute 12 hour time
        if (hr == 0)
            hr_disp = 12;
        else if (hr > 12)
            hr_disp = hr - 12;
    }
    
    // suppress leading digit in 12 hour mode
    if (time_mode == 12 && hr_disp < 10)
        dig1 = off; // all segments off
    else
        dig1 = get_segs (hr_disp / 10);
    dig2 = get_segs (hr_disp % 10); 
    dig3 = get_segs (min / 10);     // tens of minutes
    dig4 = get_segs (min % 10);     // ones of minutes
    get_secs (1);                   // set dig5 with seconds and colon
}



// init_AVDD starts an ADC conversion of AVDD
void init_avdd (void)
{
    // set ADC
    ADC0_CTRLB = 0b00000110;        // 64 results 
    ADC0_CTRLC = 0b01000011;		// small cap, Vref reference, prescale 16
    ADC0_MUXPOS = 0b00001101;		// select AIN13 to measure
    ADC0_COMMAND = 0b00000001;		// start conversion
}



// calc_AVDD converts the raw ADC result to the AVDD voltage
float calc_AVDD (void)
{ 
    float AVDD; 
    
    AVDD = ADC0_RES * adc2AVDD; 
    return (AVDD);
}



// measure AVDD voltage 
float measure_AVDD (void)
{  
    // set ADC
    init_avdd ();
    
    // wait for completion
    while (ADC0_COMMAND)       
        {}
    
    // calculate result
    return (calc_AVDD());
}



// init_current starts an ADC conversion of the USB current
void init_curr (void)
{
    // set ADC
    ADC0_CTRLB = 0b00000110;        // 64 results 
    ADC0_CTRLC = 0b01000011;		// small cap, Vref reference, prescale 16
    ADC0_MUXPOS = 0b00001100;		// select AIN12 to measure
    ADC0_COMMAND = 0b00000001;		// start conversion
}



// calc_curr converts the raw ADC result to the USB current
float calc_curr (void)
{
    float iUSB; 

    iUSB = ADC0_RES * adc2curr;   // calculate current
    return (iUSB);
}



// measure USB current 
float measure_current (void)
{  
    // set ADC
    init_curr ();
    
    // wait for completion 
    while (ADC0_COMMAND)            
        {}
    
    // calculate result 
    return (calc_curr ());
}



// measure CC1 voltage 
float measure_VCC1 (void)
{
    float VCC1;
    // set ADC
    ADC0_CTRLB = 0b00000000;        // 1 result
    ADC0_CTRLC = 0b01010011;		// small cap, Vref reference, prescale 16
    ADC0_MUXPOS = 0b00001110;		// select AIN14 to measure
    ADC0_COMMAND = 0b00000001;		// start conversion
    while (ADC0_COMMAND)            // wait for completion 
        {}
    VCC1 = (4.3 * ADC0_RES)/1023.0; // calculate voltage
    return (VCC1);
}



// measure CC2 voltage 
float measure_VCC2 (void)
{
    float VCC2;
    // set ADC
    ADC0_CTRLB = 0b00000000;            // 1 result
    ADC0_CTRLC = 0b01010011;            // small cap, Vref reference, prescale 16
    ADC0_MUXPOS = 0b00001111;           // select AIN15 to measure
    ADC0_COMMAND = 0b00000001;          // start conversion
    while (ADC0_COMMAND)                // wait for completion 
        {}
    VCC2 = (4.3 * ADC0_RES)/1023.0;     // calculate voltage
    return (VCC2);
}



// check connect checks to see if a USB_C load is connected
char USB_connected (void)
{
    if ((vcc1 < 2.3) || (vcc2 < 2.3))
        return (1);         // connected if either voltage is low
    else
        return (0);         // not connected
}



// this function turns the USB port ON with a soft start
// the soft start uses the time constant formed by the gate resistor
// and the gate capacitance. The gate charge is increased in steps with 
// a delay between steps to allow the power supply to catch up. 
void USB_on (void)
{
    PORTC_DIRCLR = USBfet;      // set direction as input
    PORTC_OUTCLR = USBfet;      // set output low (but direction is input)
    for (char i=0; i<50; i++)
    {
        PORTC_DIRSET = USBfet;  // charge the gate a little bit...
        PORTC_DIRCLR = USBfet;  // ... and then stop. 
        delay_us (100); 
    }
    // turn USB port solidly on
    PORTC_DIRSET = USBfet;
}



// this function turns the USB port OFF
void USB_off (void)
{
    // turn USB port on
    PORTC_OUTSET = USBfet; 
}



// self_test
void self_test (void)
{
    // declare variables
    unsigned int unsigned_eeprom;
    
    // test alarm LED
    for (int i = 0; i<4; i++)
    {
        PORTE_OUTSET = alrLED;      // turn alarm LED on 
        delay_ms (tick/2); 
        PORTE_OUTCLR = alrLED;      // turn alarm LED off
        delay_ms (tick/2);            
    }
    
    // test USB power FET and LED
    for (int i = 0; i<4; i++)
    {
        PORTC_OUTCLR = USBfet;      // turn alarm LED on 
        delay_ms (tick/2); 
        PORTC_OUTSET = USBfet;      // turn alarm LED off
        delay_ms (tick/2);            
    }
    
    // test RTC and seconds LEDs
    RTC_CNT = 0x0100;               // set RTC counter (display one light)
    PORTC_OUTCLR = dig5en;          // enable seconds LEDs
    do
    {
        PORTA_OUT = RTC_CNT>>8;     // display RTC as count
        
    }while ((RTC_CNT>>8) < 0x3f);   // quit on full count

    PORTC_OUTSET = dig5en;          // turn seconds LEDs off
    ei();                           // enable interrupts for normal display
    
    // test buzzer
    for (int i = 0; i<300; i++)
    {
        PORTF_OUTTGL = buzz; 
        delay_us (117);
    }
    delay_ms (100);
    for (int i = 0; i<350; i++)
    {
        PORTF_OUTTGL = buzz; 
        delay_us (100);
    }
    
    // test all segments on digit 5 (individual LEDs and colon)
    for (unsigned char i = 1; i!=0; i<<=1)
    {
        dig5 = i; 
        delay_ms (tick);
    }     
    dig5 = get_segs (' ');   // turn off digit 5
    
    // test all segments on digit 1
    dig1 = 0b00000100;  // segment A
    delay_ms (tick); 
    dig1 = 0b00001000;  // segment B
    delay_ms (tick);  
    dig1 = 0b00100000;  // segment C
    delay_ms (tick); 
    dig1 = 0b00010000;  // segment D
    delay_ms (tick);  
    dig1 = 0b10000000;  // segment E
    delay_ms (tick); 
    dig1 = 0b01000000;  // segment F
    delay_ms (tick);  
    dig1 = 0b00000010;  // segment G
    delay_ms (tick); 
    dig1 = 0b00000001;  // decimal point 
    delay_ms (tick);  
    dig1 = get_segs (' ');  // turn off digit 1
 
    // show all display digits work
    disp_4char ("8   ");
    delay_ms (tick);
    disp_4char (" 8  ");
    delay_ms (tick);
    disp_4char ("  8 ");
    delay_ms (tick);
    disp_4char ("   8");
    delay_ms (tick);
    
    // show decimal points
    disp_4char ("....");
    delay_ms (tick);
    
    // set the pit_pulse pin high and low to verify constants used in delay_us
    PORTD_OUTSET = pit_pulse; 
    delay_us (50); 
    PORTD_OUTCLR = pit_pulse; 
    delay_us (50);
    PORTD_OUTSET = pit_pulse;
    delay_us (1000); 
    PORTD_OUTCLR = pit_pulse; 
       
    // check EEPROM RTC calibration factor
    unsigned_eeprom = eeprom_read_word((unsigned int *)&USERROW_USERROW0);
    if ((~(unsigned_eeprom & 0xFF00))>>8 == (unsigned_eeprom & 0x00FF))
    {
        // eeprom read successful; display new constant
        disp_4char ("CALR");
        delay_ms (4*tick);
        if (unsigned_eeprom & 0b0000000010000000)    // check for negative 
        {
            // convert negative sign and magnitude to 2's complement
            disp_signed_char (-(signed char)(unsigned_eeprom & 0b0000000001111111));
        }
        else
        {
            disp_signed_char (unsigned_eeprom & 0b0000000011111111);
        }
        delay_ms (4*tick);
    }
    else
    {
        disp_string ("no rtc CAL");
        delay_ms (4*tick);
    }
    
    // display calibration factors
    disp_4char ("CALA");
    delay_ms (5*tick);
    disp_float (AVDD_cal, 0);
    delay_ms (5*tick);
    disp_4char ("CALC");
    delay_ms (5*tick);
    disp_float (curr_cal, 0);
    delay_ms (5*tick);
    
    // check AVDD
    disp_4char ("Add=");
    delay_ms (5*tick);
    disp_float (avdd, 0);
    delay_ms (5*tick);
    disp_4char ("    ");
    delay_ms (tick);
    
    // check USB current (should be zero since charge port is off)
    disp_4char ("Iout");
    delay_ms (5*tick);
    disp_float (current, 0);
    delay_ms (5*tick);
    disp_4char ("    ");
    delay_ms (tick);
    
    // blank screen
    disp_4char ("    ");
    delay_ms (tick);
 
    // test buttons 
    disp_string ("PrESS button");
    
    dig4 = 0b00010110;
    do 
    {
        char button; 
        button = PORTE_IN & b_mask; 
        if (button == up)
            dig4 &= 0b11111011; 
        if (button == down)
            dig4 &= 0b11101111; 
        if (button == ok)
            dig4 &= 0b11111101;
    } while (dig4);
}



// displays a splash screen
void splash (void)
{
    disp_4char ("    ");
    delay_ms (tick);
    disp_4char ("   E");
    delay_ms (tick);
    disp_4char ("  ES");
    delay_ms (tick);
    disp_4char (" ESE");
    delay_ms (tick);  
    disp_4char ("ESE ");
    delay_ms (5*tick);
    disp_4char ("    ");
    delay_ms (tick);
    disp_4char ("3   ");
    delay_ms (tick);
    disp_4char ("23  ");
    delay_ms (tick);   
    disp_4char ("123 ");
    delay_ms (tick);
    disp_4char (" 123");
    delay_ms (5*tick);
#ifdef ups
    disp_4char ("2 12");
    delay_ms (tick);
    disp_4char ("72 1");
    delay_ms (tick);   
    disp_4char ("272 ");
    delay_ms (tick);
    disp_4char (" 272");
    delay_ms (5*tick);
#endif
    
    disp_4char ("FALL");
    delay_ms (5*tick);
//    disp_string ("SPring");
    disp_4char ("2024");
    delay_ms (5*tick);
    disp_4char ("    ");
    delay_ms (2*tick);
    
#ifdef ups
    // setup pin change interrupt if ESE272 UPS module is installed
    PORTD_INTFLAGS = 0b11111111;    // clear interrupt flags (precaution)
    PORTD_PIN0CTRL = 0b00000011;    // enable falling edge interrupt on PD0
#endif
}



// this function sets the time
unsigned char set_time (void)
{
    unsigned char mode = 24; 
    unsigned char buttons; 
    signed char hr_disp;
    signed char set_hr;
    signed char set_min;
    signed long error; 
    
    // get 12/24 hour mode
    disp_4char ("24hr");
    do
    {
        // check buttons
        buttons = PORTE_IN & b_mask;
        if ((buttons == up) || (buttons == down)) // change mode on either button
        {
            if (mode == 24) // old mode was 24
            {
                mode = 12;  // new mode is 12
                disp_4char ("12hr");
            }
            else
            {
                mode = 24;  // new mode is 24
                disp_4char ("24hr");
            }
            wait_up ();     // debounce
        }
    } while (buttons != ok);
 
    // pause
    disp_4char ("    ");
    wait_up (); // debounce
    
    // start with current time  
    set_hr = hr;        
    set_min = min; 
    
    // initialize buttons to the not pressed state
    buttons = none; 
    
    // set hours
    do
    {
        hr_disp = set_hr;
        // display hours
        if (mode == 12)
        {
            // format display for 12 hour mode
            if (set_hr == 0)
                hr_disp = 12;
            else if (set_hr > 12)
                hr_disp = set_hr - 12;
        }
        // suppress first digit in 12 hour mode 
        if ((mode == 24) || (hr_disp > 9))
            dig1 = get_segs (hr_disp / 10);
        else
            dig1 = off; // blank
        dig2 = get_segs (hr_disp % 10);
        if (mode == 12) // display am or pm in 12 hour mode
        {
            if (set_hr < 12)
                dig4 = get_segs ('A');  // am
            else
                dig4 = get_segs ('P');  // pm
        }
       
        // check old button state in order to pace the count if held down 
        if ((buttons == up) || (buttons == down)) 
            // buttons are still pressed from last change; pace the count
            delay_ms (200); 
        
        // update buttons        
        buttons = PORTE_IN & b_mask;
        
        if (buttons == up)
        {
            // increment hours
            set_hr++;
            if (set_hr == 24)
                set_hr = 0;
        }
        
        if (buttons == down)
        {
            // decrement hours
            if (set_hr == 0)
                set_hr = 24;
            set_hr--;
        }
    } while (buttons != ok);
    
    wait_up (); // debounce
    
    // initialize buttons to not pressed state. 
    buttons = none; 
    
    // set minutes 
    do
    {
        // display minutes
        dig3 = get_segs (set_min / 10); 
        dig4 = get_segs (set_min % 10);

        // check old button state in order to pace the count if held down 
        if ((buttons == up) || (buttons == down)) 
            // buttons are still pressed from last change; pace the count
            delay_ms (200); 
        
        // update buttons
        buttons = PORTE_IN & b_mask;
        if (buttons == up)
        {
            // increment minutes
            set_min++;
            if (set_min == 60)
                set_min = 0;
        }
        
        if (buttons == down)
        {
            // decrement minutes
            if (set_min == 0)
                set_min = 60;
            set_min--;
        }
    } while (buttons != ok);
    
    // check to see if setting has crossed midnight
    if ((hr < 6) && (set_hr > 18))  
        hr += 24;       // correct sign for subtraction
    // calculate time error (fast clock has positive error)
    error = (hr - set_hr) * 3600L + (min - set_min) * 60 + sec;  
    // modulo 1 hour (ignore DST changes)
    error = error % (60*60);        

    // load new time
    RTC_CNT = 0;    // reset RTC counter
    
    sec = 0;        // reset seconds
    min = set_min; 
    hr = set_hr; 
    
    wait_up ();     // wait for the buttons to be released
    
    // change the crystal calibration factor if it hasn't been set recently
    if (last_set > 86400)
    {
        int rtc_cal; 
        int mag;
        char sign; 
        
        // get old RTC calibration factor (sign and magnitude format)
        rtc_cal = RTC_CALIB & 0b01111111 ;  // get magnitude
        if (RTC_CALIB & 0b10000000)
            rtc_cal *= -1;                  // fix sign
        
        // compute new RTC calibration factor
        rtc_cal += round (1.0e6 * error / last_set); 
        
        // format new RTC calibration factor
        mag = abs (rtc_cal); 
        // check for out of range
        if (mag > 0b01111111)
            mag = 0b01111111; 
        if (rtc_cal < 0)
            sign = 0b10000000; 
        else
            sign = 0; 
        RTC_CALIB = sign | mag; 
        
        // display CAL (will be followed by number or "FAIL")
        disp_4char ("CAL ");
        delay_ms (500);
        
        // save calibration constant to EEPROM
        unsigned int dword;  // make byte into a word which allows verification
        dword = ((~RTC_CALIB) << 8) | RTC_CALIB;    
        eeprom_write_word((unsigned int *)&USERROW.USERROW0, dword);
        while (NVMCTRL_STATUS & NVMCTRL_EEBUSY_bm)
           {}
        
        // read RTC_CALIB back
        dword = eeprom_read_word((unsigned int *)&USERROW_USERROW0);
        if ((~(dword & 0xFF00))>>8 == (dword & 0x00FF))
        {
            // eeprom write successful; display new constant
            if (sign)
                // convert negative sign and magnitude to 2's complement
                rtc_cal = -(rtc_cal & 0b01111111);
            disp_signed_char (rtc_cal);
            delay_ms (1000); 
        }
        else
        {
            // eeprom write failed
            disp_string ("CAL FAIL"); 
        }
    }
    
    // reset the last set time
    last_set = 0; 

    return (mode); 
}



// this function sets the alarm time
void set_alarm (unsigned char *al_hr, unsigned char *al_min, unsigned char mode)
{
    unsigned char buttons = none; 
    signed char hr_disp;
    
    // set alarm hours
    do
    {
        hr_disp = *al_hr;
        // display hours
        if (mode == 12)
        {
            if (*al_hr == 0)
                hr_disp = 12;
            else if (*al_hr > 12)
                hr_disp = *al_hr - 12;
        }
        // suppress first digit in 12 hour mode 
        if ((mode == 24) || (hr_disp > 9))
            dig1 = get_segs (hr_disp / 10);
        else
            dig1 = off; // blank
        dig2 = get_segs (hr_disp % 10);
        dig3 = off; 
        if (mode == 12) // display am or pm in 12 hour mode
        {
            if (*al_hr < 12)
                dig4 = get_segs ('A');  // am
            else
                dig4 = get_segs ('P');  // pm
        }
        else // 24 hour mode
        {
            dig4 = off; 
        }
            
        // check old button state in order to pace the count if held down 
        if ((buttons == up) || (buttons == down)) 
            // buttons are still pressed from last change; pace the count
            delay_ms (200); 
        
        // update buttons        
        buttons = PORTE_IN & b_mask;
        if (buttons == up)
        {
            // increment hours
            *al_hr = *al_hr + 1;
            if (*al_hr == 24)
                *al_hr = 0;
        }
        
        if (buttons == down)
        {
            // decrement hours
            if (*al_hr == 0)
                *al_hr = 24;
            *al_hr = *al_hr - 1;
        }
    } while (buttons != ok);
    
    wait_up (); // debounce
    
    // initialize buttons to not pressed state. 
    buttons = none; 
    
    // set alarm minutes 
    do
    {
        // display minutes
        dig3 = get_segs (*al_min / 10); 
        dig4 = get_segs (*al_min % 10);

        // check old button state in order to pace the count if held down 
        if ((buttons == up) || (buttons == down)) 
            // buttons are still pressed from last change; pace the count
            delay_ms (200); 
        
        // update buttons
        buttons = PORTE_IN & b_mask;
        if (buttons == up)
        {
            // increment minutes
            *al_min = *al_min + 1;
            if (*al_min == 60)
                *al_min = 0;
        }
        
        if (buttons == down)
        {
            // decrement minutes
            if (*al_min == 0)
                *al_min = 60;
            *al_min = *al_min - 1;
        }
    } while (buttons != ok);
    wait_up ();     // wait for the buttons to be released
    return; 
}



// choose charge mode
unsigned char choose_charge (void)
{
    unsigned char mode = 1; 
    unsigned char buttons; 
    unsigned char et = 0; 
    dig1 = get_segs ('c'); 
    dig2 = get_segs ('h');
    dig3 = get_segs (' ');
    do
    {
        delay_ms (10);                  // delay    
        et++;                           // increment elapsed time
        // check buttons
        buttons = PORTE_IN & b_mask;
        if (buttons == up) 
        {
            mode++; 
            if (mode == 4)
                mode = 1; 
            et = 0;                     // reset timer
        }
        if (buttons == down)
        {
            if (mode == 1)
                mode = 4; 
            mode--;
            et = 0;                     // reset timer
        }
        dig4 = get_segs (mode);         // display mode number
        // use individual LEDs to present a graphical representation
        switch (mode)
        {
            case 0:
                dig5 = 0b00000000;
                break;
            case 1:
                dig5 = 0b00110000;
                break;
            case 2:
                dig5 = 0b00110011; 
                break; 
            case 3: 
                dig5 = 0b00111111; 
                break;
        }
        wait_up (); // debounce
    } while ((buttons != ok) && (et<200));
    wait_up (); // wait until buttons are released
    return (mode);
}



// interrupt service routine for real time counter
// this increments the time on overflow each second
void __interrupt (RTC_CNT_vect_num) rtc_isr (void)
{
    // clear interrupt flags
    RTC_INTFLAGS = 0b00000011; 
    
    // set heartbeat on otherwise unused PD7 to check timing accuracy
    PORTD_OUTSET = rtc_pulse; 
    
    // increment loop counter if power is okay
    if (pwr_ok)
        loop_count++;   // increment loop counter
    
    // increment time
    sec++;
    if (sec == 60)
    {
        sec = 0;
        min++;
        if (min == 60)
        {
            min = 0; 
            hr++;
            if (hr == 24)
            {
                hr = 0;
                day++;
            }
        }
    }
    
    // update global variables
    last_set++;                     // increment the seconds since last set
    current = curr_acc / curr_cnt;  // update averaged current
    curr_acc = 0;                   // reset current accumulator
    curr_cnt = 0;                   // reset current counter
    avdd = avdd_acc / avdd_cnt;     // update averaged voltage
    avdd_acc = 0;                   // reset voltage accumulator
    avdd_cnt = 0;                   // resent current counter
    total_energy += current * avdd; // calculate total energy    
    
    // reset heartbeat
    PORTD_OUTCLR = rtc_pulse; 
}



// interrupt service routine for periodic interval timer
// this drives the multiplex display
void __interrupt (RTC_PIT_vect_num) pit_isr (void)
{
    // clear interrupt flags
    RTC_PITINTFLAGS = 0b00000001;
    
    // provide heartbeat on cnt_pulse pin
    PORTD_OUTSET = pit_pulse;       

    // increment digit counter
    if (digit_cnt == 5)
        digit_cnt = 1; 
    else 
        digit_cnt++;
    
    // turn off all digits
    PORTC_OUTSET = 0b00011111;
   
    switch(digit_cnt) 
    {
        case 1:
           PORTA_OUT = dig1; 
           PORTC_OUTCLR = dig1en; 
           break;
        case 2:
           PORTA_OUT = dig2; 
           PORTC_OUTCLR = dig2en; 
           break;
        case 3:
           PORTA_OUT = dig3; 
           PORTC_OUTCLR = dig3en; 
           break;
        case 4:
           PORTA_OUT = dig4; 
           PORTC_OUTCLR = dig4en; 
           break;   
        case 5:
           PORTA_OUT = dig5; 
           PORTC_OUTCLR = dig5en; 
           break;
    }        
    
    switch (adc_pending)
    {
        case init: 
            init_avdd ();
            adc_pending = volt;
            break; 
        case volt: 
            if (ADC0_COMMAND == 0)          // if conversion is complete
            {
                qavdd = calc_AVDD ();       // get AVDD
                avdd_acc += qavdd;          // add to accumulator
                avdd_cnt++;                 // increment counter
                init_curr ();               // start current measurement
                adc_pending = cur;          // pending current measurement
            }
            break; 
        case cur: 
            if (ADC0_COMMAND == 0)          // if conversion is complete
            {
                curr_acc += calc_curr ();   // add to accumulator 
                curr_cnt++;                 // increment counter
                adc_pending = vccn;         // pending configuration voltage measurement
            }
            break; 
        case vccn: 
            vcc1 = measure_VCC1 ();         // measure the voltage of CC1
            vcc2 = measure_VCC2 ();         // measure the voltage of CC2
            init_avdd ();                   // start AVDD voltage measurement
            adc_pending = volt;             // identify pending measurement
            break; 
    }
    
    // provide heartbeat on pit_pulse pin
    PORTD_OUTCLR = pit_pulse;  
}


#ifdef ups
// interrupt service routine for PORTD pin change
// this is used when the ESE 272 add on is installed
void __interrupt (PORTD_PORT_vect_num) PD0_isr (void)
{
    // VDD is falling: SCRAM!
    PORTC_OUTSET = 0b00111111;      // turn off display and USB port
    PORTE_OUTCLR = alrLED;          // turn off alarm light
    RTC_PITINTCTRL = 0b00000000;    // digit interrupt disabled
    
    // set diagnostic flag for scope timing measurements
    PORTD_OUTSET = BOD_pulse;       // diagnostic bit high
    
    // clear interrupt flags
    PORTD_INTFLAGS = 0b00000001;
    
    // signal that power is bad
    pwr_ok = 0;
    
    // clear diagnostic flag
    PORTD_OUTCLR = BOD_pulse;       // diagnostic bit low 
}
#endif


//// voltage level monitor interrupt handler. 
//void __interrupt (BOD_VLM_vect_num) bod_isr (void)
//{
//}