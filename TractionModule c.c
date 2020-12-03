/////////////////////////////////////////////////////////////////
// Title : TractionPIC  V1                                     //
/////////////////////////////////////////////////////////////////
// File <TractionModule c.c>     Device : PIC 16F88
//
// This is a model railway 12v analog speed controller, using PWM.
// There are four input speeed settings; off, crawl, mid, fast.
// PWM duty cycle changes gradually between speed steps.
// There is over current limiting
//
// PIC inputs :-
//	Speed2 & Speed1; they set 4 speeds.
//	Analog current sense across 1ohm resistor, inline with MOSFET.
//  Analog Trim pot.
//
// PIC outputs :-
//	PWM to MOSFET which drives track supply, also drives green LED.
//	onboard yellow LED - lit when at stable speed, off when not.
//	onboard red LED - lit when over current condition.
//	AlarmLED to external LED (pin shared with ICSP data)
//  Chuff signal to external sound module (pin shared with ICSP clock)
//
//  Environment : Hi-Tech C PRO(lite) compiler in Microchip MPLAB IDE
#include <htc.h>
//
//       Author : Dave Harris, Basingstoke, UK
// Initial Date : 30-Jan-2011
// 
// Version Date : 13-Feb-2011 V1.001 - 1st release
//
__IDLOC7( 1, 0, 0, 1 ) ; // Current version info in the PIC IDLOCs

/////////////////////////////////////////////////////////////////
// Device configuration                                        //
/////////////////////////////////////////////////////////////////

#define _XTAL_FREQ  8000000 // 8 Mhz internal RC osc

__CONFIG( CCPRB0 & MCLREN & LVPDIS & PWRTDIS & WDTDIS & INTIO );

/////////////////////////////////////////////////////////////////
// EEPROM data                                                 //
/////////////////////////////////////////////////////////////////
// Store title & copyright info in the EEPROM. No other use.
__EEPROM_DATA('T', 'r', 'a', 'c', 't', 'i', 'o', 'n') ;
__EEPROM_DATA('P', 'I', 'C', ' ', 'V', '1', ' ', ' ') ;
__EEPROM_DATA(' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ') ;
__EEPROM_DATA(' ', 'C', ' ', '2', '0', '1', '1', ' ') ;
__EEPROM_DATA('D', 'a', 'v', 'e', ' ', 'H', 'a', 'r') ;
__EEPROM_DATA('r', 'i', 's', ' ', ' ', ' ', ' ', ' ') ;
__EEPROM_DATA(' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ') ;

/////////////////////////////////////////////////////////////////
// Global variables & definitions                              //
/////////////////////////////////////////////////////////////////
#define Input  1  // TRIS values
#define Output 0

#define true  1   // general logic values
#define false 0

volatile bit FlagOverCurrent = false ; // Is 'volatile' as set in the ISR.

unsigned char CurrentDutyCycle = 0 ; // set by main & used by ISR

#define DutyCycleMin 13  // 5% or 26 uS PWM 'on' time
#define DutyCycleMax 249 // PWM freq to be 2 KHz.
#define CurrentLimit 35  // see note in ADC section in device_init() 

// The Functions...

/////////////////////////////////////////////////////////////////
void	device_init( void )
/////////////////////////////////////////////////////////////////
{   // Called from main() only.

	IRCF0 = true ; // until this point the clock was 31 KHz!
	IRCF1 = true ; // ...
	IRCF2 = true ; // IRCF<2:0> = 7, is 8Mhz internal RC clock

// clear Port bit latches

    PORTA = 0x00 ;
    PORTB = 0x00 ;

// set OPTION reg
	RBPU   = false ; // PortB pullup resistors are on
	INTEDG = true  ; // RB0/INT interrupt edge is rising

// set port status - input, output or analog

	ANSEL = 0x00 ; // Set all analog inputs off

// Port A directions

	TRISA0 = Input ;  // pin 17 - AN0 - current sense

	TRISA1 = Output ; // pin 18 - RA1 - diagnostic - accelerate
	#define opAccel RA1

	TRISA2 = Output ; // pin 1 - RA2 - pot high
	RA2 = 1 ; // provide 5v to the pot

	TRISA3 = Input  ; // pin 2 - AN3 - pot wiper

	TRISA4 = Output ; // pin 3 - unused

	TRISA5 = Input ;  // pin 4 - RA5 - is set as MCLR by CONFIG MCLREN

	TRISA6 = Output ; // pin 15
	#define opYellowLED RA6 

	TRISA7 = Output ; // pin 16
	#define opRedLED RA7 

// Port B directions

	TRISB0 = Output ; // pin 6 - CCP1 set by CONFIG CCPRB0 and by CCP1CON as PWM

	TRISB1 = Output ; // pin 7 - unused

	TRISB2 = Input ;  // pin 8 - Speed 2^1
	#define ipSpeed2 RB2

	TRISB3 = Input ;  // pin 9 - Speed 2^0
	#define ipSpeed1 RB3

	TRISB4 = Output ; // pin 10 - unused

	TRISB5 = Output ; // pin 11 - unused

	TRISB6 = Output ; // pin 12 - ICSP Clock & Chuff signal
	#define opChuff  RB6

	TRISB7 = Output ; // pin 13 - ICSP Data and AlarmLED output
	#define opAlarmLED RB7

// configure ADC module

	ANS0  = true  ; // enable analog input on AN0 which is on RA0
	ANS3  = true  ; // enable analog input on AN3 which is on RA3

	ADCON1 = 0x00 ; // Vref+ = VDD, Vref- = VSS
	ADCS2 = true  ; // A/D clock div by 2 - see also ADCON0 setting.
	ADFM = false  ; // left justify ADRESH:ADRESL (10 bit result).
					// Note on AD results...
				// 0.7 volt input for current limit on AN0 (0.7 amp)
				// so ADRESH:ADRESL = 0 to 143 but left shift
				// this result fits into ADRESH as 0 to 35.

// configure CCP module as PWM

	PR2 = DutyCycleMax ; // PWM freq to be 2 KHz
	CCPR1L  = 0x00 ;	// duty cycle = 0% for now
	CCP1CON = 0x00 ;	// CCP module disabled (mode = 0000h) 
	CCP1M2  = true ;	//
	CCP1M3  = true ;	// M3:M0 = 11XXh = CCP in PWM mode
	T2CKPS0 = true ;	// T2CON prescaler is 4
	TMR2ON  = true ;	// Timer 2 is on

// set interrupts

	INTCON = false ; // ensure all interrupt enable are off
	INT0IE = true  ; // RB0/INT interrupt enable - when CCP/RB0 goes high
					 // enable global interrupts comes later...
}
// end of device_init()

/////////////////////////////////////////////////////////////////
unsigned char	ADCread( char ADchan )
/////////////////////////////////////////////////////////////////
{   // Called from main() and InterruptServiceRoutine()
    // ANSEL and ADCON1 are already set.
 
	ADCON0 = 0x01 ; // clear ADCON0 but set AD on
	ADCS1 = false ; //
	ADCS0 = true  ; // Fosc/16 - in conjunction with ADCS2 in ADCON1

	if( ADchan == 3 ) // defaults as chan 0
	{
		CHS0 = true ;
		CHS1 = true ; // 0b00000011 is 3
	}
	__delay_us( 25 ) ; 		 // allow acquisition time

	ADGO = true ;			 // conversion start
	while( ADGO ) continue ; // loop until ready 	
	return ADRESH ;			 // return the AD result
}
// end of ADCread()

/////////////////////////////////////////////////////////////////
void	main( void )
/////////////////////////////////////////////////////////////////
{
	unsigned char TargetDutyCycle ; // the target duty cycle value
	unsigned char TrimPotValue ; // the value from the trim pot
	#define TrimPotMax 82  // 249 / 3 -1 = 82

	device_init() ;

	while( true ) // forever
    {
		di() ; // global interrupt enable off
		TrimPotValue = ADCread( 3 ) >> 1  ; // div 2
		ei() ; // global interrupt enable on		
		if( TrimPotValue > TrimPotMax ) // H/W should take care of this,
		{ // but...
			TrimPotValue = TrimPotMax ;
		}
		switch( ( ipSpeed2 << 1 ) + ipSpeed1 )
		{
			case 0:
				TargetDutyCycle = 0x00 ;
				break ;
			case 1:
				TargetDutyCycle = TrimPotValue ;
				break ;
			case 2:
				TargetDutyCycle = TrimPotValue * 2 ;
				break ;
			case 3:
				TargetDutyCycle = TrimPotValue * 3 ;
		} 
		if( CurrentDutyCycle > TargetDutyCycle )
		{		// need to decrease the duty cycle
			if( CurrentDutyCycle < DutyCycleMin )
			{
				CurrentDutyCycle = 0x00 ;
			}
			else
			{
				--CurrentDutyCycle ; // decrement speed
			}
			opAccel = false ;
			opYellowLED = false ;
		}
		else if( CurrentDutyCycle < TargetDutyCycle )
		{			 // need to increase the duty cycle
			if( CurrentDutyCycle == 0x00 )
			{
				CurrentDutyCycle = DutyCycleMin ;
				opChuff = true ;
			}
			else
			{
				++CurrentDutyCycle ; // increment speed
			}
			opAccel = true ;
			opYellowLED = false ;
		}
		else // Current duty cycle is same as Target
		{
			opYellowLED = true ; 
			opAccel = false ;
			opChuff = false ; // if chuff was on... set it off
		}

		if( !FlagOverCurrent )
		{
			opAlarmLED = false ;
			opRedLED = false ;
			CCPR1L = CurrentDutyCycle ; // set the PWM duty cycle
		}
		__delay_ms( 25 ) ;
	}
}
// end of main()

/////////////////////////////////////////////////////////////////
void interrupt	InterruptServiceRoutine( void )
/////////////////////////////////////////////////////////////////
{   // RB0 INT is when the PWM signal rises...
    //   time to check what current is flowing
	if( INT0IE && INT0IF )
	{
		if( ADCread( 0 ) > CurrentLimit )
		{
			opAlarmLED = true ; // tell the operater panel
			opRedLED = true ;
			FlagOverCurrent = true ;
			CCPR1L = DutyCycleMin ; // set a low duty cycle for the next cycle
		}
		else
		{
			FlagOverCurrent = false ;   // clear the over current flag
			CCPR1L = CurrentDutyCycle ; // reset the duty cycle
		}		
		INT0IF = false ; // clear the INT flag
	}
}
// end of InterruptServiceRoutine()

/////////////////////////////////////////////////////////////////
//          end of file TractionModule c.c                     //
/////////////////////////////////////////////////////////////////