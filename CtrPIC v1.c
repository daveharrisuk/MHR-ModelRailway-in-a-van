/////////////////////////////////////////////////////////////////
// Title : CtrPIC  v1                                          //
/////////////////////////////////////////////////////////////////
// File <CtrPIC v1.c>     Device : PIC16F877A
//
// This is the public controller for the MHR layout.
// ....blah blah....
//
// 
// PIC inputs :-
//  Port B is the 8 off push buttons
//  Port A bit 5 is the sound card Busy input
//
// PIC outputs :-
//  Port A bits 0 to 3 are sound card drivers
//  Port D bits 0 to 3 are the display drivers (and hence to the panel LEDs)
//  Port E bit 0 is a diagnostic yellow LED
//
//
//  Environment : Hi-Tech C PRO(lite) compiler in Microchip MPLAB IDE
#include <htc.h>
// which includes pic.h, which includes pic168xa.h for the PIC16F877A defined by the project)
//
//       Author : Russ & Rob Score, Basingstoke, UK
// Version & Date : v0.002 10-Jun-11 initial build of template
//
//
__IDLOC7( 0, 0, 0, 3 ) ; // Current version info in the PIC IDLOCs

/////////////////////////////////////////////////////////////////
// EEPROM data                                                 //
/////////////////////////////////////////////////////////////////
// Store title & copyright info in the EEPROM. Byte 0 is save config.
__EEPROM_DATA('0', ' ', 'C', 't', 'r', 'P', 'I', 'C') ;
__EEPROM_DATA(' ', 'f', 'o', 'r', ' ', 'M', 'H', 'R') ;
__EEPROM_DATA('.', ' ', 'v', '0', '.', '0', '0', '3') ;// version
__EEPROM_DATA(' ', 'C', ' ', '2', '0', '1', '1', ' ') ;
__EEPROM_DATA('R', 'u', 's', 's', ' ', '&', ' ', 'R') ;
__EEPROM_DATA('o', 'b', ' ', 'S', 'c', 'o', 'r', 'e') ;
__EEPROM_DATA(' ', 'B', 'N', 'H', 'M', 'R', 'S', '.') ;

/////////////////////////////////////////////////////////////////
// Device configuration                                        //
/////////////////////////////////////////////////////////////////

#define _XTAL_FREQ  4000000 // external 4 MHz crystal osc, mode = XT

__CONFIG(XT & BORDIS & LVPDIS & PWRTDIS & WDTDIS & DEBUGEN & UNPROTECT);

/////////////////////////////////////////////////////////////////
// Global variables & definitions                              //
/////////////////////////////////////////////////////////////////

// TRIS direction
#define Input	0xFF
#define Output	0

			unsigned char OperatingMode ; // as saved in EEPROM address 0

			unsigned char SaveLEDport ; 

volatile	unsigned char CommsTxChar	= 'A' ; // Acknowledge 
volatile	unsigned char CommsRxChar ;

volatile	unsigned char Clock5mS ;  // incremented by timer2 interrupt 
volatile	unsigned char Clock1Sec ; //  as is this. (nb: after 255, it goes to zero)

			unsigned char tmp ;

// The code procedures...

/////////////////////////////////////////////////////////////////
void	device_init( void )
/////////////////////////////////////////////////////////////////
{   // Called from main() only.

// preset all the Ports bit latches
	PORTA = 0xFF ; // set sound 1 port bits high
    PORTB = 0x00 ;
    PORTC = 0xFF ; // set sound 2 port bits high
    PORTD = 0x00 ;
    PORTE = 0x00 ;

// set OPTION reg
	RBPU   = 0 ; // PortB pullup resistors

// set ports status - input, output or analog

	ADCON0 = 0x00 ; // Set all analog inputs off

/////////////////////////////////
// Port A directions and defines - sound port #1

	#define Soundport1 PORTA

	TRISA0 = Output ; // RA0 pin 2 - Sound 1 Next button
	#define bSound1Next	0x01

	TRISA1 = Output ; // RA1 pin 3 - Sound 1 Play button
	#define bSound1Play	0x02

	TRISA2 = Output ; // RA2 pin 4 - Sound 1 ChipEnable
	#define bSound1CE	0x04

	TRISA3 = Input ;  // RA3 pin 5 - Sound 1 Busy LED
	#define bSound1Busy 0x10

	// RA4 pin 6 (nb: open drain type of output) // don't use
	// RA5 pin 7	// don't use
	//	port A6 and A7 pins are not on this chip.

/////////////////////////////////
// Port B directions and defines
// this port has pull up resistors, so is for the panel push buttons

	#define Switchport PORTB

	TRISB = Input ; // all inputs

	// RB0 pin 33
	#define ipSw0 RB0

	// RB1 pin 34
	#define ipSw1 RB1

	// RB2 pin 35
	#define ipSw2 RB2

	// RB3 pin 36
	#define ipSw3 RB3

	// RB4 pin 37
	#define ipSw4 RB4

	// RB5 pin 38
	#define ipSw5 RB5

	// RB6 pin 39 - (PGC Clock when ICSP) 
	#define ipSw6 RB6

	// RB7 pin 40 - (PGD Data when ICSP) 
	#define ipSw7 RB7

/////////////////////////////////
// Port C directions and defines - sound port #2

	#define Soundport2 PORTC

	TRISC0 = Output ; // RC0 pin 15
	#define bSound2Play	0x01

	TRISC1 = Output ; // RC1 pin 16
	#define bSound2CE	0x02

	TRISC2 = Input ;  // RC2 pin 17
	#define bSound2Busy	0x04

	TRISC3 = Output ; // RC3 pin 18
	#define bSound2Next	0x08

	TRISC4 = Output ; // RC4 pin 23	// not used - don't use
	TRISC5 = Output ; // RC5 pin 24	// not used - don't use

	// RC6 pin 25 - USART-RS232 Tx
	// RC7 pin 26 - USART-RS232 Rx 
	// don't use RC6 & RC7 in code, as are assigned to USART.

/////////////////////////////////
// Port D directions and defines - is the LED port

	#define LEDport	PORTD

	TRISD = Output ; // all are outputs
	// RD0 pin 19, RD1 pin 20, RD2 pin 21 & RD3 pin 22 & RD4 pin 27, RD5 pin 28, RD6 pin 29 - used 
	// RD7 pin 30 - not used

	// panel LEDs on RD0, RD1, RD2 & RD3 & RD4 & RD5
	#define LEDred    0x01
	#define LEDyellow 0x02
	#define LEDgreen1 0x04
	#define LEDgreen2 0x08
	#define LEDmode1  0x10
	#define LEDmode2  0x20
	#define Buzzer	  0x40  // buzzer on RD6 pin 29

/////////////////////////////////
// Port E directions and defines - is the diagnostic port.

	#define Diagport PORTE

	TRISE = Output ; // all outputs

	// RE0 pin 8   - yellow diagnostic LED - a 1 activates
	#define opDiag0 RE0

	// RE1 pin 9  - red diagnostic LED - a 1 activates
	#define opDiag1 RE1

	// RE2 pin 10 - not in use

//////////////////////////////////
// On power-up LED self tests

	opDiag0 = 1 ;
	LEDport = LEDred    ; // flash red once on powerup
	__delay_ms( 150 ) ;
	__delay_ms( 150 ) ;
	LEDport = LEDgreen1 | LEDmode1 ; // flash green1 once on powerup
	opDiag0 = 0 ;
	opDiag1 = 1 ;
	__delay_ms( 150 ) ;
	__delay_ms( 150 ) ;
	opDiag1 = 0 ;
	LEDport = LEDgreen2 | LEDmode2 ; // flash green2 once on powerup
	__delay_ms( 150 ) ;
	__delay_ms( 150 ) ;
	LEDport = LEDyellow ; // then show yellow/busy...

///////////////////////////////////////////////
// setup Timer2 as 5 mS clock

	T2CON = 0x0F ; // TMR2ON, prescalar 1:16, postscaler = 1:2 = 0.032 mS

	#define Timer2count 156
	PR2 = Timer2count ; // = 0.032 mS times = 4.992 mS ... close enough.
	TMR2IE = 1 ; // Timer2 interrupt enable

///////////////////////////////////////////////
// setup USART for async comms from RS232

	BRGH = 1 ; // Baud rate for USART comms - 9600 bps
	SPBRG = 25 ; // see tables on data sheet chapter 10.1
	SYNC = 0 ; // USART in async mode
	SPEN = 1 ; // serial port enable - switch port RC6 & RC7 to the USART
	TXEN = 1 ; // enable transmission
	CREN = 1 ; // enable reception
	RCIE = 1 ; // USART Receive interrupt enable

//////////////////////////
// set interrupt enables

	PEIE = 1 ; // peripheral interrupt enable
	ei() ;     // global interrupt enable (macro from include pic.h )
}
// end of device_init()

////////////////////////////////////////////////////////////////
void	saveOperatingMode(  unsigned char modeChar )
////////////////////////////////////////////////////////////////
{
	eeprom_write( 0 , modeChar ) ; // save in EEPROM address 0
}
// end of saveOperatingMode()

/////////////////////////////////////////////////////////////////
void	sound1cmd( unsigned char bSound )
/////////////////////////////////////////////////////////////////
{
	// The only valid bSound values are bSound1Next or bSound1Play
	Soundport1 = ~bSound & ~bSound1CE  ; // set low;
	__delay_ms( 100 ) ;
	Soundport1 = ~bSound1CE ; 	// set high
 	opDiag0 = 1 ;
	do
	{	tmp = Soundport1 ;	}
	while( ! ( tmp & bSound1Busy ) ) ;
	opDiag0 = 0 ;
	__delay_ms( 180 ) ;
}
// end of sound1cmd()

/////////////////////////////////////////////////////////////////
void	sound2cmd( unsigned char bSound )
/////////////////////////////////////////////////////////////////
{
	// The only valid bSound values are bSound2Next or bSound2Play
	Soundport2 = ~bSound & ~bSound2CE  ; // set low;
	__delay_ms( 100 ) ;
	Soundport2 = ~bSound2CE ; 	// set high
 	opDiag0 = 1 ;
	do
	{	tmp = Soundport2 ;	}
	while( ! ( tmp & bSound2Busy ) ) ;
	opDiag0 = 0 ;
	__delay_ms( 180 ) ;
}
// end of sound2cmd()

/////////////////////////////////////////////////////////////////
void	sound1reset( void )
/////////////////////////////////////////////////////////////////
{
	Soundport1 = 0xFF ; // all pins high, including CE
	__delay_ms( 100 ) ;
	Soundport1 = ~ bSound1CE ; // set just CE to low
	__delay_ms( 50 ) ;
}
// end of sound1reset()

/////////////////////////////////////////////////////////////////
void	sound2reset( void )
/////////////////////////////////////////////////////////////////
{
	Soundport2 = 0xFF ; // all pins high, including CE
	__delay_ms( 100 ) ;
	Soundport2 = ~ bSound2CE ; // set just CE to low
	__delay_ms( 50 ) ;
}
// end of sound2reset()


/////////////////////////////////////////////////////////////////
void	SoundBuzzer( void )
/////////////////////////////////////////////////////////////////
{
	SaveLEDport = LEDport ;
	LEDport = LEDred | Buzzer ;
	__delay_ms( 180 ) ;
	__delay_ms( 180 ) ;
	LEDport = SaveLEDport ;
}
// end of SoundBuzzer()

/////////////////////////////////////////////////////////////////
void	main( void )
/////////////////////////////////////////////////////////////////
{
	OperatingMode = eeprom_read( 0 ) ; // this is the saved mode from before this reboot 
	device_init();
	SoundBuzzer();
	sound1reset();
	sound2reset();

	TXREG = '>' ; // send character to SeqPic to notify of power-up

	while( 1 ) // forever loop doing the main CtrPIC code - no exit!
    {
		opDiag1 = 1 ; // flash red diag LED
		__delay_ms( 50 ) ;
		opDiag1 = 0 ;
		__delay_ms( 100 ) ;

		if( Switchport == 0xFF ) // no switches closed
		{
			LEDport = LEDyellow ;
		}
		else if( Switchport == 0xFE )
		{
			LEDport = LEDgreen1 ; // sw 0
		}
		else // sw 1 to 7
		{
			LEDport = LEDmode1 | LEDmode2 ;
		}

		if( CommsTxChar == '~' )
		{
			CommsTxChar = 'A' ; // reset reply character
			if( CommsRxChar == '3' )
			{
				sound2cmd( bSound2Next ) ; // skip message 1 card 2
				sound2cmd( bSound2Next ) ; // skip message 2 card 2
				sound2cmd( bSound2Play ) ; // play message 3 card 2
				sound2reset();
			}
			else if( CommsRxChar == '2' )
			{
				sound2cmd( bSound2Next ) ; // skip message 1 card 2
				sound2cmd( bSound2Play ) ; // play message 2 card 2
				sound2reset();
			}
			else if( CommsRxChar == '1' )
			{
				sound2cmd( bSound2Play ) ; // play message 1 card 2			
			}
			else
			{
				sound1cmd( bSound1Play ) ; // play message 1 card 1
			}
		}
	}
}
// end of main()

/////////////////////////////////////////////////////////////////
void interrupt InterruptServiceRoutine( void )
/////////////////////////////////////////////////////////////////
{
	static unsigned char TMR2count ;
	static unsigned char TimeOutRS232 ;

	if( RCIF ) // Receive character on USART from RS232
	{
		CommsRxChar = RCREG ;
		TXREG = CommsTxChar ; // send the response character back 
		CommsTxChar = '~' ; // notify main() of new input char
		TimeOutRS232 = 0 ;
	}
	if( TMR2IF ) // Timer2 finished its count, every 5 mS
	{
		TMR2IF = 0 ; // reset the interrupt flag
		Clock5mS++ ; // global 5ms conter
		TMR2count++ ;
		if( TMR2count > 200 ) // 200 * 5 mS = 1 second
		{
			Clock1Sec++ ; // global second counter
			TimeOutRS232++ ;
			if( TimeOutRS232 > 60 )
			{ // too long, do something about it...
				LEDport = LEDred ; // set red LED for now..
			} // in the end probably best to not exit this!
			TMR2count = 0 ;
		}
	}
}
// end of InterruptServiceRoutine()

////////////////////////////////////
//          end of file           //
////////////////////////////////////