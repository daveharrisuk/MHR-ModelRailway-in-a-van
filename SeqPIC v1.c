/////////////////////////////////////////////////////////////////
// Title : SeqPIC  v1                                          //
/////////////////////////////////////////////////////////////////
// File <SeqPIC v1.c>     Device : PIC16F877A
//
// This is the sequence controller for the MHR layout, aka SeqPIC.
//
// Operating modes are set by a Hex rotary switch setting :-
//	1/ Manual control - the maintenace mode to allow setup and testing
//  2/ Special Auto mode - no public interaction - prove layout opertions
//  3/ Normal Automatic mode - moves are controlled by public
//  Interactions with CtrPIC via RS232 - notifies next step and waits for Go command 
//  Mode 3 has 2 modes controlled within the CtrPIC - SeqPIC doesnt know about these.
//
// There are settings for 2 or 3 trains - setting is saved in EEPROM
//
// PIC inputs :-
//  Hex rotary Switch and set and clr pushbuttons
//  Battery monitor - powers down layout when battery is low
//  8 IR sensors - the eyes of the system (on layout)
//	RS232 Rx comms line - get commands from the CtrPIC
//
// PIC outputs :-
//  4 section relays (on layout)
//  1 layout power on relay (onboard and via FET)
//  1 buzzer (onboard and via FET)
//  4 signals (on layout)
//  1 direction signal(on layout, drives 2 turnout servos and reversing relay) 
//	RS232 Tx comms line - issues step status to CtrPIC
//  1 diagnostic LED - when lit shows Automatic mode
//  1 Error LED
//
//  Environment : Microchip MPLAB IDE v8.76
//    plus  HI-TECH C Compiler for PIC10/12/16 MCUs (Lite Mode)  V9.82
//
#include <htc.h>
// which includes pic.h, which includes pic16f877a.h
//   (as defined by the project parameters)
//
//  Author : Dave Harris, Basingstoke, UK
// v0.001 05-May-11 initial build of template
// v0.003 26-Aug-11 sequence coding
// v0.004 04-Sep-11 low battery shut down
// v0.010 09-Sep-11 upgraded MPLAB & Hi-Tech C - Reg defines changed.
// v0.011 21-Sep-11 silence buzzer added hex 7
// v0.020 30-Dec-11 dispatch tables changed
// v0.030 22-Jul-12 re-write RequestGo() as it was a mess
// v0.031 28-Jul-12 further mod to RequstGo
// v0.032 02-Aug-12 diagnostic RS232 '.' rx gives ':' on tx

__IDLOC7( 0, 0, 3, 2 ); // Current version info in the PIC IDLOCs
//
//////////////////////////////////////////////////////
// Device configuration								//
//////////////////////////////////////////////////////
//
#define _XTAL_FREQ  4000000 // external 4 MHz crystal osc, mode = XT
//
__CONFIG( BOREN_OFF & LVP_OFF & PWRTE_OFF & WDTE_OFF & FOSC_XT );
//
//////////////////////////////////////////////////////
// EEPROM data										//
//////////////////////////////////////////////////////
// Store title & copyright info in the EEPROM. Addr 0 is train count char
__EEPROM_DATA('3', ' ', 'S', 'e', 'q', 'P', 'I', 'C');
__EEPROM_DATA(' ', ' ', ' ', '.', ' ', ' ', ' ', '.');
__EEPROM_DATA(' ', 'v', '0', '.', '0', '3', '1', ' ');
__EEPROM_DATA(' ', 'C', ' ', '2', '0', '1', '1', ' ');
__EEPROM_DATA('D', 'a', 'v', 'e', ' ', 'H', 'a', 'r');
__EEPROM_DATA('r', 'i', 's', '.', ' ', ' ', ' ', ' ');

///////////////////////////////////////////////////////
// Global variables & definitions					 //
///////////////////////////////////////////////////////

#define true	1  // general logic values
#define false	0

#define on  	1  // general logic values
#define off 	0

#define Input	0xFF // TRIS direction in
#define Output	0    // TRIS direction out

typedef  unsigned char byte;

volatile byte Clock5mS;  // incremented by timer2 interrupt 
volatile byte Clock1Sec; //... as is this (after 255, they go to zero)

volatile byte CommsRxChar; // set by ISR

        byte IRmask;
        byte PortAimage;
        byte SpeedVal = 0;
		byte Silence = 0;
        char NumberOfTrains;

// The code procedures...

/////////////////////////////////////////////////////////////////
void	device_init( void )
{//---------------------------------------------------------------

    PORTA = 0x00; // clear all Ports bit latches
    PORTB = 0x00;
    PORTC = 0x00;
    PORTD = 0x00;
    PORTE = 0x00;

//--------------------------
// set up ADC 

	ADCON0 = 0x79; // 8tosc, ADon and chan 7 = RE2 - pin 10
	ADCON1 = 0; // Left justified, 8 bit result will be in ADRESH

// set port pins status - input, output or analog

//-----------------------------------
// Port A directions and defines

	TRISA = Output ;
	//TRISA0 = Output ;  // pin 2
	//TRISA1 = Output ;  // pin 3
	//TRISA2 = Output ;  // pin 4
	//TRISA3 = Output ;  // pin 5
	//TRISA4 = Output ;  // pin 6 (is open drain output, so a 0 activates)
	//TRISA5 = Output ;  // pin 7
						//	port A6 and A7 pins are not on this chip.
	#define opRelay1 	RA0
	#define opRelay2 	RA1
	#define opRelay3 	RA2
	#define opRelay4 	RA3
	#define opLEDerror	RA4
	#define opPower		RA5

//----------------------------------
// Port B directions and defines

	TRISB = Input; // majority are input
	#define SwitchPort PORTB

	//TRISB0 = Input ; // pin 33
	//TRISB1 = Input ; // pin 34
	//TRISB2 = Input ;  // pin 35
	//TRISB3 = Input ;  // pin 36
	TRISB4 = Output ; // pin 37  - yellow diagnostic LED - 1 activates
	TRISB5 = Output ; // pin 38 - LED trains 2 / 3
	//TRISB6 = Input ; // pin 39 - (PGC Clock when ICSP) 
	//TRISB7 = Input ; // pin 40 - (PGD Data when ICSP) 

	#define opLEDdiag 	RB4
	#define opLEDtrains	RB5

	#define bSwUp	0x40
	#define bSwDown	0x80
	#define bSwHex	0x0F
	#define NormalOps  0x0A
	#define SpecialOps 0x0F

	nRBPU   = 0 ; // PortB pullup resistors are on. !!! v9.8 change!

//-----------------------------------
// Port C directions and defines

	TRISC = Output;
	//TRISC0 = Output ; // pin 15
	//TRISC1 = Output ; // pin 16
	//TRISC2 = Output ; // pin 17
	//TRISC3 = Output ; // pin 18
	//TRISC4 = Output ; // pin 23
	//TRISC5 = Output ; // pin 24
	TRISC6 = Input ; // pin 25 - USART Tx
	TRISC7 = Input ; // pin 26 - USART Rx 

	#define opDirection RC0
	#define opSig21		RC1
	#define opSig41		RC2
	#define opSig51		RC3
	#define opSig52		RC4
	#define opBuzzer 	RC5

	#define DirLtoR	1
	#define DirRtoL	0

	#define Signal21 1
	#define Signal41 2
	#define Signal51 3
	#define Signal52 4
	#define danger 0
	#define clear  1

//---------------------------------
// Port D directions and defines 

	TRISD = Input;
	//TRISD0 = Input ; // pin 19
	//TRISD1 = Input ; // pin 20
	//TRISD2 = Input ; // pin 21
	//TRISD3 = Input ; // pin 22
	//TRISD4 = Input ; // pin 27
	//TRISD5 = Input ; // pin 28
	//TRISD6 = Input ; // pin 29 
	//TRISD7 = Input ; // pin 30 

	#define IRPort PORTD
	#define IR11 0x01
	#define IR21 0x02
	#define IR22 0x04
	#define IR31 0x08
	#define IR41 0x10
	#define IR42 0x20
	#define IR51 0x40
	#define IR52 0x80 //#define nilIR 0

//-----------------------------------
// Port E directions and defines

	#define SpeedPort PORTE

	TRISE0 = Output; // pin 8  - Speed bit 1
	TRISE1 = Output; // pin 9  - Speed bit 2
	TRISE2 = Input;  // pin 10 - analog Bat mon

	#define STOP	0
	#define SLOW	1
	#define MEDIUM	2
	#define FAST	2 
// was 3

//--------------------------------
// setup Timer2 as 5 mS clock

	T2CON = 0x0F; // TMR2ON, prescalar 1:16, postscaler = 1:2 = 0.032 mS

	PR2 = 156;  // 156 times 0.032 mS = 4.992 mS
	TMR2IE = 1; // Timer2 interrupt enable

//-------------------------------------------
// setup USART for async comms from RS232

	BRGH = 1;   // Baud rate for USART comms - 9600 bps
	SPBRG = 25; // see tables on data sheet chapter 10.1
	SYNC = 0;   // USART in async mode
	SPEN = 1;   // serial port enable
	TXEN = 1;   // enable transmission
	CREN = 1;   // enable reception
	RCIE = 1;   // USART Receive interrupt enable

//------------------------------
// set interrupt tree enables

	PEIE = 1; // peripheral interrupt enable
	ei();     // global interrupt enable (macro from include pic.h )

}// end of device_init()

//////////////////////////////
void 	Buzzer( byte Dir )
{//----------------------------

	opBuzzer = Dir; // set FET to switch buzzer

}// end of Buzzer()

///////////////////////////////////
void	DiagnosticLED( byte Dir )
{//--------------------------------

	opLEDdiag = Dir;

}// end of DiagnosticLED()

////////////////////////////
void	delay150( void )
{//--------------------------

	__delay_ms(150);	

}//end of delay150()

////////////////////////////////////
void	Beep( byte beepCount )
{//----------------------------------

	byte countBeeps = 0 ;
	while( countBeeps < beepCount )
	{
		Buzzer( on );
		delay150();
		Buzzer( off );
		delay150();
		countBeeps++ ;
	}
	delay150();	delay150();

}// end of Beep()

///////////////////////////////////////
void	InformCtrPIC( char character )
{//------------------------------------

	TXREG = character;

}// end of InformCtrPIC()

//////////////////////////////////////////
void	TestBatteryVolts( void )
{//---------------------------------------

	GO_nDONE = 1; // conversion start
	while( GO_nDONE ) continue; // loop until ready 	
	if( ADRESH < 190 ) // 11.7v	
	{
		InformCtrPIC( 'B' );
		Beep( 6 );

		SPEN = 0;	// serial port off
		ADCON0 = 0;	// ADC off
		T2CON = 0;	// Timer2 off
		di(); 	// no interrupt wakeup, only a reboot gets out of this
		TRISA = Input; // kill all outputs
		TRISB = Input;
		TRISC = Input;
		TRISD = Input;
		TRISE = Input;
		SLEEP(); // now chip current drain under 0.05 uA
	}
}// end of TestBatteryVolts()

///////////////////////////////////
void	ResetClocks( void )
{//---------------------------------

	Clock5mS  = 0;
	Clock1Sec = 0; // start at 0 & is incremented by ISR timer

}// end of ResetClocks()

////////////////////////////////////////////
void	PauseSeconds( byte SecondsCount )
{//------------------------------------------

	ResetClocks();
	do{ ; }while( Clock1Sec < SecondsCount );

}// end of PauseSeconds()

///////////////////////////////////
byte	GetPushButton( void )
{//---------------------------------

	switch( ~SwitchPort & ( bSwUp | bSwDown ) )
	{
		case bSwDown : return 0; // for down/clr button pressed
		case bSwUp   : return 1; // up/set button pressed
		default 	 : return 2; // for no buton pressed
	}

}// end of GetPushButton()

///////////////////////////////////
byte	GetHexSwitch( void )
{//---------------------------------

	return SwitchPort & bSwHex; // isolate just the hex sw

}// end of GetHexSwitch()

//////////////////////////////////////////////
void	SectionRelay( byte Value, byte Dir )
{//--------------------------------------------

	if( Dir == 1 )
	{	PortAimage = PortAimage | 1 << --Value;	}
	else
	{	PortAimage = PortAimage & ~(1 << --Value );	}
	PORTA = PortAimage;

}// end of SectionRelay()

//////////////////////////////////
void 	LayoutPower( byte Dir ) 
{//--------------------------------

	if( Dir == 1 )
	{	PortAimage = PortAimage | 0x20 ;	}
	else
	{	PortAimage = PortAimage & ~0x20 ;	}
	PORTA = PortAimage ;
	PauseSeconds( 2 ); // let every thing settle

}// end of LayoutPower()

//////////////////////////////////
void	ErrorLED( byte Dir )
{//--------------------------------

	if( Dir == 0 ) // the error LED is active low
	{	PortAimage = PortAimage | 0x10;	 }
	else
	{	PortAimage = PortAimage & ~0x10; }
	PORTA = PortAimage;

}// end of ErrorLED()

//////////////////////////////////////////
void 	Direction( byte Dir )
{//----------------------------------------

	opDirection = Dir;

}// end of SetDirection

//////////////////////////////////
void	Speed( byte value )
{//--------------------------------

	SpeedVal = value;
	SpeedPort = value;

}// end of Speed()

///////////////////////////////////////
void 	StepSpeed( byte Dir )
{//-------------------------------------

	if( Dir == 1 )
	{ if( SpeedVal < 3 ) SpeedVal++ ; else Beep( 1 ); }
	else
	{ if( SpeedVal > 0 ) SpeedVal-- ; else Beep( 1 ); }
	SpeedPort = SpeedVal;

}// end of StepSpeed()

/////////////////////////////////////////////
void 	Signal( byte SignalCode, byte Dir )
{//-------------------------------------------
	
	switch( SignalCode )
     	{	case Signal21: opSig21 = Dir; break;
			case Signal41: opSig41 = Dir; break;
			case Signal51: opSig51 = Dir; break;
			case Signal52: opSig52 = Dir; break;
		}
}// end of Signal()

/////////////////////////////////////
void	GetNumberOfTrains( void )
{//----------------------------------

	NumberOfTrains = eeprom_read( 0 ) ;
	switch( NumberOfTrains )
	{	case '2' : opLEDtrains = 0 ; break;
		case '3' : opLEDtrains = 1 ; break;
	}

}// end of GetNumberOfTrains()

////////////////////////////////////////
void 	StoreNumberOfTrains( byte Dir )
{//-------------------------------------

	if( Dir == 1 )
		eeprom_write( 0,  '3' );
	else // Dir == 0
		eeprom_write( 0,  '2' );
	GetNumberOfTrains();

}// end of StoreNumberOfTrains()

//////////////////////////
void ResetAll( void )
{//------------------------

	PortAimage = 0x10 ; // alarm LED off
	PORTA = PortAimage;
	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	Speed( STOP ); // PORTE
	GetNumberOfTrains();

}// end of ResetAll()

///////////////////////////////////
void	program_init( void )
{//---------------------------------

	ResetAll();
	LayoutPower( on );	// turn on layout power
	InformCtrPIC( '>' ); // send startup character
	PauseSeconds( 2 );

}// end of program_init()

////////////////////////////////////////
void	FatalError( byte BeepCount )
{//--------------------------------------

	while( true ) // only a reboot can exit this loop.
	{
		Speed( STOP );
		ErrorLED( on );
		InformCtrPIC( 'X' ); // send error character
		if( GetHexSwitch() != 7 ) // Silence?
		{
			Beep( BeepCount );
		}
	}
}// end of FatalError()

///////////////////////////////////////////
void	WaitForIR( byte IRportValue )
{ //---------------------------------------------------------

	ResetClocks(); // they start at 0 & incremented by ISR timer
	do{
		if( Clock1Sec > 60 ) // is it a timeout?
			FatalError( 3 ); // needs a reboot
	}
	while( ( IRPort & IRmask ) != IRportValue );
} // end of WaitForIR()

////////////////////////////////////////
void	RequestGo( char CommsStepChar )
{ //------------------------------------
  // if not in Special Ops mode, interact with the public panel.
  // Re-written 22-Jul-12 as it was a mess! Further edit 28-Jul-12

	DiagnosticLED( on ); // yellow LED on SeqPIC module
	ResetClocks();
	byte AcknFlag = true;
	byte NoAcknCount = 0;
	CommsRxChar = '~';

	while( 1 )	//forever while CommsRxChar is not 'X', '>' or 'G' or timeout on 'A'
	{			// There is no timeout for a 'G'
		if( GetHexSwitch() == SpecialOps )
		{
			DiagnosticLED( off );
			return; // exit the loop
		}
		if( Clock1Sec >= 1 ) // once per second...
		{
			ResetClocks(); // Clock1Sec reset... is incr by ISR
			if( AcknFlag ){
				AcknFlag = false;
				NoAcknCount = 0;
			}else{
				if( NoAcknCount >= 60 ) FatalError( 4 );
				DiagnosticLED( off );
				Beep(1);
				DiagnosticLED( on );
				NoAcknCount++ ; 
			}
			InformCtrPIC( CommsStepChar ); // send step number
		}
		switch( CommsRxChar ) // ISR sets CommsRxChar
		{
			case  'A':	AcknFlag = true;
						CommsRxChar = '~';
			case  '~':	break;
			case  '>':	Beep(4);
						// CtrPIC Power-up treat as GO, assume panels are out of sync
			case  'G':	DiagnosticLED( off );
						return;		// have GO, so exit RequestGo()
			case  'X':	FatalError( 5 ); // this call never returns.
		  	default  :	CommsRxChar = '~';
						Beep(1);// none of the above chars
						Beep(3);
		}
	}
} // end of RequestGo()

/////////////////////////////////
void	MoveNumber1( void )
{ //-------------------------------
  // S2 to S3 via main line - CtrPIC arrival - entry is IR21 & ~IR31

	IRmask =( IR51 | IR52 | IR31 ); // narrow the view of IR sensors
	SectionRelay( 2, on );	// turn on platform 2 section
	SectionRelay( 3, on );	// turn on platform 3 section
	RequestGo( '4' );		// inform public panel & wait for Go
	Direction( DirLtoR );	// set turnouts
	RequestGo( '1' );		// inform public panel & wait for Go
	Signal( Signal52, clear );
	PauseSeconds( 1 );
	Signal( Signal21, clear );
	PauseSeconds( 1 );
	Speed( SLOW );
	WaitForIR( IR51 ); // wait for LH home sensor
	Speed( MEDIUM );
	WaitForIR( IR52 ); // wait for RH home sensor
	Signal( Signal21, danger );
	Speed( SLOW );
	SectionRelay( 2, off );
	WaitForIR( IR31 ); // wait for platform end sensor
	Speed( STOP );
	PauseSeconds( 2 );
	Signal( Signal52, danger ); 
	SectionRelay( 3, off );
} // end of MoveNumber1()

/////////////////////////////////
void	MoveNumber3( void )
{ //-------------------------------
  // S4 to S1 via main line - CtrPIC departure - entry is IR41 & ~IR11

	IRmask =( IR51 | IR52 | IR11 ); // narrow the view of IR sensors
	SectionRelay( 1, on );
	SectionRelay( 4, on );
	RequestGo( '2' );		// inform public panel & wait for Go
	Direction( DirRtoL );	// set turnouts
	RequestGo( '3' );		// inform public panel & wait for Go
	Signal( Signal41, clear );
	PauseSeconds( 8 );
	Speed( SLOW );
	WaitForIR( IR52 );  // wait for RH home sensor
	Speed( MEDIUM );
	Signal( Signal51, clear );
	WaitForIR( IR51 );  // wait for LH home sensor
	Signal( Signal41, danger );
	Speed( SLOW );
	SectionRelay( 4, off );
	WaitForIR( IR11 );  // wait for platform end sensor
	Speed( STOP );
	PauseSeconds( 2 );
	Signal( Signal51, danger );
	SectionRelay( 1, off );
} // end of MoveNumber3()

//////////////////////////////////////
void	MoveNumber2( void )
{ //------------------------------------
  // section 1 to section 2 Sequence preparation move - goes around left hand loop

	IRmask =( IR21 | IR22 ); // narrow the view of IR sensors
	SectionRelay( 1, on );
	SectionRelay( 2, on );
	PauseSeconds( 1 );
	Speed( MEDIUM );	// was SLOW but needed more to start on curve
	WaitForIR( IR22 ); // wait for end of loop back curve
	Speed( SLOW );
	SectionRelay( 1, off );
	WaitForIR( IR21 ); // wait for platform end sensor
	Speed( STOP );
	PauseSeconds( 2 );
	SectionRelay( 2, off );
} // end of MoveNumber2()

///////////////////////////////////////
void	MoveNumber4( void )
{ //------------------------------------
  // section 3 to section 4 Sequence preparation move - goes around right hand loop

	IRmask =( IR41 | IR42 ); // narrow the view of IR sensors
	SectionRelay( 3, on );
	SectionRelay( 4, on );
	Speed( MEDIUM );	// was SLOW but needed more to start on curve
	WaitForIR( IR42 ); // wait for end of loop back curve
	Speed( SLOW );
	SectionRelay( 3, off );
	WaitForIR( IR41 ); // wait for platform end sensor
	Speed( STOP );
	PauseSeconds( 2 );
	SectionRelay( 4, off );
} // end of MoveNumber4()

//////////////////////////////////////////////
byte	GetMoveNumberFromPositions( void )
{ //------------------------------------------

	if( NumberOfTrains == '2' )
		switch( IRPort )
	 	{
			case IR21 | IR31 : return 4;	
			case IR31 | IR41 : return 3;
			case IR11 | IR31 : return 2;
			case IR21 | IR41 : return 1;
			case IR11 | IR21 : return 1; // R.Score req this add
			case IR11 | IR41 : return 2; // R.Score req this add
		}
	else // NumberOfTrains is '3'
	 	switch( IRPort )
	 	{
			case IR11 | IR21 | IR31 : return 4;	
			case IR21 | IR31 | IR41 : return 3;	
			case IR11 | IR31 | IR41 : return 2;	
			case IR11 | IR21 | IR41 : return 1;	
	 	}
	return 0; // none of the above - trains not in right places.
} // end of GetMoveNumberFromPositions()

//////////////////////////////////////////////////////////
void 	TrainDispatch( void )
{ //------------------------------------------------------

	if( GetMoveNumberFromPositions() != 0 ) // dispach if trains are in position
	{
		ResetAll();
		LayoutPower( on );	// turn on layout power

		while( GetHexSwitch() == NormalOps || GetHexSwitch() == SpecialOps )
		{
			TestBatteryVolts();
			switch( GetMoveNumberFromPositions() )
			{
				case 1 : MoveNumber1();   break;
				case 2 : MoveNumber2();   break;
				case 3 : MoveNumber3();   break;
				case 4 : MoveNumber4();   break;
				default: FatalError( 2 ); break;
			}
			Beep(1); // diagnostic beep
		}
	}
} // end of TrainDispatch()

/////////////////////////////////////////////////////////////////
void	main( void )
{ //-------------------------------------------------------------

	byte	PushButton;
	byte	LastPushButton;
	byte	HexCode;

	device_init();
	program_init();

	switch( GetHexSwitch() )
	{  // if trains are not in position then TrainDispatch returns
		 case NormalOps: case SpecialOps: TrainDispatch(); break;
	}
	while( 1 ) // manual control mode...
    {
		TestBatteryVolts();
	    ErrorLED( off );
		if( GetMoveNumberFromPositions() == 0 ) // trains not in position?
		{
			ErrorLED( on ); 
			if( Silence == 0 ){ Beep( 1 ); }
		}
		else if( SpeedVal > 0 )
		{
			Speed( STOP ); // trains in position so stop move in-progress
			Beep( 1 ); // once
		}
 	  	PushButton = GetPushButton();
	   	if( PushButton != LastPushButton ) // has button press changed?
		{
	   	    if( PushButton != 2 ) // Up or Down is pressed...
		    {
    	 		HexCode = GetHexSwitch();
     			switch( HexCode )
     			{
				  case 0x00: ResetAll(); 						break;
				  case 0x05: Direction( PushButton );			break;
				  case 0x06: StepSpeed( PushButton ); 			break;
				  case 0x07: Silence = PushButton; 				break;
				  case 0x08: StoreNumberOfTrains( PushButton ); break;
				  case 0x09: LayoutPower( PushButton ); 		break;
				  case 0x0A: case 0x0F : // aka NormalOps & SpecialOps
									TrainDispatch(); Beep(1);	break; 
				  case 0x0B: case 0x0C: case 0x0D: case 0x0E:
						Signal( HexCode - 0x0A, PushButton );	break;
                       // switch values left are 1, 2, 3 or 4
				  default: SectionRelay( HexCode, PushButton );
				}
		    }
			LastPushButton = PushButton;
		}
		__delay_ms( 10 );
	}
} // end of main()

//////////////////////////////////////////////////////////////
void interrupt	InterruptServiceRoutine( void )
{ //----------------------------------------------------------

	static byte TMR2count;

	if( RCIF ) // Receive character on USART from RS232
	{
		CommsRxChar = RCREG; // save the character
		if( CommsRxChar == '.')
		{
			TXREG = ':'; // diagnostic feature
			CommsRxChar = '~';
		}
	}
	if( TMR2IF ) // Timer2 finished its count, every 5 mS
	{
		TMR2IF = 0; // reset the interrupt flag
		Clock5mS++ ;
		TMR2count++ ;
		if( TMR2count > 200 ) // 200 * 5 mS = 1 second
		{
			TMR2count = 0;
			Clock1Sec++ ;
		}
	}
} // end of InterruptServiceRoutine()

/*
--------------------------------------
 SeqPIC Beep Codes
...............................
Condition                Beeps
-----------------------  -----
call your attention        1 (a single beep)
Trains start postions bad  2 beeps pause 2 beeps pause ...
Timeout moving Trains      3 beeps pause 3 ...
Timeout for CtrPIC reply   4 ...
Bad character from CtrPIC  1,3
Error code sent by CtrPIC  5
Low Battery shutdown       6 (shutdown so no repeat)

--------------------
    end of file
--------------------
*/
