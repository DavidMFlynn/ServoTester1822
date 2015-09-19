;=========================================================================================
; 
;   Filename:	F1822_I2C_MasterTest.asm
;   Date:	1/24/2015
;   File Version:	1.0a1
;
;    Author:	David M. Flynn
;    Company:	Oxford V.U.E., Inc.
;    E-Mail:	dflynn@oxfordvue.com
;    Web Site:	http://www.oxfordvue.com/
;
;====================================================================================================
; Adapted from: AN734C
; PIC16F1937
; I2C MASTER DRIVER CODE
; Author: Chris Best
; Microchip Technologies
; DATE: 07/11/2013
;-----------------------------------------------------------------------------------------------
;
; CODE FUNCTION:
; The code implements the MSSP (or SSP) module as an I2C master.
; The master will transmit a set up data from an array to the slave, and after each byte
; is transmitted, that location in the array is overwritten with a value of 0xCC. This
; helps to make sure that the data was transmitted properly. After the data is transmitted,
; the master then reads data from an array in the slave, and loads the data into another
; array within the master.
; It is important to keep in mind that this code is for demonstration
; of the MSSP module for slave I2C communications. It does not include
; other interrupt possibilities, which would need to be added, and may require
; this code to be modified. The code is written to work directly
; with the 'I2C SLAVE CODE VERSION 2' and can be used either the assembly or C
; versions since they both do the same thing.
;==========================================================================================
;    RC Servo Testor for PIC12F1822 sample project
;
;    History:
;
; 1.0a1   1/24/2015	Copied from AN734 code, begin customized version
;
;==========================================================================================
; Options
;
;====================================================================================================
;====================================================================================================
; What happens next:
;
;   The system LED blinks once per second
;
;====================================================================================================
;
;  Pin 1 VDD (+5V)		+5V
;  Pin 2 RA5		n/c
;  Pin 3 RA4		System LED Active Low/Center switch Active Low
;  Pin 4 RA3/MCLR*/Vpp (Input only)	n/c
;  Pin 5 RA2		SDA
;  Pin 6 RA1/ICSPCLK		SCL
;  Pin 7 RA0/ICSPDAT		n/c
;  Pin 8 VSS (Ground)		Ground
;
;====================================================================================================
;
;
;
	list	p=12f1822,r=hex,w=0	; list directive to define processor
;
	nolist
	include	p12f1822.inc	; processor specific variable definitions
	list
;
	__CONFIG _CONFIG1,_FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF & _IESO_OFF
;
;
;
; INTOSC oscillator: I/O function on CLKIN pin
; WDT disabled
; PWRT disabled
; MCLR/VPP pin function is digital input
; Program memory code protection is disabled
; Data memory code protection is disabled
; Brown-out Reset enabled
; CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
; Internal/External Switchover mode is disabled
; Fail-Safe Clock Monitor is enabled
;
	__CONFIG _CONFIG2,_WRT_OFF & _PLLEN_OFF & _LVP_OFF
;
; Write protection off
; 4x PLL disabled
; Stack Overflow or Underflow will cause a Reset
; Brown-out Reset Voltage (Vbor), low trip point selected.
; Low-voltage programming Disabled ( allow MCLR to be digital in )
;  *** must set apply Vdd before Vpp in ICD 3 settings ***
;
; '__CONFIG' directive is used to embed configuration data within .asm file.
; The lables following the directive are located in the respective .inc file.
; See respective data sheet for additional information on configuration word.
;
	constant	oldCode=0
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
; 0.5uS res counter from 8MHz OSC
CCP1CON_Clr	EQU	b'00001001'	;Clear output on match
CCP1CON_Set	EQU	b'00001000'	;Set output on match
;
; I2C Commands
kCmdMoveToAlpha	EQU	0xA1
kCmdMoveToBeta	EQU	0xA2
kCmdMoveToCenter	EQU	0xA3
;
kOffsetCtrValue	EQU	d'2047'
kMinPulseWidth	EQU	d'1800'	;900uS
kMidPulseWidth	EQU	d'3000'	;1500uS
kMaxPulseWidth	EQU	d'4200'	;2100uS
kServoDwellTime	EQU	d'40000'	;20mS
;
#define	RX_ELEMENTS	.8	; number of allowable array elements
#define	WRITE_ADD	0x30	; slave write address
#define	READ_ADD	0x31	; slave read address
#define	BRG_VAL	0x27	; baud rate setting = 100kHz
#define	CLEAR	0xCC	; place in transmit array after transmit complete
#define	LOAD	0xAA	; value to load into array to transmit to slave

;====================================================================================================
	nolist
	include	F1822_Macros.inc
	list
;
;    Port A bits
PortADDRBits	EQU	b'11001111'
#Define	SystemLED	LATA,4	;Output: 0=LED ON, Input: 0 = Switch Pressed
#Define	SysLEDTrisBit	TRISA,4
#Define	CtrBtnBit	PORTA,4
#Define	AlphaBtnBit	PORTA,1
#Define	BetaBtnBit	PORTA,2
#Define	ModeBtnBit	PORTA,3
;
PortAValue	EQU	b'00000000'
;
;====================================================================================================
;====================================================================================================
;
;Constants
All_In	EQU	0xFF
All_Out	EQU	0x00
;
TMR0Val	EQU	0xB2	;0xB2=100Hz, 0.000128S/Count
LEDTIME	EQU	d'100'	;1.00 seconds
LEDErrorTime	EQU	d'10'
kWDTime	EQU	d'200'	;2 seconds
;
T1CON_Val	EQU	b'00000001'	;PreScale=1,Fosc/4,Timer ON
;
CCP1CON_Value	EQU	0x00	;CCP1 off
T2CON_Value	EQU	0x04	;Tmr 2 On, Presale 1:1, Postscale 1:1
;
DebounceTime	EQU	d'10'
;
;================================================================================================
;***** VARIABLE DEFINITIONS
; there are 128 bytes of ram, Bank0 0x20..0x7F, Bank1 0xA0..0xBF
; there are 256 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;================================================================================================
;  Bank0 Ram 020h-06Fh 80 Bytes
;
	cblock	0x20
;
	ISR_Temp		;scratch mem
	LED_Time	
	lastc		;part of tickcount timmer
	tickcount		;Timer tick count
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
	Timer1Lo		;1st 16 bit timer
	Timer1Hi		; one second RX timeiout
;
	Timer2Lo		;2nd 16 bit timer
	Timer2Hi		;
;
	Timer3Lo		;3rd 16 bit timer
	Timer3Hi		;GP wait timer
;
	Timer4Lo		;4th 16 bit timer
	Timer4Hi		; debounce timer
;
	SysFlags
;
	endc
;
;================================================================================================
;  Bank1 Ram 0A0h-0BFh 32 Bytes
;
I2C_Buffers	udata	0xA0
I2C_ARRAY_TX	RES	RX_ELEMENTS	; array to transmit to master
I2C_ARRAY_RX 	RES	RX_ELEMENTS 	; array to receive from master
INIT_MSTR_REC	res	1	; initializes reception
SET_RCEN	res	1	; initializes RCEN bit to receive data
REC_BYTE	res	1	; sets up receive byte sequence
READ_REC_BYTE	res	1	; read and store data
SET_ACKEN	res	1	; sets up acknowledge sequence
REC_COMPLETE	res	1	; set when reception is complete
COUNTERL	res	1	; used in delay routine
COUNTERH	res	1	; used in delay routine
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70	; set up in shared memory for easy access
	Flags
;Globals from I2C_SLAVE.inc
	INDEX_I2C		; index used to point to array location	
	GFlags
	GFlags2
	GFlags3
	Param75
	Param76
	Param77
	Param78
	Param79
	Param7A
	Param7B
	Param7C
	Param7D
	Param7E
	Param7F
	endc
;
; Flags bits
#Define	ServoOff	Flags,4
#Define	CtrBtnFlag	Flags,0
#Define	AlphaBtnFlag	Flags,1
#Define	BetaBtnFlag	Flags,2
#Define	DataChangedFlag	Flags,3
#Define	CtrBtnRawFlag	Flags,5
;
;
#Define	I2C_TXLocked	GFlags,0	; Set/cleared by ISR, data is being sent
#Define	I2C_RXLocked	GFlags,1	; Set/cleared by ISR, data is being received
#Define	I2C_NewRXData	GFlags,2	; Set by ISR, The new data is here!
#Define	I2C_INIT_START	GFlags,3	; used to set bit for start sequence
#Define	I2C_WRITE_TO_SLAVE	GFlags,4	; used for write sequence
#Define	I2C_STOP	GFlags,5	; used to set bit for stop sequence
#Define	I2C_TRANS_WRT_ADD	GFlags,6	; bit sets up write address transmission
#Define	I2C_INIT_TRANS_DATA	GFlags,7	; bit sets up data transmission
#Define	I2C_TRANS_DATA	GFlags2,0	; used to monitor transmission
#Define	I2C_TRANS_COMPLETE	GFlags2,1	; set when transmission is complete
#Define	I2C_TRANS_RD_ADD	GFlags2,2	; sets up read address transmission
#Define	I2C_READ_FROM_SLAVE	GFlags2,3	; used for read sequence
#Define	I2C_INIT_MSTR_REC	GFlags2,4	; initializes reception
#Define	I2C_SET_RCEN	GFlags2,5	; initializes RCEN bit to receive data
#Define	I2C_REC_BYTE	GFlags2,6	; sets up receive byte sequence
#Define	I2C_READ_REC_BYTE	GFlags2,7	; read and store data
#Define	I2C_SET_ACKEN	GFlags3,0	; sets up acknowledge sequence
#Define	I2C_REC_COMPLETE	GFlags3,1	; set when reception is complete
;
;=========================================================================================
;Conditionals
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10A2

;==============================================================================================
;============================================================================================
;
 	ORG    	0x0000	; reset vector
    	goto   	start	; jump to main
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
    	ORG    	0x0004	; interrupt vector
	CLRF	BSR	; bank0
;
;
	btfss	INTCON,T0IF
	goto	IRQ_2
;
	movlw	TMR0Val	;256x39+16 cycles (10,000uS)
	addwf	TMR0,F	; reload TMR0 with -40
	bcf	INTCON,T0IF	; reset interupt flag bit
;
;Decrement timers until they are zero
; 
	call	DecTimer1	;if timer 1 is not zero decrement
; 
	call	DecTimer2
; GP, Wait loop timer
	call	DecTimer3
; 
	call	DecTimer4
;
;-----------------------------------------------------------------
; blink LEDs
	BankSel	TRISA
	BSF	SysLEDTrisBit	;LED off
;
IRQ_2:
IRQ_4	MOVLB	0x00
	btfss 	PIR1,SSP1IF 	; Is this a SSP interrupt?
	goto 	IRQ_4_End 	; if not, bus collision int occurred
	btfsc	I2C_WRITE_TO_SLAVE	; is it a write sequence?
	goto	I2C_WRITE	; if so go here
	goto	I2C_READ	; if not, it is a read sequence
I2C_READ_Return:
I2C_WRITE_Return	movlb	0x00
	bcf 	PIR1,SSPIF	; clear the SSP interrupt flag
IRQ_4_End
;
;-----------------------------------------------------------------------------------------
; I2C Bus Collision
IRQ_5	MOVLB	0x00
	btfss	PIR2,BCLIF
	goto	IRQ_5_End
	movlb	0x04	;banksel SSPBUF						
	clrf	SSP1BUF	; clear the SSP buffer
	movlb	0x00	;banksel PIR2
	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag	

IRQ_5_End:
;
;==============================================================================================
;==============================================================================================
; This routine runs every 1/2 second. Unless an error is active then every 1/10th
;
ToggleSysLED	MOVF	LED_Time,W
	MOVWF	tickcount
	BankSel	TRISA
	BCF	SysLEDTrisBit	;LED on
	MOVLB	0
	RETURN
;
;==============================================================================================
;==============================================================================================
;
	include	I2C_MASTER.inc
;
;==============================================================================================
;**********************************************************************************************
;=========================================================================================
;
start	MOVLB	0x01	; select bank 1
	bsf	OPTION_REG,NOT_WPUEN	; disable pullups on port B
	bcf	OPTION_REG,TMR0CS	; TMR0 clock Fosc/4
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0	;111 8mhz/4/256=7812.5hz=128uS/Ct=0.032768S/ISR
	bsf	OPTION_REG,PS1	;101 8mhz/4/64=31250hz=32uS/Ct=0.008192S/ISR
	bsf	OPTION_REG,PS2
;
	MOVLB	0x01	; bank 1
	MOVLW	b'01110000'	;8MHz
	MOVWF	OSCCON
	movlw	b'00010111'	; WDT prescaler 1:65536 period is 2 sec (RESET value)
	movwf	WDTCON 	
;	
	MOVLB	0x01	; bank 3
	CLRF	ANSELA	;AN0 only
;
; setup timer 1 for 0.5uS/count
;
	BANKSEL	T1CON	; bank 0
	MOVLW	T1CON_Val
	MOVWF	T1CON
	bcf	T1GCON,TMR1GE
;
; clear memory to zero
	CALL	ClearRam
;
; setup ccp1
;
	BSF	ServoOff
	BANKSEL	APFCON
	BSF	APFCON,CCP1SEL	;RA5
	BANKSEL	CCP1CON
	CLRF	CCP1CON
;
	MOVLB	0x01	;Bank 1
	bsf	PIE1,CCP1IE
;
; setup data ports
	BANKSEL	PORTA
	MOVLW	PortAValue
	MOVWF	PORTA	;Init PORTA
	BANKSEL	LATA
	MOVWF	LATA	;Data Latch
	BANKSEL	ANSELA
	MOVLW	0x01
	MOVWF	ANSELA	;digital I/O except AN0
	BANKSEL	TRISA
	MOVLW	PortADDRBits
	MOVWF	TRISA
;
	MOVLB	0	;bank 0
	CLRWDT
;
	MOVLW	LEDTIME
	MOVWF	LED_Time
;
	MOVLW	0x01
	MOVWF	tickcount
;
	CLRWDT
	CALL	CopyToRam
	CLRWDT
	BTFSC	PositionAlphaH,7	;Test EEPROM Blank?
	CALL	InitEEData	; Yes
;
;
	bsf	INTCON,PEIE	; enable periferal interupts
	bsf	INTCON,T0IE	; enable TMR0 interupt
	bsf	INTCON,GIE	; enable interupts
;
	MOVLB	0x00	;bank 0
	BTFSC	ModeBtnBit	;Test/setup mode?
	BSF	I2C_IsActive	; Yes
;
	BTFSC	I2C_IsActive	;Test/setup mode?
	call	INITIALIZE_I2C	; No, set up uC
;

;
MainLoop	clrwdt		; clear WDT to prevent hangups
	call	Idle_I2C
;
	goto	MainLoop
;

	end		; END OF PROGRAM


