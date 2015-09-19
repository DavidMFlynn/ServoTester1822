;====================================================================================================
;
;   Filename:	RCServoTester.asm
;   Date:	1/11/2015
;   File Version:	1.0a2
;
;    Author:	David M. Flynn
;    Company:	Oxford V.U.E., Inc.
;    E-Mail:	dflynn@oxfordvue.com
;    Web Site:	http://www.oxfordvue.com/
;
;====================================================================================================
;    RC Servo Testor for PIC12F1822 sample project
;
;    History:
;
; 1.0a3   1/15/2015	Added I2C slave
; 1.0a2   1/11/2015	Added Center button.
; 1.0a1   1/10/2015	First working tests.
; 1.0d1   9/4/2014	First Code.
;
;====================================================================================================
; Options
;
;====================================================================================================
;====================================================================================================
; What happens next:
;
;   The system LED blinks once per second
;   Once at power-up: read RA3
;    if RA3 is high (Btn not active) enable I2C comms mode
;    else enable setup mode and buttons
;
;  Setup mode:
;   If the Center button is active the servo goes to 1,500uS
;   else if ButtonAlpha goto PositionAlpha
;   else if ButtonBeta goto PositionBeta
;   else AN0 is read into memory
;     Pulse is calculated as AN0 * 4 - 2047 + 3000
;
;   To program Alpha or Beta positions:
;     Move servo to target position (no buttons pressed)
;     Press Alpha or Beta button and hold while clicking the Center button
;     Release all buttons
;
;  Communications Mode:
;
;
;
;   CCP1 outputs a 900uS to 2,100uS (1800..4200 counts) pulse every 16,000uS
;
;   Resolution is 0.5uS
;
;====================================================================================================
;
;  Pin 1 VDD (+5V)		+5V
;  Pin 2 RA5		CCP1
;  Pin 3 RA4		System LED Active Low/Center switch Active Low
;  Pin 4 RA3/MCLR*/Vpp (Input only)	Mode Btn, Setup Mode = Active Low
;  Pin 5 RA2		SDA or ButtonBeta Active Low
;  Pin 6 RA1/ICSPCLK		SCL or ButtonAlpha Active Low
;  Pin 7 RA0/ICSPDAT		AN0
;  Pin 8 VSS (Ground)		Ground
;
;====================================================================================================
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
	constant	useRS232=0
;
#Define	_C	STATUS,C
#Define	_Z	STATUS,Z
;
; 0.5uS res counter from 8MHz OSC
CCP1CON_Clr	EQU	b'00001001'	;Clear output on match
CCP1CON_Set	EQU	b'00001000'	;Set output on match
;
kOffsetCtrValue	EQU	d'2047'
kMinPulseWidth	EQU	d'1800'	;900uS
kMidPulseWidth	EQU	d'3000'	;1500uS
kMaxPulseWidth	EQU	d'4200'	;2100uS
kServoDwellTime	EQU	d'40000'	;20mS
;
; I2C Commands
kCmdMoveToAlpha	EQU	0xA1
kCmdMoveToBeta	EQU	0xA2
kCmdMoveToCenter	EQU	0xA3
;
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
;
	EEAddrTemp		;EEProm address to read or write
	EEDataTemp		;Data to be writen to EEProm
;
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
	PositionAlpha
	PositionAlphaH
	PositionBeta
	PositionBetaH
	SysFlags
;
; RS232 stuff
;	TXByte	
;	RXByte	
;	SerialFlags	
;
	endc
;
;#Define	DataSentFlag	SerialFlags,0
;#Define	DataReceivedFlag	SerialFlags,1
;
#Define	FirstRAMParam	PositionAlpha
#Define	LastRAMParam	SysFlags
;
#define	I2C_ADDRESS	0x30	; Slave address
#define	RX_ELEMENTS	.8	; number of allowable array elements, in this case 8
;
;================================================================================================
;  Bank1 Ram 0A0h-0BFh 32 Bytes
;
	if useRS232
Ser_Buff_Bank	EQU	0x01
	cblock	0xA0
	Ser_In_InPtr
	Ser_In_OutPtr
	Ser_Out_InPtr
	Ser_Out_OutPtr
	Ser_In_Bytes
	Ser_Out_Bytes
	Ser_In_Buff:8
	Ser_Out_Buff:8
	endc
	endif
;
I2C_Buffers	udata	0xA0
I2C_ARRAY_TX	RES	RX_ELEMENTS	; array to transmit to master
I2C_ARRAY_RX 	RES	RX_ELEMENTS 	; array to receive from master
;
;=======================================================================================================
;  Common Ram 70-7F same for all banks
;      except for ISR_W_Temp these are used for paramiter passing and temp vars
;=======================================================================================================
;
	cblock	0x70
	SigOutTime
	SigOutTimeH
	Flags
;Globals from I2C_SLAVE.inc
	INDEX_I2C
	GFlags
;
	CalcdDwell
	CalcdDwellH
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
#Define	I2C_TXLocked	GFlags,0	; Set/cleared by ISR, data is being sent
#Define	I2C_RXLocked	GFlags,1	; Set/cleared by ISR, data is being received
#Define	I2C_NewRXData	GFlags,2	; Set by ISR, The new data is here!
#Define	I2C_IsActive	GFlags,3	; Set at startup in setup button is open (RA3 is high)
;
;=========================================================================================
;Conditionals
HasISR	EQU	0x80	;used to enable interupts 0x80=true 0x00=false
;
;=========================================================================================
;==============================================================================================
; ID Locations
	__idlocs	0x10A2
;	__idlocs	'0'
;	__idlocs	'a'
;	__idlocs	'2'
;
;==============================================================================================
; EEPROM locations (NV-RAM) 0x00..0x7F (offsets)
;	org	0x2100
;
;nvPositionAlphaLo	de	LOW kMidPulseWidth
;nvPositionAlphaHi	de	HIGH kMidPulseWidth
;nvPositionBetaLo	de	LOW kMidPulseWidth
;nvPositionBetaHi	de	HIGH kMidPulseWidth
;
;nvSysFlags	de	0x00
;
	cblock	0x00
	nvPositionAlphaLo
	nvPositionAlphaHi
	nvPositionBetaLo
	nvPositionBetaHi
;
	nvSysFlags
	endc
;
#Define	nvFirstParamByte	nvPositionAlphaLo
#Define	nvLastParamByte	nvSysFlags
;
;
;==============================================================================================
;============================================================================================
;
;
	ORG	0x000	; processor reset vector
	CLRF	STATUS
	CLRF	PCLATH
  	goto	start	; go to beginning of program
;
;===============================================================================================
; Interupt Service Routine
;
; we loop through the interupt service routing every 0.008192 seconds
;
;
	ORG	0x004	; interrupt vector location
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
	BankSel	PORTA	;bank 0
	BCF	CtrBtnRawFlag
	BTFSS	CtrBtnBit
	BSF	CtrBtnRawFlag
	MOVF	Timer4Lo,F	;Debounce time done?
	SKPZ
	GOTO	BtnReTests
	BCF	CtrBtnFlag	;Yes
	BCF	AlphaBtnFlag
	BCF	BetaBtnFlag
;
	BTFSC	CtrBtnBit	;Switch Down?
	GOTO	AlphaBtnTest	; No
	BSF	CtrBtnFlag	; Yes
	GOTO	BtnIsDown
;
	BTFSC	I2C_IsActive	;Test/Setup mode?
	GOTO	SystemBlink_1
;
AlphaBtnTest	BTFSC	AlphaBtnBit	;Switch Down?
	GOTO	BetaBtnTest	; No
	BSF	AlphaBtnFlag	; Yes
	GOTO	BtnIsDown
;
BetaBtnTest	BTFSC	BetaBtnBit	;Switch Down?
	GOTO	SystemBlink_1	; No
	BSF	BetaBtnFlag	; Yes
	GOTO	BtnIsDown
;
BtnReTests	BTFSS	CtrBtnFlag
	GOTO	BtnReTests_1
	BTFSS	CtrBtnBit
	GOTO	BtnIsDown
BtnReTests_1	BTFSS	AlphaBtnFlag
	GOTO	BtnReTests_2
	BTFSS	AlphaBtnBit
	GOTO	BtnIsDown
BtnReTests_2	BTFSS	BetaBtnFlag
	GOTO	SystemBlink_1
	BTFSC	BetaBtnBit
	GOTO	SystemBlink_1
;	
BtnIsDown	MOVLW	DebounceTime
	MOVWF	Timer4Lo
;
SystemBlink_1	MOVF	tickcount,F
	SKPNZ
	GOTO	SystemBlinkDone
	DECF	tickcount,F
	SKPNZ
	CALL	ToggleSysLED
	GOTO	SystemBlink_end
;
SystemBlinkDone	MOVF	LED_Time,F
	SKPZ
	CAll	ToggleSysLED
;
SystemBlink_end	
;
;-----------------------------------------------------------------
;
IRQ_2:
;==================================================================================
;
; Handle CCP1 Interupt Flag, Enter w/ bank 0 selected
;
IRQ_Servo1	MOVLB	0	;bank 0
	BTFSS	PIR1,CCP1IF
	GOTO	IRQ_Servo1_End
;
	BTFSS	ServoOff	;Are we sending a pulse?
	GOTO	IRQ_Servo1_1	; Yes
;
;Oops, how did we get here???
	MOVLB	0x05
	CLRF	CCP1CON
	GOTO	IRQ_Servo1_X
;
IRQ_Servo1_1	MOVLB	0x05
	BTFSC	CCP1CON,CCP1M0	;Set output on match?
	GOTO	IRQ_Servo1_OL	; No
; An output just went high
;
	MOVF	SigOutTime,W	;Put the pulse into the CCP reg.
;	MOVLW	LOW kMidPulseWidth	;tc
	ADDWF	CCPR1L,F
	MOVF	SigOutTime+1,W
;	MOVLW	HIGH kMidPulseWidth	;tc
	ADDWFC	CCPR1H,F
	MOVLW	CCP1CON_Clr	;Clear output on match
	MOVWF	CCP1CON	;CCP1 clr on match
;Calculate dwell time
	MOVLW	LOW kServoDwellTime
	MOVWF	CalcdDwell
	MOVLW	HIGH kServoDwellTime
	MOVWF	CalcdDwellH
	MOVF	SigOutTime,W
	SUBWF	CalcdDwell,F
	MOVF	SigOutTime+1,W
	SUBWFB	CalcdDwellH,F
	GOTO	IRQ_Servo1_X
;
; output went low so this cycle is done
IRQ_Servo1_OL	MOVLW	LOW kServoDwellTime
	ADDWF	CCPR1L,F
	MOVLW	HIGH kServoDwellTime
	ADDWFC	CCPR1H,F
;
	MOVLW	CCP1CON_Set	;Set output on match
	MOVWF	CCP1CON
;
IRQ_Servo1_X	MOVLB	0x00
	BCF	PIR1,CCP1IF
IRQ_Servo1_End:
;-----------------------------------------------------------------------------------------
; I2C Com
IRQ_4	MOVLB	0x00
	btfss 	PIR1,SSP1IF 	; Is this a SSP interrupt?
	goto 	IRQ_4_End 	; if not, bus collision int occurred
	banksel	SSP1STAT						
	btfsc	SSPSTAT,R_NOT_W	; is it a master read:
	goto	I2C_READ	; if so go here
	goto	I2C_WRITE	; if not, go here
I2C_READ_Return:
I2C_WRITE_Return	movlb	0x00
	bcf 	PIR1,SSP1IF	; clear the SSP interrupt flag
IRQ_4_End
;-----------------------------------------------------------------------------------------
; I2C Bus Collision
IRQ_5	MOVLB	0x00
	btfss	PIR2,BCL1IF
	goto	IRQ_5_End
	banksel	SSPBUF						
	clrf	SSPBUF	; clear the SSP buffer
	movlb	0x00	;banksel PIR2
	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag	
	banksel	SSPCON1
	bsf	SSPCON1,CKP	; release clock stretch
	movlb	0x00
;
IRQ_5_End:
;
;--------------------------------------------------------------------
;
	retfie		; return from interrupt
;
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
	include	I2C_SLAVE.inc
;
;==============================================================================================
;**********************************************************************************************
;==============================================================================================
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
;=========================================================================================
;=========================================================================================
;  Main Loop
;
	CALL	StartServo
	BTFSS	I2C_IsActive	;Test/Setup mode?
	CALL	ReadAN0_ColdStart	; Yes
;=========================================================================================
;
MainLoop	CLRWDT
	BTFSC	I2C_IsActive	;Test/Setup mode?
	GOTO	I2C_DataInturp
	CALL	ReadAN0	; Yes
;
	MOVLB	0x00
	BTFSS	CtrBtnFlag
	GOTO	MainLoop_NotCtr
DoCenter	CALL	SetMiddlePosition
	GOTO	Move_It
;
MainLoop_NotCtr	BTFSS	AlphaBtnFlag
	GOTO	MainLoop_NotA
	BTFSS	CtrBtnRawFlag	;Both down Alpha>>Ctr
	GOTO	DoAlpha	; No
	MOVF	Param7C,W	; Yes, copy AN0 to Alpha
	MOVWF	PositionAlpha
	MOVF	Param7D,W
	MOVWF	PositionAlphaH
	BSF	DataChangedFlag
	GOTO	Move_It
;
DoAlpha	MOVF	PositionAlpha,W
	MOVWF	Param7C
	MOVF	PositionAlphaH,W
	MOVWF	Param7D
	CALL	ClampInt
	GOTO	Move_It	
;
MainLoop_NotA	BTFSS	BetaBtnFlag
	GOTO	MainLoop_NotB
	BTFSS	CtrBtnRawFlag	;Both down Alpha>>Ctr
	GOTO	DoBeta	; No
	MOVF	Param7C,W
	MOVWF	PositionBeta
	MOVF	Param7D,W
	MOVWF	PositionBetaH
	BSF	DataChangedFlag
	GOTO	Move_It
;
DoBeta	MOVF	PositionBeta,W
	MOVWF	Param7C
	MOVF	PositionBetaH,W
	MOVWF	Param7D
	CALL	ClampInt
	GOTO	Move_It	
;
MainLoop_NotB	BTFSC	DataChangedFlag
	CALL	SaveParams
	NOP		;tc
	BCF	DataChangedFlag
Move_It	CALL	Copy7CToSig
;
	goto	MainLoop
;
;==============================================================
;
I2C_DataInturp	BTFSC	I2C_RXLocked
	RETURN
	LOADFSR0	I2C_ARRAY_RX,0
	MOVF	INDF0,W
	MOVWF	Param78
;
	MOVLW	kCmdMoveToAlpha
	SUBWF	Param78,W
	SKPNZ
	GOTO	DoAlpha
;
	MOVLW	kCmdMoveToBeta
	SUBWF	Param78,W
	SKPNZ
	GOTO	DoBeta
;
	MOVLW	kCmdMoveToCenter
	SUBWF	Param78,W
	SKPNZ
	GOTO	DoCenter
;
	GOTO	MainLoop
;
;=========================================================================================
;=========================================================================================
; Initialize the EEData memory
;
InitEEData	MOVLW	LOW kMidPulseWidth
	MOVWF	PositionAlpha
	MOVWF	PositionBeta
	MOVLW	HIGH kMidPulseWidth
	MOVWF	PositionAlphaH
	MOVWF	PositionBetaH
	CLRF	SysFlags
	GOTO	SaveParams
;
;=========================================================================================
;=========================================================================================
; Setup or Read AN0
;
ReadAN0	MOVLB	1	;bank 1
	BTFSS	ADCON0,ADON	;Is the Analog input ON?
	GOTO	ReadAN0_ColdStart	; No, go start it
	BTFSC	ADCON0,NOT_DONE	;Conversion done?
	GOTO	ReadAN0_Rtn	; No
	MOVF	ADRESH,W
	MOVWF	Param7D
	MOVF	ADRESL,W
	MOVWF	Param7C
	BSF	ADCON0,ADGO	;Start next conversion.
	MOVLB	0
	LSLF	Param7C,F	;x2 = 0..2046
	RLF	Param7D,F
	LSLF	Param7C,F	;x2 = 0..4095
	RLF	Param7D,F
;
	MOVLW	low kOffsetCtrValue	;Subtract center
	SUBWF	Param7C,F
	MOVLW	high kOffsetCtrValue
	SUBWFB	Param7D,F
;
	MOVLW	low kMidPulseWidth
	ADDWF	Param7C,F
	MOVLW	high kMidPulseWidth
	ADDWFC	Param7D,F
	GOTO	ClampInt
;
; Don't disable interrupts if you don't need to...
Copy7CToSig	MOVF	Param7C,W
	SUBWF	SigOutTime,W
	SKPZ
	GOTO	ReadAN0_1
	MOVF	Param7D,W
	SUBWF	SigOutTimeH,W
	SKPNZ
	Return
;
ReadAN0_1	bcf	INTCON,GIE
	MOVF	Param7C,W
	MOVWF	SigOutTime
	MOVF	Param7D,W
	MOVWF	SigOutTimeH
	bsf	INTCON,GIE
;	
	RETURN
;
ReadAN0_ColdStart	MOVLB	1
	MOVLW	0xA0	;Right Just fosc/32
	MOVWF	ADCON1
	MOVLW	b'00000001'	;Select AN0
	MOVWF	ADCON0
	BSF	ADCON0,GO
ReadAN0_Rtn	MOVLB	0
	Return
;
;=========================================================================================
;=========================================================================================
; Set CCP1 to go high is 0x100 clocks
;
StartServo	MOVLB	0	;bank 0
	BTFSS	ServoOff
	RETURN
	BCF	ServoOff
;
	CALL	SetMiddlePosition
	CALL	Copy7CToSig
;
	MOVLW	0x00	;start in 0x100 clocks
	MOVWF	TMR1L
	MOVLW	0xFF
	MOVWF	TMR1H
;
	MOVLB	0x05
	CLRF	CCPR1H
	CLRF	CCPR1L
	MOVLW	CCP1CON_Set
	MOVWF	CCP1CON	;go high on match
	MOVLB	0x00	;Bank 0
	RETURN
;
; Don't disable interrupts if you don't need to...
SetMiddlePosition	MOVLW	LOW kMidPulseWidth
	MOVWF	Param7C
	MOVLW	HIGH kMidPulseWidth
	MOVWF	Param7D
	Return
;
;=========================================================================================
; ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
; Entry: Param7D:Param7C
; Exit: Param7D:Param7C=ClampInt(Param7D:Param7C,kMinPulseWidth,kMaxPulseWidth)
;
ClampInt	MOVLW	high kMaxPulseWidth
	SUBWF	Param7D,W	;7D-kMaxPulseWidth
	SKPNB		;7D<Max?
	GOTO	ClampInt_1	; Yes
	SKPZ		;7D=Max?
	GOTO	ClampInt_tooHigh	; No, its greater.
	MOVLW	low kMaxPulseWidth	; Yes, MSB was equal check LSB
	SUBWF	Param7C,W	;7C-kMaxPulseWidth
	SKPNZ		;=kMaxPulseWidth
	RETURN		;Yes
	SKPB		;7C<Max?
	GOTO	ClampInt_tooHigh	; No
	RETURN		; Yes
;
ClampInt_1	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,W	;7D-kMinPulseWidth
	SKPNB		;7D<Min?
	GOTO	ClampInt_tooLow	; Yes
	SKPZ		;=Min?
	RETURN		; No, 7D>kMinPulseWidth
	MOVLW	low kMinPulseWidth	; Yes, MSB is a match
	SUBWF	Param7C,W	;7C-kMinPulseWidth
	SKPB		;7C>=Min?
	RETURN		; Yes
;	
ClampInt_tooLow	MOVLW	low kMinPulseWidth
	MOVWF	Param7C
	MOVLW	high kMinPulseWidth
	MOVWF	Param7D
	RETURN
;
ClampInt_tooHigh	MOVLW	low kMaxPulseWidth
	MOVWF	Param7C
	MOVLW	high kMaxPulseWidth
	MOVWF	Param7D
	RETURN
;
	if oldCode
;=========================================================================================
;=====================================================================================
;
MoveTo78	MOVWF	FSR0L
	MOVF	INDF0,W
	MOVWF	Param78
	INCF	FSR0L,F
	MOVF	INDF0,W
	MOVWF	Param79
	RETURN
;
;=====================================================================================
;
MoveTo7C	MOVWF	FSR0L
	MOVF	INDF0,W
	MOVWF	Param7C
	INCF	FSR0L,F
	MOVF	INDF0,W
	MOVWF	Param7D
	RETURN
;
;=====================================================================================
;
Move78To7C	MOVF	Param78,W
	MOVWF	Param7C
	MOVF	Param79,W
	MOVWF	Param7D
	RETURN
;
;=====================================================================================
;
MoveFrom7C	MOVWF	FSR0L
	MOVF	Param7C,W
	MOVWF	INDF0
	INCF	FSR0L,F
	MOVF	Param7D,W
	MOVWF	INDF0
	RETURN
;
;=====================================================================================
; Less or Equal
;
; Entry: Param7D:Param7C, Param79:Param78
; Exit: Param77:0=Param7D:Param7C<=Param79:Param78
;
Param7D_LE_Param79	CLRF	Param77	;default to >
	MOVF	Param79,W
	SUBWF	Param7D,W	;Param7D-Param79
	SKPNB		;Param7D<Param79?
	GOTO	SetTrue	; Yes
	SKPZ		;Param7D>Param79?
	RETURN		; Yes
	MOVF	Param78,W	; No, MSB is a match
	SUBWF	Param7C,W	;Param7C-Param78
	SKPNB		;Param7C<Param78?
	GOTO	SetTrue	; Yes
	SKPZ		;LSBs then same?
	RETURN		; No
;
SetTrue	BSF	Param77,0
	RETURN
;
;=====================================================================================
; Greater or Equal
;
; Entry: Param7D:Param7C, Param79:Param78
; Exit: Param77:0=Param7D:Param7C>=Param79:Param78
;
Param7D_GE_Param79	CLRF	Param77	;default to <
	MOVF	Param79,W
	SUBWF	Param7D,W	;Param7D-Param79
	SKPNB		;Param7D<Param79?
	RETURN		; Yes
	SKPZ		;Param7D>Param79?
	GOTO	SetTrue	; Yes
Param7D_GE_Param79_1	MOVF	Param78,W	; No, MSB is a match
	SUBWF	Param7C,W	;Param7C-Param78
	SKPNB		;Param7C<Param78?
	RETURN		; Yes
	GOTO	SetTrue	; No
;
;======================================================================================
;
EqualMin	CLRF	Param77
	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,W
	SKPZ
	RETURN
	MOVLW	low kMinPulseWidth
	SUBWF	Param7C,W
	SKPNZ
	BSF	Param77,0
	RETURN
	
;
Subtract1000	MOVLW	low kMinPulseWidth
	SUBWF	Param7C,F
	SUBBF	Param7D,F
	MOVLW	high kMinPulseWidth
	SUBWF	Param7D,F
	RETURN
;
Subtract1500	MOVLW	low d'1500'
	SUBWF	Param7C,F
	SUBBF	Param7D,F
	MOVLW	high d'1500'
	SUBWF	Param7D,F
	RETURN
;
X2	CLRC
	RLF	Param7C,F
	RLF	Param7D,F
	RETURN
;
Add1000	MOVLW	low kMinPulseWidth
	ADDWF	Param7C,F
	ADDCF	Param7D,F
	MOVLW	high kMinPulseWidth
	ADDWF	Param7D,F
	RETURN
;
	endif
;=============================================================================================
;==============================================================================================
;
	include	F1822_Common.inc
;
;=========================================================================================
;=========================================================================================
;
;
;
;
	END
;
