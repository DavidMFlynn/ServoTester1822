;==========================================================================
;
;    Filename:	TruckRepair.asm
;    Date:	10/18/2008                                  
;    File Version:	1.0d1
;
;    Author:	David M. Flynn
;    Company:	Oxford V.U.E., Inc.
;    E-Mail:	dflynn@oxfordvue.com
;
;==========================================================================
;    Truck Repair is a PIC based 1ms servo driver.
; 
; The truck repair bisiness at westend yard needs some animation.
;
;    History:
; 1.0d1    10/18/2008	Copied from ThreePosition
;  ThreePosition
; 1.0b1    5/30/2007	Changed to PIC16F628A for smoother servo operation.
; 1.0d3    8/26/2005	Fixed an interupt timing issue w/ bsf/bcf INTCON,GIE
; 1.0d2    8/8/2003	Moved pulse times to EEPROM.
; 1.0d1    7/28/2003	First Code.
;
;==========================================================================
; What happens next:
;
; Open and close bay doors
; Turn lights on and off.
;
;
;==========================================================================
; Notes:
;
; Pin 1, RA2  Bay 2 lights (Active high output)
; Pin 2, RA3  Bay 3 lights (Active high output)
; Pin 3, RA4  System LED (Active Low)
; Pin 4, RA5/MCLR* Dark (Active High Input)
; Pin 5, Gnd  Ground
; Pin 6, RB0  A0 (Active High Output)
; Pin 7, RB1  A1 (Active High Output)
; Pin 8, RB2  A2 (Active High Output)
; Pin 9, CCP1 Servo PWM signal output
; Pin 10, RB4 SW1/LED1 (Active Low Input/Output)
; Pin 11, RB5 SW2/LED2 (Active Low Input/Output)
; Pin 12, RB6 SW3/LED3 (Active Low Input/Output)
; Pin 13, RB7
; Pin 14, Vcc +5 volts
; Pin 15, RA6 Yard Lights (Active high output)
; Pin 16, RA7 Porch Light (Active high output)
; Pin 17, RA0 Office Light (Active high output)
; Pin 18, RA1 Bay 1 lights (Active high output)
;
;==========================================================================
;==========================================================================
;
	list      p=16f628A,r=hex,W=1	; list directive to define processor
;
	include	p16f628A.inc
;
	nolist
	include BMacros.asm
	list
;==========================================================================
;==========================================================================
;
;Constants for conditional assembly
;
	constant	oldCode=0
kDefaultFlags	EQU	b'00000100'	;nvLampOnHigh=1
;
T1CON_Val	EQU	b'00000001'	;PreScale=1 65.536mS
;T1CON_Val	EQU	b'00010101'	;PreScale=2
CCP1CON_Value	EQU	b'00001000'
; by the book
CCP1CON_Clr	EQU	b'00001001'	;Clear output on match
CCP1CON_Set	EQU	b'00001000'	;Set output on match
;
CCPR1_Val	EQU	d'1200'	;1.2mS
CCPR1_Val2	EQU	d'1500'	;1.5mS
kCCPR1_Val_Min	EQU	d'1000'
kCCPR1_Val_Mid	EQU	d'1500'
kCCPR1_Val_Max	EQU	d'2000'
ServoCycleTime	EQU	d'500'	;time between pulses
;
ML_DelayVal	EQU	0x04
DebounceTime	EQU	0x201
;	
	__CONFIG _CP_OFF & _WDT_OFF & _BOREN_OFF & _PWRTE_ON & _MCLRE_OFF & _LVP_OFF & _INTOSC_OSC_NOCLKOUT
;
;==============================================================================================
;
;    Port A bits
;
SystemLEDMask	EQU	0x01
SystemLEDPort	EQU	PORTA
#Define	LED_0	PORTA,0	;Office Light (Active high output)
#Define	LED_1	PORTA,1	;Bay 1 lights (Active high output)
#Define	LED_2	PORTA,2	;Bay 2 lights (Active high output)
#Define	LED_3	PORTA,3	;Bay 3 lights (Active high output)
#Define	SysLed	SystemLEDPort,4	;System LED (Active Low)
#Define	Dark_Sensor	PORTA,5	;Dark (Active High Input)
#Define	LED_4	PORTA,6	;Yard Lights (Active high output)
#Define	LED_5	PORTA,7	;Porch Light (Active high output)
;
PortADDRBits	EQU	b'00100000'	;Bit 5 in, others out
PortAValue	EQU	b'00000000'
;
;    Port B bits
;
#Define	ADDR_A0	PORTB,0	;A0 (Active High Output)
#Define	ADDR_A1	PORTB,1	;A1 (Active High Output)
#Define	ADDR_A2	PORTB,2	;A2 (Active High Output)
#Define	CCP1_OutBit	PORTB,3	;Output Active High
#Define	SW1Bit	PORTB,4	;SW1/LED1 (Active Low Input/Output)
#Define	SW2Bit	PORTB,5	;SW2/LED2 (Active Low Input/Output)
#Define	SW3Bit	PORTB,6	;SW3/LED3 (Active Low Input/Output)
; RB7 not used
PortBDDRBits	EQU	b'00000000'	;All Out
PortBValue	EQU	b'00000000'
;
;
;Constants
TMR0Val	EQU	0xD8
LEDTIME	EQU	d'100'
;
;***** VARIABLE DEFINITIONS
; there are 80 bytes of ram starting at 0x20
; there are 128 bytes of EEPROM starting at 0x00 the EEPROM is not mapped into memory but
;  accessed through the EEADR and EEDATA registers
;
;----------------------------------------------------------------------------------------
; Bank 0, 0x20..0x6F
;
	ORG	0x20   
status_temp	RES	1	; variable used for context saving
fsr_temp	RES	1	; variable used for context saving
fsr_swap	RES	1	; 2nd FSR reg
temp	RES	1	; temporary variable
temp2	RES	1	; used by SwapFSRs
EEAddrTemp	RES	1	;EEProm address to read or write
EEDataTemp	RES	1	;Data to be writen to EEProm
tickcount	RES	1	;Timer tick count
Timer1Lo	RES	1	;1st 16 bit timer
Timer1Hi	RES	1
Timer2Lo	RES	1	;2nd 16 bit timer
Timer2Hi	RES	1
Timer3Lo	RES	1
Timer3Hi	RES	1
;
CurrentServo	RES	1 	;0..7
; Servo timers if the command has not changed in the last 1 sec then skip this servo
ServoTmr0	RES	1
ServoTmr1	RES	1
ServoTmr2	RES	1
ServoTmr3	RES	1
ServoTmr4	RES	1
ServoTmr5	RES	1
ServoTmr6	RES	1
ServoTmr7	RES	1
; Output pulse width
ServoCurDC0	RES	2	;Current time constant
ServoCurDC1	RES	2	;Current time constant
ServoCurDC2	RES	2	;Current time constant
ServoCurDC3	RES	2	;Current time constant
ServoCurDC4	RES	2	;Current time constant
ServoCurDC5	RES	2	;Current time constant
ServoCurDC6	RES	2	;Current time constant
ServoCurDC7	RES	2	;Current time constant
; Destination pulse width
ServoDestDC0	RES	2	;Destination time constant
ServoDestDC1	RES	2	;Destination time constant
ServoDestDC2	RES	2	;Destination time constant
ServoDestDC3	RES	2	;Destination time constant
ServoDestDC4	RES	2	;Destination time constant
ServoDestDC5	RES	2	;Destination time constant
ServoDestDC6	RES	2	;Destination time constant
ServoDestDC7	RES	2	;Destination time constant
;
Servo0Flags	RES	1
Servo1Flags	RES	1
Servo2Flags	RES	1
Servo3Flags	RES	1
Servo4Flags	RES	1
Servo5Flags	RES	1
Servo6Flags	RES	1
Servo7Flags	RES	1
;
; Servo Flags bits
PulseDone	EQU	0
ServoOff	EQU	1
;
;----------------------------------------------------------------------
; 0x70..0x7F are common to all banks
Param70	EQU	0x70
Param71	EQU	0x71
;
Param78	EQU	0x78
Param79	EQU	0x79
Param7A	EQU	0x7A
Param7B	EQU	0x7B
Param7C	EQU	0x7C
Param7D	EQU	0x7D
w_temp	EQU	0x7E
;
;=================================================================================
; EEPROM locations (NV-RAM) 0x00..0x3F
;
	ORG	0x00
;
;==================================================================================
; ID Locations
	ORG	0x2000
	DE	'1','0','d','1'
;
;==================================================================================
; Default EEPROM data
	ORG	0x2100
;
;==================================================================================
;
; Reset enters here
;
	ORG	0x0000	; processor reset vector
  	goto	start	; go to beginning of program
;
;==================================================================================
;********************************* ISR ********************************************
;==================================================================================
; Interupts enter here
;
	ORG	0x0004	; interrupt vector location
	movwf	w_temp	; save current W register contents
	movf	STATUS,W	; move status register into W register
	clrf	STATUS
	movwf	status_temp	; save contents of STATUS register
	movf	FSR,W	; save contents of FSR register
	movwf	fsr_temp
;
	btfss	INTCON,T0IF
	goto	NoToggleSysLED
;
	movlw	TMR0Val	;256x39+16 cycles (10,000uS)
	addwf	TMR0,F	; reload TMR0 with -40
	bcf	INTCON,T0IF	; reset interupt flag bit
;
;Decrement timers 1,2 and 3 until they are zero
	call	TestT10	;test if timer 1 is zero
	btfss	STATUS,Z	;skip if zero
	call	DecTimer1	;was not zero
	call	TestT20
	btfss	STATUS,Z
	call	DecTimer2
	call	TestT30
	btfss	STATUS,Z
	call	DecTimer3
;
;Decrement Servo timers until they are zero
;	MOVF	ServoTmr0,F
;	SKPZ
;	DECF	ServoTmr0,F
;	MOVF	ServoTmr1,F
;	SKPZ
;	DECF	ServoTmr1,F
;	MOVF	ServoTmr2,F
;	SKPZ
;	DECF	ServoTmr2,F
;	MOVF	ServoTmr3,F
;	SKPZ
;	DECF	ServoTmr3,F
;	MOVF	ServoTmr4,F
;	SKPZ
;	DECF	ServoTmr4,F
;	MOVF	ServoTmr5,F
;	SKPZ
;	DECF	ServoTmr5,F
;	MOVF	ServoTmr6,F
;	SKPZ
;	DECF	ServoTmr6,F
;	MOVF	ServoTmr7,F
;	SKPZ
;	DECF	ServoTmr7,F
;
	BSF	SysLed	;LED Off
;
	DECF	tickcount,F
	BTFSS	STATUS,Z
	GOTO	NoToggleSysLED
;
ToggleSysLED	MOVLW	LEDTIME
	MOVWF	tickcount
	BCF	SysLed	;LED On
;
NoToggleSysLED
;
;-----------------------------------------------------------------
;Timer 1 has rolled over to 0x0000
;
;IRQ_2	GOTO	IRQ_2_End
;
;	btfss	PIR1,TMR1IF	;Timer 1 caused IRQ?
;	goto	IRQ_2_End	; No
;
;	bcf	PIR1,TMR1IF
;
;
;
;IRQ_2_End
;-----------------------------------------------------------------
; Handle CCP1 Interupt Flag
;
IRQ_Servo1	BTFSS	PIR1,CCP1IF
	GOTO	IRQ_Servo1_End
;
	BTFSC	CCP1CON,CCP1M0	;Set on match?
	GOTO	IRQ_Servo1_OL	; No
;
	BCF	_IRP
	MOVF	CurrentServo,W
	ADDLW	ServoCurDC0
	MOVWF	FSR
	MOVF	INDF,W	;Put the pulse into the CCP reg.
;	MOVLW	low d'1500'	;tc
	ADDWF	CCPR1L,F
	INCF	FSR,F
	MOVF	INDF,W
;	MOVLW	high d'1500'	;tc
	SKPNC
	ADDLW	0x01
	ADDWF	CCPR1H,F
	GOTO	IRQ_Servo1_1
;
IRQ_Servo1_OL	MOVLW	low ServoCycleTime	;Dead time between servo pulses
	ADDWF	CCPR1L,F
	MOVLW	high ServoCycleTime
	SKPNC
	ADDLW	0x01
	ADDWF	CCPR1H,F
;
	MOVF	CurrentServo,W
	ADDLW	Servo0Flags
	MOVWF	FSR
;
	bsf	INDF,PulseDone
;
	INCF	CurrentServo,W	;Next servo
	ANDLW	0x07
	MOVWF	CurrentServo
	MOVF	PORTB,W
	ANDLW	0xF0
	IORWF	CurrentServo,W
	MOVWF	PORTB
;
;
IRQ_Servo1_1	MOVLW	0x01
	XORWF	CCP1CON,F
	NOP
	NOP
	NOP
	BCF	PIR1,CCP1IF
;
IRQ_Servo1_End
;--------------------------------------------------------------------
;	
;
	movf	fsr_temp,W
	movwf	FSR
	movf	status_temp,W	; retrieve copy of STATUS register
	movwf	STATUS	; restore pre-isr STATUS register contents
	swapf	w_temp,F
	swapf	w_temp,W	; restore pre-isr W register contents
	retfie		; return from interrupt
;
;=========================================================================================
;*****************************************************************************************
;=========================================================================================
;
	if oldCode
;
StartServo	btfss	ServoOff
	RETURN
	bcf	ServoOff
	clrf	CCPR1H
	clrf	CCPR1L
	MOVLW	0xF0
	MOVWF	TMR1L
	MOVLW	0xFF
	MOVWF	TMR1H
	MOVLW	CCP1CON_Set
	MOVWF	CCP1CON	;go high on match
	RETURN
;
StopServo	btfsc	ServoOff
	RETURN
	bsf	ServoOff
	CLRF	CCP1CON
	RETURN
;
	endif
;=========================================================================================
;=========================================================================================
;
start	CLRF	STATUS
	CLRF	PCLATH
	bsf	_RP0	; select bank 1
	bcf	OPTION_REG,NOT_RBPU	; enable pullups on port B
	bcf	OPTION_REG,T0CS	; TMR0 clock CLKOUT
	bcf	OPTION_REG,PSA	; prescaler assigned to TMR0
	bsf	OPTION_REG,PS0
	bsf	OPTION_REG,PS1
	bsf	OPTION_REG,PS2
	bsf	INTCON,PEIE	; enable peripheral IRQs
	bsf	INTCON,T0IE	; enable TMR0 interupt
	bcf	_RP0	; select bank 0
;
; Initialize Ports
	movlw	PortBValue
	movwf	PORTB	;init port B
	movlw	PortAValue
	movwf	PORTA	;init port A
; Setup Comparitors
	movlw	0x07
	movwf	CMCON	;Digital I/O
; Set Port bits I or O
	bsf	_RP0	; select bank 1
	movlw	PortADDRBits
	movwf	TRISA
	movlw	PortBDDRBits	;setup for programer
	movwf	TRISB
	bcf	_RP0	; select bank 0
;
; clear memory to zero (Timer1Lo..0x7F)
	movlw	Timer1Lo
	movwf	FSR
RLoop	clrf	INDF
	incf	FSR,F
	btfss	FSR,7
	goto	RLoop
;
	movlw	T1CON_Val
	movwf	T1CON
;
	bsf	_RP0	; select bank 1
;	bsf	PIE1,TMR1IE
	bsf	PIE1,CCP1IE
	bcf	_RP0	; select bank 0
;
;setup CCP1
;
	CLRF	CCPR1H
	CLRF	CCPR1L
	CLRF	CCP1CON
	CLRF	CurrentServo
	MOVLW	0x08
	MOVWF	Param79	;Setup 8 servos to 1.5mS
	BCF	_IRP
;
; Set all servos to 1.5mS
RLoop2	MOVF	CurrentServo,W
	ADDLW	ServoCurDC0
	MOVWF	FSR
	MOVLW	low kCCPR1_Val_Mid
	MOVWF	INDF
	INCF	FSR,F
	MOVLW	high kCCPR1_Val_Mid
	MOVWF	INDF
;
	MOVF	CurrentServo,W
	ADDLW	ServoDestDC0
	MOVWF	FSR
	MOVLW	low kCCPR1_Val_Mid
	MOVWF	INDF
	INCF	FSR,F
	MOVLW	high kCCPR1_Val_Mid
	MOVWF	INDF
;
	INCF	CurrentServo,F
	DECFSZ	Param79,F
	GOTO	RLoop2
	CLRF	CurrentServo
;
; Start servo control
	MOVLW	0xF0
	MOVWF	TMR1L
	MOVLW	0xFF
	MOVWF	TMR1H
	MOVLW	CCP1CON_Set
	MOVWF	CCP1CON	;go high on match
;
	bsf	_GIE	; enable interupts
;==========================================================================
;  Main Loop
;
;==========================================================================
;
MainLoop	CLRWDT
;	MOVF	Timer1Lo,W
;	SKPZ
;	GOTO	ML_1
;	MOVLW	ML_DelayVal
;	MOVWF	Timer1Lo
;
	GOTO	MainLoop	;tc
;---------------------
; Adjust servo positions
;
	MOVLW	0x08
	MOVWF	Param79
	CLRF	Param78
;
AdjPos_L1	MOVF	Param78,W
	ADDLW	Servo0Flags
	MOVWF	FSR
	
	btfsc	INDF,ServoOff	;Don't change position while
	GOTO	AdjPos_Next	; the servo is off.
;
	btfss	INDF,PulseDone	;Don't change position until
	GOTO	AdjPos_Next	; last position is sent.
;
	BCF	INDF,PulseDone
;
	MOVLW	0x08
	SUBWF	FSR,F
	MOVF	INDF,W	;ServoDestDC
	MOVWF	Param7C
	INCF	FSR,F	;ServoDestDC+1
	MOVF	INDF,W
	MOVWF	Param7D
;
	MOVLW	0x09
	SUBWF	FSR,F	;ServoCurDC
	MOVF	Param7C,W	;ServoDestDC
	SUBWF	INDF,W	;W=ServoDestDC-ServoCurDC
	SKPZ		;DestDC=CurDC?
	GOTO	AdjPos	;No
;
	MOVF	Param7D,W	;ServoDestDC+1
	INCF	FSR,F
	SUBWF	INDF,W	;W=(ServoDestDC+1)-(ServoCurDC+1)
	SKPNZ		;ServoDestDC+1=ServoCurDC+1?
	GOTO	AdjPos_Next	;yes
;
; if DestDC < CurDC then CurDC--
AdjPos	MOVLW	ServoCurDC0+1
	ADDWF	Param78,W
	MOVWF	FSR
	MOVF	Param7D,W	;ServoDestDC+1
	SUBWF	INDF,W	;W=(CurDC+1)-(DestDC+1)
	SKPNB		;skip if not borrow
	GOTO	AdjPos_Up	;borrowed, (CurDC+1)<(DestDC+1)
;
	SKPZ		;(CurDC+1)=(DestDC+1)?
	GOTO	AdjPos_Down	;no, (CurDC+1)>(DestDC+1)
	DECF	FSR,F	;yes, test low byte
	MOVF	Param7C,W
	SUBWF	INDF,W	;W=(CurDC)-(DestDC)
	SKPNB		;skip if not borrow
	GOTO	AdjPos_Up	;borrowed, (CurDC)<(DestDC)
			;we already tested for equal so decriment
;
AdjPos_Down	MOVLW	ServoCurDC0
	ADDWF	Param78,W
	MOVWF	FSR
	bcf	_GIE	; interupts OFF
	DECF	INDF,F
	INCF	INDF,W	;0xFF+1=0
	SKPZ		;=0?
	GOTO	AdjPos_Down_1
	INCF	FSR,F
	DECF	INDF,F	;Yes
AdjPos_Down_1	bsf	_GIE	; interupts ON
	GOTO	AdjPos_Next
;
; else if DestDC > CurDC then CurDC++
AdjPos_Up	MOVLW	ServoCurDC0
	ADDWF	Param78,W
	MOVWF	FSR
	bcf	_GIE	; interupts OFF
	INCF	INDF,F
	SKPZ
	GOTO	AdjPos_Up_1
	INCF	FSR,F
	INCF	INDF,F
AdjPos_Up_1	bsf	_GIE	; interupts ON
;
AdjPos_Next	INCF	Param78,F
	DECFSZ	Param79,F
	GOTO	AdjPos_L1
;
;-------------------------------------------------------------------------
ML_1
;
MainEnd	goto	MainLoop
;
