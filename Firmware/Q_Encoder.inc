;====================================================================================================
;
;    Filename:      Q_Encoder.inc
;    Date:          1/22/2014
;    File Version:  1.0d1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;====================================================================================================
;    Quadrature Encoder Interface.
;
;    History:
;
; 1.0d1  01/22/2014	First Code
;
;====================================================================================================
;
;   Pin 1 (RA2/AN2) 
;   Pin 2 (RA3/AN3) 
;   Pin 3 (RA4/AN4) System LED (Active Low)
;   Pin 4 (RA5/MCLR*) 
;   Pin 5 (GND) Ground
;   Pin 6 (RB0/INT) Encoder Index <<<<<<<<<<<<
;   Pin 7 (RB1/AN11/SDA1) I2C Data
;   Pin 8 (RB2/AN10/RX) Master:RX Receive Data or Slave:NC
;   Pin 9 (RB3/CCP1) 
;
;   Pin 10 (RB4/AN8/SLC1) I2C Clock
;   Pin 11 (RB5/AN7) Master:TX Transmit Data 
;   Pin 12 (RB6/AN5/CCP2) Encoder Channel-A <<<<<<<<<<<<<<
;   Pin 13 (RB7/AN6) Encoder Channel-B <<<<<<<<<<<<<
;   Pin 14 (Vcc) +5 volts
;   Pin 15 (RA6) 
;   Pin 16 (RA7) 
;   Pin 17 (RA0) 
;   Pin 18 (RA1) 
;
;====================================================================================================
; Memory used in Bank7 because IOC stuff is in bank 7
;
;BitsTemp	RES	1	;Copy of Q_Enc_Port data
;EdgeFlags	RES	1	;Copy of IOCBF
;QEnc_Counter	RES	4	;Used only by ISR
;QEnc_Position	RES	4	;Updated by ISR if QEnc_Semaphore is clear
;
;QEnc_Flags	RES	1
;#Define	QEnc_X1	QEnc_Flags,0	;Set only for 1x
;#Define	QEnc_X2	QEnc_Flags,1	;Set only for 2x
;#Define	QEnc_X4	QEnc_Flags,2	;Set only for 4x
;#Define	QEnc_Z	QEnc_Flags,3	;Set by ISR when Index is seen
;#Define	QEnc_Counted	QEnc_Flags,4	;Set by ISR when inc or dec counter
;#Define	QEnc_Semaphore	QEnc_Flags,7
;
; PortB config
;  RB0,RB6,RB7 Inputs, Interrupt on change
;
#Define	Q_Enc_Port	PORTB
#Define	Q_Enc_Tris	TRISB
#Define	Q_Enc_Z_Bit	0
#Define	Q_Enc_A_Bit	6
#Define	Q_Enc_B_Bit	7
;
;Q_Encoder Initialization Code
;	BANKSEL	ANSELB
;	CLRF	ANSELB	;All Digital
;	BANKSEL	WPUB
;	BCF	WPUB,Q_Enc_Z_Bit	;No weak pull-up
;	BCF	WPUB,Q_Enc_A_Bit
;	BCF	WPUB,Q_Enc_B_Bit
;	BANKSEL	Q_Enc_Tris
;	BSF	Q_Enc_Tris,Q_Enc_Z_Bit	;Inputs
;	BSF	Q_Enc_Tris,Q_Enc_A_Bit
;	BSF	Q_Enc_Tris,Q_Enc_B_Bit
;	BANKSEL	IOCBP
;	BSF	QEnc_X1	;Set encoding x1,x2 or x4
;	BCF	QEnc_X2
;	BCF	QEnc_X4
;	BCF	QEnc_Semaphore
;	BSF	IOCBP,Q_Enc_A_Bit	;x1, Rising edge of channel-A
;	BTFSS	QEnc_X1
;	BSF	IOCBP,Q_Enc_B_Bit	;x2, Rising edge of channel-B
;	BSF	IOCBP,Q_Enc_Z_Bit	;Rising edge of channel-Z
;	BTFSC	QEnc_X4
;	BSF	IOCBN,Q_Enc_A_Bit	;x4, Falling edge of channel-A
;	BTFSC	QEnc_X4
;	BSF	IOCBN,Q_Enc_B_Bit	;x4, Falling edge of channel-B
;	BSF	IOCBN,Q_Enc_Z_Bit	;Falling edge of channel-Z
;	BSF	INTCON,IOCE
;
;====================================================================================================
;
ISR_Q_Enc	BTFSS	INTCON,IOCF	;Our interrupt?
	GOTO	ISR_Q_Enc_End	; No
	BANKSEL	Q_Enc_Port
	MOVF	Q_Enc_Port,W
	BANKSEL	IOCBF
	MOVWF	BitsTemp	;Save current state
	MOVLW	0xFF
	XORWF	IOCBF,W
	ANDWF	IOCBF,F	;Clear interrupt flag(s)
	XORLW	0xFF
	MOVWF	EdgeFlags	;Save edges detected
;
	BTFSS	QEnc_X1	;Clear? Clear if x2 or x4
	GOTO	ISR_Q_Enc_2or4	; Yes
; x1 Encoding
	BTFSS	EdgeFlags,Q_Enc_A_Bit	;Has Ch-A changed?
	GOTO	ISR_Q_Enc_NoCount	; No
	BTFSC	BitsTemp,Q_Enc_B_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;
; x2 Encoding
ISR_Q_Enc_2or4	BTFSS	QEnc_X2
	GOTO	ISR_Q_Enc_x4
;	
	BTFSS	EdgeFlags,Q_Enc_A_Bit	;Has Ch-A changed?
	GOTO	ISR_Q_Enc_x2NotA	; No	
	BTFSC	BitsTemp,Q_Enc_B_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;	
ISR_Q_Enc_x2NotA	BTFSS	EdgeFlags,Q_Enc_B_Bit	;Has Ch-B changed?
	GOTO	ISR_Q_Enc_NoCount	; No	
	BTFSS	BitsTemp,Q_Enc_A_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;
; x4 Encoding
ISR_Q_Enc_x4	BTFSS	EdgeFlags,Q_Enc_A_Bit	;Has Ch-A changed?
	GOTO	ISR_Q_Enc_x4NotA	; No	
	BTFSS	BitsTemp,Q_Enc_A_Bit	;Ch-A went high?
	GOTO	ISR_Q_Enc_x4_A_Fell	; No
	BTFSC	BitsTemp,Q_Enc_B_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;
ISR_Q_Enc_x4_A_Fell	BTFSS	BitsTemp,Q_Enc_B_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;	
ISR_Q_Enc_x4NotA	BTFSS	EdgeFlags,Q_Enc_B_Bit	;Has Ch-B changed?
	GOTO	ISR_Q_Enc_NoCount	; No
	BTFSS	BitsTemp,Q_Enc_B_Bit	;Ch-B went high?
	GOTO	ISR_Q_Enc_x4_B_Fell	; No
	BTFSS	BitsTemp,Q_Enc_A_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;
ISR_Q_Enc_x4_B_Fell	BTFSC	BitsTemp,Q_Enc_A_Bit
	GOTO	ISR_Q_Enc_Down
	GOTO	ISR_Q_Enc_Up
;
;
ISR_Q_Enc_Down	MOVLW	0x01
	SUBWF	QEnc_Counter,F
	CLRW
	SUBWFB	QEnc_Counter+2,F
	SUBWFB	QEnc_Counter+3,F
	SUBWFB	QEnc_Counter+4,F
	BSF	QEnc_Counted
	GOTO	ISR_Q_Enc_Update	
;
ISR_Q_Enc_Up	MOVLW	0x01
	ADDWF	QEnc_Counter,F
	CLRW
	ADDWFC	QEnc_Counter+1,F
	ADDWFC	QEnc_Counter+2,F
	ADDWFC	QEnc_Counter+3,F
	BSF	QEnc_Counted
;
ISR_Q_Enc_Update	BTFSC	QEnc_Semaphore	;OK to update counter?
	GOTO	ISR_Q_Enc_Index	; No
	MOVF	QEnc_Counter,W
	MOVWF	QEnc_Position
	MOVF	QEnc_Counter+1,W
	MOVWF	QEnc_Position+1
	MOVF	QEnc_Counter+2,W
	MOVWF	QEnc_Position+2
	MOVF	QEnc_Counter+3,W
	MOVWF	QEnc_Position+3
;
ISR_Q_Enc_NoCount
ISR_Q_Enc_Index	BTFSC	EdgeFlags,Q_Enc_Z_Bit	;Any Index edge was seen?
	BSF	QEnc_Z	; Yes
	BTFSC	BitsTemp,Q_Enc_Z_Bit	;Index is still High?
	BSF	QEnc_Z	; Yes
ISR_Q_Enc_End
;
;
;==================================================================================================
; Using the position data sample code
;
;	BANKSEL	QEnc_Flags
;	BSF	QEnc_Semaphore
;	BTFSS	QEnc_Semaphore	;Wait for Semaphore
;	GOTO	$-2
; Do stuff with the position data
;
;	BCF	QEnc_Semaphore
;
;==================================================================================================
