;=========================================================================================
; PIC16F1822
; I2C MASTER DRIVER CODE
;
;    Filename:      I2C MASTER.inc
;    Date:          9/4/2014
;    File Version:  1.0.1
;
;    Author:        David M. Flynn
;    Company:       Oxford V.U.E., Inc.
;    E-Mail:        dflynn@oxfordvue.com
;    Web Site:      http://www.oxfordvue.com/
;
;=========================================================================================
;   I2C Master Driver Code for PIC16F1822
;
;    History:
;
; 1.0.1  9/4/2014	Modified for 12F1822
; 1.0   11/23/2013	Adapted from AN734C
;
; *** Adapted from AN734C ****
; PIC16F1937 
;I2C MASTER DRIVER CODE 
;Author: Chris Best 
;Microchip Technologies
;DATE: 07/03/2013
;
;=========================================================================================
; Options
;
;   I2C_ADDRESS and RX_ELEMENTS must be set and be the same in the slave driver.
;
;=========================================================================================
;=========================================================================================
; What happens next:
;  Sends or receives RX_ELEMENTS bytes of data from the slave
;
;=========================================================================================
;
#define	RX_ELEMENTS	.8	; number of allowable array elements
#define	WRITE_ADD	0x30	; slave write address
#define	READ_ADD	0x30	; slave read address
#define	BRG_VAL	0x27	; baud rate setting = 100kHz
#define	I2C_TX_Init_Val	0xAA	; value to load into transmit array to send to master
#define	I2C_RX_Init_Val	0xAA	; value to load into received data array
;
;
;------------------------------------ variables-------------------------------------------
;
;	cblock	0x70									; set up in shared memory for easy access
;	INDEX_I2C		; index used to point to array location	
;	GFlags
;	GFlags2
;	GFlags3
;	endc
;
;#Define	I2C_TXLocked	GFlags,0	; Set/cleared by ISR, data is being sent
;#Define	I2C_RXLocked	GFlags,1	; Set/cleared by ISR, data is being received
;#Define	I2C_NewRXData	GFlags,2	; Set by ISR, The new data is here!
;#Define	I2C_INIT_START	GFlags,3	; used to set bit for start sequence
;#Define	I2C_WRITE_TO_SLAVE	GFlags,4	; used for write sequence
;#Define	I2C_STOP	GFlags,5	; used to set bit for stop sequence
;#Define	I2C_TRANS_WRT_ADD	GFlags,6	; bit sets up write address transmission
;#Define	I2C_INIT_TRANS_DATA	GFlags,7	; bit sets up data transmission
;#Define	I2C_TRANS_DATA	GFlags2,0	; used to monitor transmission
;#Define	I2C_TRANS_COMPLETE	GFlags2,1	; set when transmission is complete
;#Define	I2C_TRANS_RD_ADD	GFlags2,2	; sets up read address transmission
;#Define	I2C_READ_FROM_SLAVE	GFlags2,3	; used for read sequence
;#Define	I2C_INIT_MSTR_REC	GFlags2,4	; initializes reception
;#Define	I2C_SET_RCEN	GFlags2,5	; initializes RCEN bit to receive data
;#Define	I2C_REC_BYTE	GFlags2,6	; sets up receive byte sequence
;#Define	I2C_READ_REC_BYTE	GFlags2,7	; read and store data
;#Define	I2C_SET_ACKEN	GFlags3,0	; sets up acknowledge sequence
;#Define	I2C_REC_COMPLETE	GFlags3,1	; set when reception is complete
;
;	udata
;I2C_ARRAY_TX	res	RX_ELEMENTS	; array to transmit to slave
;I2C_ARRAY_RX	res	RX_ELEMENTS	; array to receive from slave
;
;
;=========================================================================================
;
;LOADFSR1	macro 	ADDRESS,INDEX 	; ADDRESS = I2C_ARRAY, INDEX = INDEX_I2C			
;	movlw 	low ADDRESS 	; load address 				
;	addwf	INDEX,W	; add the index value to determine location in array
;	movwf 	FSR1L	; load FSR1L with pointer info
;	movlw	high ADDRESS
;	movwf	FSR1H
;	endm
;
;=========================================================================================
;----------------------------------- interrupt service routine ---------------------------
;
;IRQ_4	MOVLB	0x00
;	btfss 	PIR1,SSP1IF 	; Is this a SSP interrupt?
;	goto 	IRQ_4_End 	; if not, bus collision int occurred
;	btfsc	I2C_WRITE_TO_SLAVE	; is it a write sequence?
;	goto	I2C_WRITE	; if so go here
;	goto	I2C_READ	; if not, it is a read sequence
;I2C_READ_Return:
;I2C_WRITE_Return	movlb	0x00
;	bcf 	PIR1,SSPIF	; clear the SSP interrupt flag
;IRQ_4_End
;
;-----------------------------------------------------------------------------------------
; I2C Bus Collision
;IRQ_5	MOVLB	0x00
;	btfss	PIR2,BCLIF
;	goto	IRQ_5_End
;	movlb	0x04	;banksel SSPBUF						
;	clrf	SSP1BUF	; clear the SSP buffer
;	movlb	0x00	;banksel PIR2
;	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag	
;
;IRQ_5_End
;
;=========================================================================================
;=========================================================================================
;	Main
;-----------------------------------------------------------------------------------------
;
;	call	INITIALIZE_I2C	; set up uC
;
;MainLoop
;	call	Idle_I2C
;=========================================================================================
;
Idle_I2C	btfss	I2C_WRITE_TO_SLAVE	; is it a write sequence?
	goto	SET_READ_I2C	; if not set up master to read
	btfss	I2C_INIT_START	; is start bit set?
	goto	INIT_TRANSMIT_I2C	; if not, check if transmission ready
	bcf	I2C_INIT_START	; if so, clear start bit
	banksel	SSP1CON2
	bsf	SSP1CON2,SEN	; set start enable bit
Idle_I2C_end	movlb	0x00
	return
;
; set up transmit sequence
;
INIT_TRANSMIT_I2C	btfss	I2C_INIT_TRANS_DATA	; is bit set?
	goto	SET_PEN_BIT_I2C	; if not, transmit of data complete
	bcf	I2C_INIT_TRANS_DATA	; if so, clear bit
	bsf	I2C_TRANS_DATA	; set bit for transmission seq in INT routine
	return
;
; set up stop sequence
;
SET_PEN_BIT_I2C	btfss	I2C_TRANS_COMPLETE	; is transmission complete?
	return		; if not, go back to LOOP
	bcf	I2C_TRANS_COMPLETE
	bsf	I2C_STOP	; if so, set stop bit
	banksel	SSP1CON2
	bsf	SSP1CON2,PEN	; enable stop sequence
	call	DELAY	; short delay while stop sequence is performed
	goto	Idle_I2C_end
;
; set up READ routines
;
SET_READ_I2C	btfsc	I2C_WRITE_TO_SLAVE	; make sure write is complete
	return		; if not, go back to LOOP
	btfsc	I2C_READ_FROM_SLAVE	; is READ sequence bit set?
	goto	READ_SEQUENCE_I2C	; if so, go read
	call	DELAY	; if not, short delay
	bcf	I2C_REC_COMPLETE	; clear receive complete bit
	clrf	INDEX_I2C	; clear index
	bsf	I2C_INIT_START	; set start bit
	bsf	I2C_READ_FROM_SLAVE	; set read sequence bit
	return
;
; start read sequence
;
READ_SEQUENCE_I2C	btfss	I2C_READ_FROM_SLAVE	; make sure read sequence is set
	return		; if not, go back to LOOP
	btfss	I2C_INIT_START	; is start bit set?
	goto	PREP_TO_REC_I2C	; if not, go get ready to read
	bcf	I2C_INIT_START	; if so, clear bit
	banksel	SSPCON2
	bsf	SSPCON2,SEN	; set SEN bit to begin start sequence
	return
;
; get ready to receive data
;
PREP_TO_REC_I2C	btfss	I2C_INIT_MSTR_REC	; is bit set?
	goto	RECEIVE_BYTE_I2C	; if not, go receive data
	bcf	I2C_INIT_MSTR_REC	; if so, clear bit
	bsf	I2C_SET_RCEN	; set this to so in next INT RCEN is set
	return
;
; receive data byte
;
RECEIVE_BYTE_I2C	btfss	I2C_REC_BYTE	; is bit set?
	goto	ACKNOWLEDGE_I2C	; if not, byte received so go acknowledge
	bsf	I2C_READ_REC_BYTE	; if so, go receive data
	return
;
ACKNOWLEDGE_I2C	btfss	I2C_SET_ACKEN	; is bit set?
	goto	RECEIVE_COMPLETE_I2C	; if not, finish read routines
	bcf	I2C_SET_ACKEN
	bsf	I2C_INIT_MSTR_REC
	banksel	SSP1CON2
	bsf	SSP1CON2,ACKEN	; if so, set ACK bit to acknowledge slave
	goto	Idle_I2C_end
;
; finish up read routine
;
RECEIVE_COMPLETE_I2C	btfss	I2C_REC_COMPLETE	; is bit set?
	return		; if not, go back to LOOP
	bsf	I2C_STOP	; if so, set up stop
	banksel	SSP1CON2
	bsf	SSP1CON2,PEN	; set stop bit to begin stop sequence
	call	DELAY
	goto	Idle_I2C_end
;
; delay routine
;
DELAY	banksel	COUNTERL
	decfsz	COUNTERL,F
	goto	DELAY
	decfsz	COUNTERH,F
	goto	DELAY
	return
;
;------------------------------------- end main ------------------------------------------
;
;-----------------------------------------------------------------------------------------
;	Initialize: Sets up register values
;-----------------------------------------------------------------------------------------
;
INITIALIZE_I2C	movlw	low I2C_ARRAY_TX
	movwf	FSR1L
	movlw	high I2C_ARRAY_TX
	movwf	FSR1H
	movlw	RX_ELEMENTS
	movwf	INDEX_I2C
	movlw	I2C_TX_Init_Val
INITIALIZE_I2C_L1	movwf	INDF1
	incf	FSR1L
	decfsz	INDEX_I2C
	goto	INITIALIZE_I2C_L1
;
	movlw	low I2C_ARRAY_RX
	movwf	FSR1L
	movlw	high I2C_ARRAY_RX
	movwf	FSR1H
	movlw	RX_ELEMENTS
	movwf	INDEX_I2C
	movlw	I2C_RX_Init_Val
INITIALIZE_I2C_L2	movwf	INDF1
	incf	FSR1L
	decfsz	INDEX_I2C
	goto	INITIALIZE_I2C_L2
;
FIN_LOAD	bsf	I2C_INIT_START	; set start bit
	bcf	I2C_STOP	; clear stop bit
	bsf	I2C_WRITE_TO_SLAVE	; set WRITE bit
	bsf	I2C_TRANS_WRT_ADD	; set bit to ready write add seq.
	bcf	I2C_INIT_TRANS_DATA
	bcf	I2C_TRANS_DATA
	bcf	I2C_TRANS_COMPLETE
	bsf	I2C_TRANS_RD_ADD
	bcf	I2C_READ_FROM_SLAVE
	bcf	I2C_INIT_MSTR_REC
	bcf	I2C_SET_RCEN
	bcf	I2C_REC_BYTE
	bcf	I2C_READ_REC_BYTE
	bcf	I2C_SET_ACKEN
	bcf	I2C_REC_COMPLETE
;
;I2C set up
	movlb	0x04	; banksel SSPSTAT
	bsf	SSP1STAT,SMP	; Slew rate control disabled for standard speed mode
	movlw	b'00101000'	; Enable serial port, I2C master mode, 7-bit address
	movwf	SSP1CON1
	bsf	SSP1CON3,SDAHT	; Minimum of 300 ns hold time
	movlw	BRG_VAL	; load thE baud rate value
	movwf	SSP1ADD

	movlb	0x01	; banksel PIE2
	bsf	PIE1,SSP1IE	; enable SSP interrupts
	bsf	PIE2,BCL1IE	; enable SSP interrupts
	movlb	0x00	; banksel PIR1
	bcf	PIR1,SSP1IF	; clear the SSP interrupt flag
	bcf	PIR2,BCL1IF	; clear the SSP interrupt flag
	return
;
;------------------------------------- END INITIALIZE ------------------------------------
;
;-----------------------------------------------------------------------------------------
; 	Interrupt Service Routine (ISR)
;-----------------------------------------------------------------------------------------
;
; DATA TRANSMISSION SEQUENCE OF EVENTS
;
I2C_WRITE	btfsc	I2C_TRANS_COMPLETE	; is transmission complete?
	goto	END_TRANSMISSION	; if so, finish transmission
	btfss	I2C_TRANS_WRT_ADD	; if not, has trans add been sent?
	goto	TRANSMIT_DATA	; if so, go start to transmit
	bcf	I2C_TRANS_WRT_ADD	; if not, send trans add
	bsf	I2C_INIT_TRANS_DATA
	banksel	SSP1BUF
	movlw	WRITE_ADD
	movwf	SSP1BUF
	goto	I2C_WRITE_Return
;
;
TRANSMIT_DATA	btfss	I2C_TRANS_DATA	; is transmission complete?
	goto	END_TRANSMISSION	; if so, go finish sequence
;
	movlw	RX_ELEMENTS	; load array elements value
	subwf	INDEX_I2C,W	; if Z = 1, subtract index from number of elements
	btfsc	_C	; did a carry occur after subtraction?
	goto	COMP_TRANS	; if so, array limit reached
;
	LOADFSR1	I2C_ARRAY_TX,INDEX_I2C
	movf	INDF1,W	; move value into W to load to SSP buffer
	banksel	SSP1BUF
	movwf	SSP1BUF	; load SSP buffer
	incf	INDEX_I2C,F	; increment INDEX_I2C 'pointer'
	btfsc	SSP1CON1,WCOL	; did a write collision occur?
	bcf	SSP1CON1,WCOL	; if so, clear bit
	goto	I2C_WRITE_Return
;
;
; is transmission complete?
;
COMP_TRANS	bcf	I2C_TRANS_DATA	; clear bit
	bsf	I2C_TRANS_COMPLETE	; get ready to end sequence
	goto	I2C_WRITE_Return
;
; finish transmit sequence
;
END_TRANSMISSION	btfss	I2C_STOP	; is stop set?
	goto	I2C_WRITE_Return	; yes, we're done
	bcf	I2C_STOP	; if so, clear stop
	bcf	I2C_WRITE_TO_SLAVE	; clear bit to prevent more transmissions
	goto	I2C_WRITE_Return
;
; end of write routine
;=========================================================================================
; READ FROM SLAVE SEQUENCE OF EVENTS
;
I2C_READ	btfsc	I2C_REC_COMPLETE	; finished reading?
	goto	END_RECEPTION	; if so, end read sequence
	btfss	I2C_TRANS_RD_ADD	; if not, was read add transmitted to slave?
	goto	ENABLE_REC	; if so, set master to receive
	bcf	I2C_TRANS_RD_ADD	; clear bit
	bsf	I2C_INIT_MSTR_REC	; set bit to get ready next INT
	banksel	SSP1BUF
	movlw	READ_ADD	; transmit read address to slave
	movwf	SSP1BUF
	goto	I2C_READ_Return
;
; sets up master as a receiver
;
ENABLE_REC	btfss	I2C_SET_RCEN	; is bit set?
	goto	READ_DATA_BYTE	; if not, master is in receive mode
	bcf	I2C_SET_RCEN	; otherwise, get set up
	bsf	I2C_REC_BYTE
	banksel	SSP1CON2
	bsf	SSP1CON2,RCEN	; set RCEN to enable master receive mode
	goto	I2C_READ_Return
;
; read the data byte from slave
;
READ_DATA_BYTE	btfss	I2C_READ_REC_BYTE	; was all data received?
	goto	COMP_RECEIVE	; if so, finish sequence
	bcf	I2C_READ_REC_BYTE	; otherwise, read data and store in array
	bcf	I2C_REC_BYTE
	;bsf	I2C_INIT_MSTR_REC
;
	movlw	RX_ELEMENTS	; load array elements value
	subwf	INDEX_I2C,W	; if Z = 1, subtract index from number of elements
	btfsc	_C	; did a carry occur after subtraction?
	goto	COMP_RECEIVE	; if so, Master is trying to write to many bytes
;
	LOADFSR1	I2C_ARRAY_RX,INDEX_I2C
	banksel	SSP1BUF
	movfw	SSP1BUF	; move the contents of the buffer into W
	movwf 	INDF1	; load INDF1 with data to write
	incf	INDEX_I2C,F	; increment INDEX_I2C 'pointer'
	bsf	I2C_SET_ACKEN	; set up ack sequence in MAIN
	goto	I2C_READ_Return
;
; get ready to wrap things up
;
COMP_RECEIVE	bcf	I2C_INIT_MSTR_REC
	bcf	I2C_REC_BYTE
	bcf	I2C_READ_REC_BYTE
	bsf 	I2C_REC_COMPLETE
	goto	I2C_READ_Return
;
END_RECEPTION	btfsc	I2C_STOP
	goto	I2C_READ_Return
	bcf	I2C_READ_FROM_SLAVE
	goto	I2C_READ_Return
;
;

