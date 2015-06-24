;--------------------------------------------------------
; Freeware.
; Digital Thermometer with sensor DS1820.
; -55C - +99C on LCD-display (16x1)	 
; Version: 0.21
;
; Developer: Claes Sundman (menola@home.se)
; 
;
; Thanks to:
;
; Wayne Peacock wayne.peacock@senet.com.au
; buffi (Christoph Redecker) donbuffi@t-online.de
; Grahame grahame@cocal.co.uk
; tony.pattison (Tony Pattison) tony@appliedlaser.co.uk
; val2048 (valera kolupaev) val_85@mail.ru
;
; and other @ avrfreaks.net
;	
;--------------------------------------------------------
;
; 				AT90S2313	
;			PIN					PIN			
; RESET 1	Reset 	  		20	VCC	+5v	
; PD0 	2   (RXD)     		19  PB7 D7 
; PD1	3	(TXD)			18	PB6	D6		
; X1 	4 	4Mhz			17	PB5	D5
; X2	5 	4Mhz			16	PB4	D4				  
; PD2 	6	nc				15	PB3	RS						
; PD3 	7	nc				14	PB2	R/W	
; PD4 	8 	nc				13	PB1	E	
; PD5	9 	1-Wire_Data		12	PB0	Led_Background_Power		
; GND	10	Gnd				11 	PD6	1-Wire_Power
;
;--------------------------------------------------------
.include "2313def.inc"

.DSEG
.def	tmp1			= r16	;General use working register
.def	tmp2			= r17
.def	tmp3			= r18
.def	tmp4			= r19
.def	timeout			= r20	;Timeout value passed to subroutine
.def	ioreg			= r21	;Register for reading/writing DS1820
.def	temphi			= r22	;High byte of temperature reading
.def	templo			= r23	;Low byte of temperature reading
.def	loopcnt			= r24	;Loop counter for serial read/write

.equ	dsio			= 6							;I/O pin connected to PD1
.equ	T0PRE			= 0x02						;Timer 0 prescaler 4MHz / 8 = 2uS
;.equ	BAUDcode			= 25					;9600 bps at 4.00 MHz.		
;.equ    clock       	= 4000000  					; clock frequency 
;.equ    baudrate    	= 9600     					; choose a baudrate 
;.equ    baudconstant	= (clock/(16*baudrate))-1

;--------------------------------------------------------
.cseg
.org $000
rjmp	Reset			;Reset Vector
.org	INT0addr		;External Interrupt0 Vector
	reti
.org	INT1addr		;External Interrupt1 Vector
	reti
.org	ICP1addr		;Input Capture1 Interrupt Vector
	reti
.org	OC1addr 		;Output Compare1 Interrupt Vector
	reti
.org	OVF1addr		;Overflow1 Interrupt Vector
	reti
.org	OVF0addr		;Overflow0 Interrupt Vector
	rjmp	t0int
.org	URXCaddr		;UART Receive Complete Interrupt Vector
	reti
.org	UDREaddr		;UART Data Register Empty Interrupt Vector
	reti
.org	UTXCaddr		;UART Transmit Complete Interrupt Vector
	reti
.org	ACIaddr			;Analog Comparator Interrupt Vector
	reti

;--------------------------------------------------------
Reset:
	ldi		tmp1, 	low(RAMEND)
    out   	SPL, 	tmp1	
	ldi		tmp1,	0xFF
	out		DDRB,	tmp1
	out		PORTB,	tmp1
	out		DDRD,	tmp1
	out		PORTD,	tmp1
	ldi		tmp1,	0b00000010	;Enable Timer 0 interrupt
	out		TIMSK,	tmp1
;	ldi		tmp1,	BAUDcode
;	out		UBRR,	tmp1		;Set baud rate generator
;	ldi		tmp1,	0b00001000
;	out		UCR,tmp1			;Enable UART tx w/o interrupt
	sei							;Enable interrupts

;--------------------------------------------------------
main:
	rcall	init_port
	rcall	lcd_init
	rcall	lcd_write_text
	rcall	DS1820 			; infinite loop inside subrutine DS1820

;--------------------------------------------------------
; Enable background light to LCD-disp.
led_on:
	sbi		portb, 	0
ret

led_off:
	cbi		portb, 	0
ret

;--------------------------------------------------------
; Init the LCD in 4-bit mode
lcd_init:			
	push		tmp1
	ldi			tmp1, 	0x30
	out			portb, 	tmp1
	rcall		enable
	rcall		wait
	rcall		wait

	ldi			tmp1, 	0x30
	out			portb, 	tmp1
	rcall		enable
	rcall		wait

	ldi			tmp1, 	0x30
	out			portb, 	tmp1
	rcall		enable
	rcall		wait

	ldi			tmp1, 	0x20
	rcall		lcd_cmd

	ldi			tmp1, 	0x28
	rcall		lcd_cmd

	ldi			tmp1, 	0x14
	rcall		lcd_cmd

	ldi			tmp1, 	0x0c
	rcall		lcd_cmd

	ldi			tmp1, 	0x06
	rcall		lcd_cmd

	ldi			tmp1, 	0x01
	rcall		lcd_cmd

	ldi			tmp1, 	0x80
	rcall		lcd_cmd
	pop			tmp1
ret				

;--------------------------------------------------------
; Write content " Temp:      C" to LCD
lcd_write_text:
	ldi		tmp1, 	' '
	rcall	lcd_char
	ldi		tmp1, 	'T'
	rcall	lcd_char
	ldi		tmp1, 	'e'
	rcall	lcd_char
	ldi		tmp1, 	'm'
	rcall	lcd_char
	ldi		tmp1, 	'p'
	rcall	lcd_char
	ldi		tmp1, 	':'
	rcall	lcd_char
	ldi		tmp1,	0xC5
	rcall	lcd_cmd
	ldi		tmp1, 	0b11011111
	rcall	lcd_char
	ldi		tmp1, 	'C'
	rcall	lcd_char
ret

;--------------------------------------------------------
; init the LCD in 4-bit mode 
init_port:
	ldi			tmp1, 	0xff
	out			ddrb, 	tmp1
ret

;--------------------------------------------------------
; LCD-commands
lcd_cmd:
	push		tmp1
	push		tmp2
	rcall		lcd_busyread
	mov			tmp2, 	tmp1	;Load the value to the tmp2 also.
	andi		tmp2, 	0xf0	;select the upper 4bits of the sent byte
	ori			tmp2, 	0x00	;mask the tmp2 RS=0
	out			portb, 	tmp2	;put result out to portc
	rcall		enable
	mov			tmp2, 	tmp1	;load sent value to tmp2
	andi		tmp2, 	0x0f	;select the 4 lower bits
	swap		tmp2			;swap the lower D3-D0 bits to D7-D4
	ori			tmp2, 	0x00	;mask the tmp2 RS=0
	out			portb, 	tmp2	;put result out to portc
	rcall		enable
	pop			tmp2
	pop			tmp1
	rcall		wait_short
ret

;--------------------------------------------------------
; Write to LCD
lcd_char:
	push		tmp1
	push		tmp2
	rcall		lcd_busyread
	mov			tmp2, 	tmp1	;Load the value to the tmp2 also.
	andi		tmp2, 	0xf0	;select the upper 4bits of the sent byte
	ori			tmp2, 	0x02	;mask the tmp2 RS=1
	out			portb, 	tmp2	;put result out to portc
	rcall		enable
	mov			tmp2, 	tmp1	;load sent value to tmp2
	andi		tmp2, 	0x0f	;select the 4 lower bits
	swap		tmp2			;swap the lower D3-D0 bits to D7-D4
	ori			tmp2, 	0x02	;mask the tmp2 and RS=1
	out			portb, 	tmp2	;put result out to portc
	rcall		enable
	pop			tmp2
	pop			tmp1
ret

;--------------------------------------------------------
; Test busyflag
lcd_busyread:
	push 		tmp1
	ldi			tmp1, 	0x00	;clear portb
	out			portb, 	tmp1	;make it happen
	ldi			tmp1, 	0x0f	;turn D7-D4 to inputs
	out			ddrb, 	tmp1	;make it happen 
lcd_busyread_wait:	
	sbi			portb, 	2		;R/W = 1
	sbi			portb, 	3		;Enable = 1
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	in			tmp1,	pinb	;read from LCD
	cbi			portb, 	3		;Enable = 0
	nop
	nop
	nop
	nop
	rcall 		enable			;to complete reading the byte
	andi		tmp1, 	0x80	;want only D7
	cpi			tmp1, 	0x00	;is it zero ?
	brne		lcd_busyread_wait	;jump if not equal to zero
	ldi			tmp1, 	0xff	;make portb to output
	out			ddrb,	tmp1	;make it happen
	pop			tmp1
ret

;--------------------------------------------------------
; Toggle the Enable on the LCD 
enable:				
	sbi			portb, 	3		;pin B1 = 1
	push		tmp1
	ldi			tmp1, 	10
enable_loop:
	dec			tmp1
	brne		enable_loop
	pop			tmp1
	cbi			portb, 	3		;pin B1 = 0
ret

;--------------------------------------------------------
; Wait-loop
wait_short:
	push		tmp1
	ldi			tmp1, 	0x80
wait_short_loop:
	dec			tmp1
	brne		wait_short_loop
	pop			tmp1
ret

wait:
	push		tmp2
	ldi			tmp2, 	0xff
wait_loop:
	rcall		wait_short
	dec			tmp2
	brne		wait_loop
	pop			tmp2
ret

;--------------------------------------------------------
; LCD_PrintMem. Prints from memory. Put the starting, 
; memory location in Z (r31:r30). Put the number of 
; characters to print in tmp2. After execution, 
; Z is at the character AFTER the last to be 
; printed and tmp2 is zero. This function will 
; not wrap if you the string is bigger than the LCD.

LCD_PrintMem:	
	push	tmp1
LCD_MemRead:
   	ld		tmp1, 		Z+
	rcall	lcd_char
	dec		tmp2
	brne	LCD_MemRead
	pop		tmp1
ret

;--------------------------------------------------------
; LCD_PrintPM. Prints from program memory

LCD_PrintPM:	
	push	r0
	push	tmp1
LCD_PMRead: 
	lpm
    mov		tmp1, 		r0
	rcall	lcd_char
	adiw	r30, 		1
	dec		tmp2
	brne	LCD_PMRead
	pop		tmp1
	pop		r0
ret

;--------------------------------------------------------
; Temp. sensor subrutine
DS1820:
	ldi		tmp1,	0xC2
	rcall	lcd_cmd
	rcall	led_on
	rcall	resetds			; Reset DS1820 device
	ldi		ioreg,	0xCC	; Skip ROM command
	rcall	writeds
	ldi		ioreg,	0x44	; Convert T command
	rcall	writeds
twait:
	rcall	readds			; Check for conversion complete
	tst		ioreg			; Returns all zeros while busy
	breq	twait
	rcall	resetds			; Reset again
	ldi		ioreg,	0xCC	; Skip ROM command
	rcall	writeds
	ldi		ioreg,	0xBE	; Read scratchpad command
	rcall	writeds
	rcall	readds			; Read temperature LSB
	mov		templo,	ioreg	; Store reading
	rcall	readds			; Read temperature MSB
	mov		temphi,	ioreg	; Store reading
	rcall	resetds			; Don't read the rest
	rcall	readds			; Read temp.high - not used
	mov		tmp1,	ioreg	; Store reading
	rcall	readds			; Read temp.low - not used
	mov		tmp1,	ioreg	; Store reading
	rcall	readds			; Read Reserved - not used
	mov		tmp1,	ioreg	; Store reading
	rcall	readds			; Read Reserved - not used
	mov		tmp1,	ioreg	; Store reading
;- - - - - not used - - - - - - - - - - - - - - - - - - -
	rcall	readds			; Read Count_remain, in the future...
	mov		tmp1,	ioreg	; Store reading
	rcall	readds			; Read Count_per_C 
	mov		tmp1,	ioreg	; Store reading
;- - - - - - - - - - - - - - - - - - - - - - - - - - - - 
	ldi		tmp1,	'+'		; write "+"-sign to lcd
	tst		temphi			; See if temperature is negative
	breq	x1
	neg		templo			; Change to negative
	ldi		tmp1,	'-'		; Print a minus sign
	subi	templo,	0xFF
	inc		templo
x1:
	rcall	lcd_char		; write "-"-sign to lcd
	ror		templo
	rcall	hex2ascii
	mov		tmp1,	tmp3
	rcall	lcd_char
	mov		tmp1,	tmp2
	rcall	lcd_char		; write to lcd "+x5 C"
rjmp		DS1820			; Do it again.....

;--------------------------------------------------------
; Write a byte to the DS1820
writeds:
	ldi		loopcnt,8		;Send 8 bits to device
wrloop:
	sbi		DDRD,	dsio	;Make I/O pin an output
	cbi		PORTD,	dsio	;Pull I/O pin low
	ldi		timeout,	-1	;Make timer 0 count 2uS
	rcall	delay
	ror		ioreg			;Get bit into carry
	brcc	wrzero			;If clear, leave line low
	sbi		PORTD,	dsio	;Otherwise pull line high
wrzero:
	ldi		timeout,	-30	;Make timer 0 count 30 * 2uS
	rcall	delay
	sbi		PORTD,	dsio	;Pull line back high
	ldi		timeout,-1		;Make timer 0 count 2uS
	rcall	delay
	dec		loopcnt			;Decrement bit counter
	brne	wrloop			;Loop for all 8 bits
ret

;--------------------------------------------------------
; Read a byte from the DS1820
readds:
	ldi		loopcnt,8		;Get 8 bits from device
	ldi		ioreg,	0		;Clear holding register
rdloop:
	sbi		DDRD,	dsio	;Make I/O pin an output
	cbi		PORTD,	dsio	;Pull I/O pin low
	ldi		timeout,-1		;Make timer 0 count 2uS
	rcall	delay
	cbi		DDRD,	dsio	;Make I/O pin an input, will pull up high
	ldi		timeout,-5		;Make timer 0 count 5 * 2uS
	rcall	delay			;Give pin time to go high if needed
	clc						;clear carry, assume bit is a zero
	sbic	PIND,dsio		;Check I/O pin
	sec						;Set carry if pin was high
	ror		ioreg			;Get bit into register
	ldi		timeout,-25		;Make timer 0 count 25 * 2uS
	rcall	delay
	dec		loopcnt			;Decrement bit counter
	brne	rdloop			;Loop for all 8 bits
ret

;--------------------------------------------------------
; Reset DS1820 and read presence pulse
resetds:
	sbi		DDRD,	dsio	;Make I/O pin an output
	cbi		PORTD,	dsio	;Pull I/O pin low
	ldi		timeout,6		;Make timer 0 count 250 * 2uS
	rcall	delay
	sbi		PORTD,	dsio	;Pull I/O pin high
	cbi		DDRD,	dsio	;Make I/O pin an input
	ldi		timeout,-45		;Make timer 0 count 45 * 2uS
	rcall	delay
	sbis	PIND,	dsio	;Check for presence pulse
	ldi		timeout,6		;Make timer 0 count 250 * 2uS
	out		TCNT0,	tmp1
	rjmp	delay			;Fall through delay routine

;--------------------------------------------------------
; Timer 0 overflow interrupt handler
t0int:
	set						;Set T flag
	ldi		tmp1,	0		;Timer 0 off
	out		TCCR0,	tmp1	;Stop timer
reti						;Done, return

;--------------------------------------------------------
; Delay n*2 microseconds using timer 0, delay time passed in timeout
delay:
	out		TCNT0,	timeout
	clt						;Clear T
	ldi		tmp1,	T0PRE	;Timer 0 prescaler
	out		TCCR0,	tmp1	;Run timer
dwait:
	brtc	dwait			;Wait for timer 0 interrupt to set T
ret

;--------------------------------------------------------
; Convert from temp.sensor to LCD
hex2ascii: 
	ldi 	tmp2,	0x30	; ones 
    ldi 	tmp3,	0x30 	; tens
    ldi 	tmp4,	0x30	; hundreds - not used!
	cpi 	templo,	0x64 	; compare with 0x64 = 100 
    brsh 	hex2ascii_1		; more than 100      
    cpi 	templo,	0x0A 	; compare if we have 10 inside the number 
    brsh 	hex2ascii_2    	; test if same or higher
    add 	tmp2,	templo	; ones only
ret		
         
hex2ascii_1:      	
	subi 	templo,	0x64 	; subtracting 100 
    inc 	tmp4    	   
    cpi 	templo,	0x64    ; repeat
    brsh 	hex2ascii_1       
    cpi 	templo,	0x0A  	; test if we have 10 inside. 
    brsh 	hex2ascii_2          
hex2ascii_2:     
	subi 	templo,	0x0A 	; subtracting 10
    inc 	tmp3
    cpi 	templo,	0x0A 	; repeat
    brsh 	hex2ascii_2 
    add 	tmp2,	templo
ret

;--------------------------------------------------------