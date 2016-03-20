; This code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
; The code provided initializes all 3 ports (A,B,E) with the ECE LED bar array plugged into the Tiva board
; Port F with the Tiva 3 LEDs (Red, Green, Blue) and two buttons is also initialized
; Then the LEDs on each port are turned off and on with time delays - while the Tiva board R, G, B LEDs are turned on and off

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

; These equates allow one to associate a name with a value to make the code more readable
GPIO_PORTF_AFSEL_R EQU 0x40025420
GPIO_PORTF_PUR_R   EQU 0x40025510
GPIO_PORTF_DEN_R   EQU 0x4002551C
GPIO_PORTF_LOCK_R  EQU 0x40025520
GPIO_PORTF_CR_R    EQU 0x40025524
GPIO_PORTF_AMSEL_R EQU 0x40025528
GPIO_PORTF_PCTL_R  EQU 0x4002552C
	
RED       EQU 0x02					; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; on the left side of the Tiva board
SW2       EQU 0x01                 ; on the right side of the Tiva board

Start
	BL  Port_Init                  ; initialize input and output pins of Ports A to F
	BL  LED_OFF
	MOV R11, #0xABCD				;RANDOM 16 bit
loop
	BL  RandomNum
	MOV R0, R11						;for delay
	MOV R8, #50000					;multiplier
	BL Delay
	MOV R3, #1
	BL DISPLAY_NUM					; To turn on the led to test reflex
	MOV R4, #0						; reflex Counter
	
poll	
    BL  PortF_Input                 ; read all of the switches on Port F
    CMP R0, #0x01                   ; R0 == 0x01?
    BEQ sw1pressed                  ; if so, switch 1 pressed
	ADD R4, #1
	MOV R8, #1						; MULTIPLIER
	BL Delay
	
	B   poll

sw1pressed
	MOV R5, R4
	
display_loop	
	MOV R4, R5
	MOV R6, #4	;Countdown
inner_loop	
	AND R3, R4,#0xFF
	BL DISPLAY_NUM 	;Displays the 8 bits in R3
	LSR R4, #8
	;AND R3, R4,#0xFF
	MOV R8, #20000
	BL Delay
	SUBS R6, #1
	BNE inner_loop
	MOV R8, #45000
	BL Delay
	B display_loop
	

;------------RandomNum------------
; R11 holds a 16-bit random number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; R11 holds a non-zero 16-bit number.  If a zero is fed in, as the seed, the pseudo-random sequence will stay stuck at 0
; Take as many bits of R11 as you need.  If you take the lowest 4 bits then you get a number between 1 and 15.
;   If you take bits 5..1 you'll get a number between 0 and 15 (assuming you right shift by 1 bit).
;
; R11 MUST be initialized to a non-zero 16-bit value at the start of the program OR ELSE!
; R11 can be read anywhere in the code but must only be written to by this subroutine
;
RandomNum		STMFD		R13!,{R1, R2, R3, R14}

				AND			R1, R11, #0x8000
				AND			R2, R11, #0x2000
				LSL			R2, #2
				EOR			R3, R1, R2
				AND			R1, R11, #0x1000
				LSL			R1, #3
				EOR			R3, R3, R1
				AND			R1, R11, #0x0400
				LSL			R1, #5
				EOR			R3, R3, R1		; the new bit to go into the LSB is present
				LSR			R3, #15
				LSL			R11, #1
				ORR			R11, R11, R3
				
				LDMFD		R13!,{R1, R2, R3, R15}


; NOTE: The NMI (non-maskable interrupt) is on PF0.  That means that
; the Alternate Function Select, Pull-Up Resistor, Pull-Down Resistor,
; and Digital Enable are all locked for PF0 until a value of 0x4C4F434B
; is written to the Port F GPIO Lock Register.  After Port F is
; unlocked, bit 0 of the Port F GPIO Commit Register must be set to
; allow access to PF0's control registers.  On the LM4F120, the other
; bits of the Port F GPIO Commit Register are hard-wired to 1, meaning
; that the rest of Port F can always be freely re-configured at any
; time.  Requiring this procedure makes it unlikely to accidentally
; re-configure the JTAG and NMI pins as GPIO, which can lock the
; debugger out of the processor and make it permanently unable to be
; debugged or re-programmed.

GPIO_PORTF_DIR_R   EQU 0x40025400		; Port F Data Direction Register setting pins as input or output
GPIO_PORTF_DATA_R  EQU 0x400253FC
	
; These are the configuration registers which should not be touched
; Port Base addresses for the legacy (not high-performance) interface to I/O ports
GPIO_PORTA			EQU 0x40004000
GPIO_PORTB			EQU 0x40005000
GPIO_PORTC			EQU 0x40006000
GPIO_PORTD			EQU 0x40007000
GPIO_PORTE			EQU 0x40024000
GPIO_PORTF			EQU 0x40025000

; These are the masks for pins which are outputs
PORT_A_MASK			EQU 0xE0		; PA7,6,5 are outputs for LEDs
PORT_B_MASK			EQU 0xFF		; PB5,4,1,0 are outputs %0011 0011
PORT_E_MASK			EQU 0x30		; PE5,4 are outputs %0011 0000
PORT_F_MASK			EQU 0x0e		; PF has LEDs on PF 1,2,3 and buttons PF0, PF4
	
; Offsets are from table 10-6 on page 660
GPIO_DATA_OFFSET	EQU 0x000		; Data address is the base address - YOU HAVE TO ADD AN ADDRESS MASK TOO to read or write this!!
GPIO_DIR_OFFSET		EQU 0x400		; Direction register
GPIO_AFSEL_OFFSET EQU 0x420			; Alternate Function SELection
GPIO_PUR_OFFSET   EQU 0x510			; Pull Up Resistors
GPIO_DEN_OFFSET   EQU 0x51C			; Digital ENable
GPIO_LOCK_OFFSET  EQU 0x520
GPIO_CR_OFFSET    EQU 0x524
GPIO_AMSEL_OFFSET EQU 0x528			; Analog Mode SELect
GPIO_PCTL_OFFSET  EQU 0x52C

SYSCTL_HBCTL  EQU   0x400FE06C		; high performance bus control for ports A to F

GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
SYSCTL_RCGCGPIO_R  EQU   0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2
Port_Init
	STMFD		R13!,{R14}		; push the LR or return address

; First enable the clock to the I/O ports, by default the clocks are off to save power
; If a clock is not enabled to a port and you access it - then the processor hard faults
	LDR R1, =SYSCTL_RCGCGPIO_R      ; activate clock for Ports (see page 340)
    LDR R0, [R1]                 
    ORR R0, R0, #0x3F               ; turn on clock to all 6 ports (A to F, bits 0 to 5)
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
	
; Set all ports to APB bus instead of AHB - this should be unnecessary
;	LDR R1, =SYSCTL_HBCTL
;	LDR R0, [R1]
;	AND R0, #0xFFFFFFE0		; set Ports A thru F to APB (0) and leave the rest at their default
;	STR R0, [R1]

; Page 650, Table 10-1 GPIO Pins with Special Considerations.
; These pins must be left as configured upon reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Init I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; R1 has to be the port address,
; R2 is the output pin mask
; R3 is the input pin mask

	MOV R3, #0x00				; Select no pins as input (unless it's changed as for port F)
	
; Init Port A, B, E are by default GPIO - set all output pins used to a 1 to enable them
;   and leave all of the other pins as previously configured!
    LDR R1, =GPIO_PORTA
    MOV R2, #PORT_A_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port B
    LDR R1, =GPIO_PORTB
    MOV R2, #PORT_B_MASK            ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port E
	LDR R1, =GPIO_PORTE
    MOV R2, #PORT_E_MASK                   ; enable commit for Port, 1 means allow access
	BL Port_Init_Individual

; Init Port F
	LDR R1, =GPIO_PORTF
    MOV R2, #PORT_F_MASK		; enable commit for Port, 1 means allow access
	MOV R3, #0x11				; enable weak pull-up on PF0 and PF4 (buttons)
	BL Port_Init_Individual

	LDMFD		R13!,{R15}		; pull the LR or return address from the stack and return

;------------Port_Init_Individual------------
; Initialize one GPIO Port with select bits as inputs and outputs
; Output: none
; Input: R1, R2, R3
; R1 has to be the port address
; R2 has to hold the mask for output pins
; R3 has to be the mask for input pins
; Modifies: R0

Port_Init_Individual
	STMFD		R13!,{R14}		; push the LR or return address
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1, #GPIO_LOCK_OFFSET]	; 2) unlock the lock register
	ORR R0, R2, R3					; all access to inputs and outputs as masked in R2 and R3
    STR R0, [R1, #GPIO_CR_OFFSET]	; enable commit for Port F
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1, #GPIO_AMSEL_OFFSET]	; 3) disable analog functionality
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1, #GPIO_PCTL_OFFSET]	; 4) configure as GPIO
    LDR R0, [R1, #GPIO_DIR_OFFSET]	; 5) read default direction register configuration
    ORR R0, R2						; ORR in only the bits we want as outputs
    STR R0, [R1, #GPIO_DIR_OFFSET]	; 5) set direction register
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1, #GPIO_AFSEL_OFFSET]	; 6) regular port function
    STR R3, [R1, #GPIO_PUR_OFFSET]	; pull-up resistors for PF4,PF0
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1, #GPIO_DEN_OFFSET]
	LDMFD		R13!,{R15}		; pull the LR or return address and return

; Delay subroutine - delay for a fixed amount of time
; R0 destroyed
; Input: none
; Output: R0 set to 0

MILISEC          	  EQU 534      ; approximately 0.1 ms delay at ~16 MHz clock

Delay
	LDR R0, =MILISEC   	       		; R0 = a value to get about a second delay
delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	delay_loop
	SUBS R8, R8, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	Delay

	; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
	BX LR
	
	
;------------PortF_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2
PortF_Init
    LDR R1, =SYSCTL_RCGCGPIO_R      ; 1) activate clock for Port F
    LDR R0, [R1]                 
    ORR R0, R0, #0x20               ; set bit 5 to turn on clock
    STR R0, [R1]                  
    NOP
    NOP                             ; allow time for clock to finish
    LDR R1, =GPIO_PORTF_LOCK_R      ; 2) unlock the lock register
    LDR R0, =0x4C4F434B             ; unlock GPIO Port F Commit Register
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_CR_R        ; enable commit for Port F
    MOV R0, #0xFF                   ; 1 means allow access
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_AMSEL_R     ; 3) disable analog functionality
    MOV R0, #0                      ; 0 means analog is off
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_PCTL_R      ; 4) configure as GPIO
    MOV R0, #0x00000000             ; 0 means configure Port F as GPIO
    STR R0, [R1]                  
    LDR R1, =GPIO_PORTF_DIR_R       ; 5) set direction register
    MOV R0,#0x0E                    ; PF0 and PF7-4 input, PF3-1 output
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_AFSEL_R     ; 6) regular port function
    MOV R0, #0                      ; 0 means disable alternate function 
    STR R0, [R1]                    
    LDR R1, =GPIO_PORTF_PUR_R       ; pull-up resistors for PF4,PF0
    MOV R0, #0x11                   ; enable weak pull-up on PF0 and PF4
    STR R0, [R1]              
    LDR R1, =GPIO_PORTF_DEN_R       ; 7) enable Port F digital port
    MOV R0, #0xFF                   ; 1 means enable digital I/O
    STR R0, [R1]                   
    BX  LR      

DISPLAY_NUM	STMFD R13!, {R2, R14}
	BL LED_OFF
    LDR R1, =GPIO_PORTB + (PORT_B_MASK << 2)
	AND R2, R3, #0xFD
    STR R2, [R1, #GPIO_DATA_OFFSET]
	
	LDR R1, =GPIO_PORTE + (PORT_E_MASK << 2)
	AND R2, R3, #0x02
	
    STR R2, [R1, #GPIO_DATA_OFFSET]
	LDMFD R13!, {R2, R15}
	
	
	BX LR
	
LED_OFF
    LDR R1, =GPIO_PORTB + (PORT_B_MASK << 2)
	MOV R0, #0x00       		; PB5,4,1,0 are outputs %0011 0011
    STR R0, [R1, #GPIO_DATA_OFFSET]
	
	LDR R1, =GPIO_PORTE + (PORT_E_MASK << 2)
	MOV R0, #0x0            ; PB5,4,1,0 are outputs %0011 0011
    STR R0, [R1, #GPIO_DATA_OFFSET]
	
	BX LR

;------------PortF_Input------------
; Read and return the status of the switches.
; Input: none
; Output: R0  0x01 if only Switch 1 is pressed
;         R0  0x10 if only Switch 2 is pressed
;         R0  0x00 if both switches are pressed
;         R0  0x11 if no switches are pressed
; Modifies: R1

PortF_Input
    LDR R1, =GPIO_PORTF_DATA_R ; pointer to Port F data
    LDR R0, [R1]               ; read all of Port F
    AND R0,R0,#0x11            ; just the input pins PF0 and PF4
    BX  LR                     ; return R0 with inputs

;------------PortF_Output------------
; Set the output state of PF3-1.
; Input: R0  new state of PF
; Output: none
; Modifies: R1
PortF_Output
    LDR R1, =GPIO_PORTF_DATA_R 		; pointer to Port F data
    STR R0, [R1]               		; write to PF3-1
    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file
;Lab Report Answer:;8-bits - 0.0255 second;16-bits - 6.5535 seconds;24-bits - 1677.7215 seconds = 27.962025 minutes;32-bits -  429496.7295 seconds = 119.3046470833 hours;16 bits is the best size to measure the typical human reaction time since it can’t be;more than 6.5 seconds but is not less than 0.0255 seconds
