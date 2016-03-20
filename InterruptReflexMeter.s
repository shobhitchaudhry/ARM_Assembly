; This code is based upon InputOutput.s from the book:
;  "Embedded Systems: Introduction to ARM Cortex M Microcontrollers"
;  ISBN: 978-1469998749, Jonathan Valvano, copyright (c) 2014
;
; The code provided initializes all 3 ports (A,B,E) with the ECE LED bar array plugged into the Tiva board
; Port F with the Tiva 3 LEDs (Red, Green, Blue) and two buttons is also initialized

        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start
		EXPORT restart

; These equates allow one to associate a name with a value to make the code more readable

RED       EQU 0x02		; These are the values (bit locations) for various LEDs on the Tiva (Port F)
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; on the left side of the Tiva board
SW2       EQU 0x01                 ; on the right side of the Tiva board
	MOV  R6, #0
Start
	CMP  R6, #1
	BEQ restart						; donot want to reinitialise
	
	BL  Port_Init					; initialize input and output pins of Ports A to F
	BL	Interrupt_Init				; Init interrupts for the switches on the Tiva
;
; Insert your main program here
;
restart

	
	BL 	LED_OFF					 		;turn off all LEDs

	MOV R11, #0xABCD					;RANDOM 16 bit
	BL  RandomNum
	
	MOV R3, R11							; count register initialised to a random number
	AND R3, #0xff						; first 8 bits only

loop
	BL	DISPLAY_NUM						; display count down
	MOV R8, #10							; multiplier = 10
	BL 	Delay
	SUBS R3, #0xA						; countdown from random number
	BHS	loop							; while positive loop
	MOV R3, #0
    BL	DISPLAY_NUM						; display 0
	MOV  R6, #1
	
	B	loop			; an infinite loop around the main program


;----------------RandomNum--------------------
; R11 holds a 16-bit random number via a pseudo-random sequence as per the Linear feedback shift register (Fibonacci) on WikiPedia
; R11 holds a non-zero 16-bit number.  If a zero is fed in the pseudo-random sequence will stay stuck at 0
; Take as many bits of R11 as you need.  If you take the lowest 4 bits then you get a number between 1 and 15.
;   If you take bits 5..1 you'll get a number between 0 and 15 (assuming you right shift by 1 bit).
;
; R11 MUST be initialized to a non-zero 16-bit value at the start of the program OR ELSE!
; R11 can be read anywhere in the code but must only be written to by this subroutine
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


;
; -----------Interrupt_Init--------------
; Setup interrupts so that a switch on Port F generates an interrupt
; Input: none
; Output: none
; Modifies: R0, R1
;
;Table 10-4 provides a GPIO Interrupt Configuration Example

; Registers have one bit (starting at LSB - same as other I/O such as LEDS) for each I/O pin
;
GPIO_IS_OFFSET  EQU 0x404	;GPIOIS - Interrupt Sense : 0 = edge, 1 = level interrupt
GPIO_IBE_OFFSET  EQU 0x408	;GPIOIBE - Interrupt Both Edges : 0 = single edge, 1 = both edges
GPIO_IEV_OFFSET  EQU 0x40c	;GPIOIEV - Interrupt Event : 0 = low level or falling edge, 1= high level or rising edge
GPIO_IM_OFFSET  EQU 0x410	;GPIOIM - Interrupt Mask : 0 = masked, 1 = unmasked
GPIO_RIS_OFFSET  EQU 0x414		; Raw Interrupt Status - READ ONLY
GPIO_MIS_OFFSET  EQU 0x418		; Masked Interrupt Status - READ ONLY
GPIO_ICR_OFFSET  EQU 0x41c		; Interrupt Clear - writing a 1 clears the RIS and MIS registers

Interrupt_Init
	STMFD		R13!,{R14}		; push the LR or return address

;Program the GPIOIS, GPIOIBE, GPIOEV, and GPIOIM registers to configure the type, event,
;and mask of the interrupts for each port.

;The default settings of 0x0 are for a single falling edge detection
; But reconfigure them just to make sure that things are the way we want them

;Note: To prevent false interrupts, the following steps should be taken when re-configuring
;GPIO edge and interrupt sense registers:

;a. Mask the corresponding port by clearing the IME field in the GPIOIM register.
	
	LDR R1, =GPIO_PORTF
    MOV R0, #0x00             ; 0 means mask or block interrupts
    STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

; b. Configure the IS field in the GPIOIS register the IBE field in the GPIOIBE register.

    MOV R0, #0x00             ; 0 means edge detecting interrupt
    STR R0, [R1, #GPIO_IS_OFFSET]	; 

; Configure the IEV field in the GPIOIEV register.
    MOV R0, #0x00             ; 0 means falling edge detecting
    STR R0, [R1, #GPIO_IEV_OFFSET]	; 

; Configure the IBE field in the GPIOIBE register.

    MOV R0, #0x00             ; 0 means single edge detection
    STR R0, [R1, #GPIO_IBE_OFFSET]	; 

;c. Clear the GPIORIS register using the ICR register to clear any pending interrupts.
; Set the appropiate bit to 1 to clear any pending interrupts
    MOV R0, #0x10             ; 0 means mask or block interrupts	; changed here
    STR R0, [R1, #GPIO_ICR_OFFSET]	; clear any interrupts recieved

;d. Unmask the port by setting the IME field in the GPIOIM register.
; Set the appropiate bit to 1 to enable interrupts for only the one switch required
    MOV R0, #0x10             ; 0 means mask or block interrupts				; changed here
	STR R0, [R1, #GPIO_IM_OFFSET]	; mask interrupts from happening

;Looking in the Startup.s file one will find an EXPORT of the address for interrupt handlers, one for each GPIO port

; Interrupt Enable Registers

; Finally - enable interrupts from Port F to get to the NVIC
; Set the correct bit in the EN? register to allow Port F interrupts

CORE_PERIPHERALS 		EQU 0xe000e000				

INTERRUPT_EN0_OFFSET 	EQU 0x100			; changed here

	MOV R0,# 0x40000000		; this enables GPIO Port F Interrupts  ; changed here

	LDR R1, =CORE_PERIPHERALS
	STR R0, [R1, #INTERRUPT_EN0_OFFSET]		; GPIO Interrupts require this enable

	LDMFD		R13!,{R15}		; pop the LR or return address and return
	

;
;  ---- Port Configuration  ----
;
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
PORT_B_MASK			EQU 0xFD		; PB5,4,1,0 are outputs %0011 0011
PORT_E_MASK			EQU 0x02		; PE5,4 are outputs %0011 0000
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
SYSCTL_RCGCGPIO_R  EQU 0x400FE608		; Register to enable clocks to the I/O port hardware

;------------Port_Init------------
; Initialize GPIO Port F for negative logic switches on PF0 and
; PF4 as the Launchpad is wired.  Weak internal pull-up
; resistors are enabled, and the NMI functionality on PF0 is
; disabled.  Make the RGB LED's pins outputs.
; Input: none
; Output: none
; Modifies: R0, R1, R2, R3
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
; These pins must be left as configured after reset:
;  PA[5:0] (UART0 and SSIO), PB[3:2] (I2C), PC[3:0] (JTAG)

; Initialize the I/O ports A, B, E, F via a common subroutine Port_Init_Individual
; Call Port_Init_Individual with the following paramaters passed:
; R1 is the base port address
; R2 is the output pin mask (which bits are outputs)
; R3 is the input pin mask  (which bits get configured as inputs)

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

MILISEC          	  EQU 534000      ; approximately 0.1 s delay at ~16 MHz clock

Delay
	LDR R0, =MILISEC   	       		; R0 = a value to get about a second delay
delay_loop
    SUBS R0, R0, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	delay_loop
	SUBS R8, R8, #1                 ; R0 = R0 - 1 (count = count - 1) and set N, Z, C status bits
	BNE	Delay

	; Note: For SUBs the "s" suffix means to set the status bits, without this the loops would not exit
	BX LR
	

DISPLAY_NUM
	STMFD R13!, {R2, R14}
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
	
    ALIGN                           ; make sure the end of this section is aligned
    END                             ; end of file
