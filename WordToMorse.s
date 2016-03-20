;*----------------------------------------------------------------------------
;* Name:    Lab_2_program.s 
;* Purpose: This code template is for Lab 2 on the Tiva Board
;* Author: Eric Praetzel and Rasoul Keshavarzi 
;*----------------------------------------------------------------------------*/


GPIO_PORTF_DATA_R  EQU 0x400253FC
GPIO_PORTF_DIR_R   EQU 0x40025400
GPIO_PORTF_AFSEL_R EQU 0x40025420
GPIO_PORTF_PUR_R   EQU 0x40025510
GPIO_PORTF_DEN_R   EQU 0x4002551C
GPIO_PORTF_LOCK_R  EQU 0x40025520
GPIO_PORTF_CR_R    EQU 0x40025524
GPIO_PORTF_AMSEL_R EQU 0x40025528
GPIO_PORTF_PCTL_R  EQU 0x4002552C
GPIO_LOCK_KEY      EQU 0x4C4F434B  ; Unlocks the GPIO_CR register
RED       EQU 0x02
BLUE      EQU 0x04
GREEN     EQU 0x08
SW1       EQU 0x10                 ; on the left side of the Launchpad board
SW2       EQU 0x01                 ; on the right side of the Launchpad board
SYSCTL_RCGCGPIO_R  EQU   0x400FE608


        AREA    |.text|, CODE, READONLY, ALIGN=2
        THUMB
        EXPORT  Start

Start
;
; Turn off all LEDs	
    BL  PortF_Init                  	; initialize input and output pins of Port F
	MOV R3, #0							; load in the value to turn the RED led OFF
	LDR R4, =GPIO_PORTF_DATA_R 			; pointer to Port F data register
	STR R3, [R4]						; write data to Port F to turn lights on and off

ResetLUT
		LDR         R5, =InputLUT       ; loads the address of InputLUT into R5

NextChar
        LDRB        R0, [R5]			; Read one byte from the inputLut and load into R0 (LDR Byte)
       	ADD         R5, #1              ; point to next value for number of delays, jump by 1 byte
		TEQ         R0, #0              ; If we hit 0 (null at end of the string) then reset to the start of lookup table
		BNE			ProcessChar			; If we have a character process it

		MOV			R0, #4				; delay 4 extra spaces (7 total) between words
		BL			DELAY				; branch with link to delay
		BEQ         ResetLUT			; branch to ResetLut if flag is zero

ProcessChar	BL		CHAR2MORSE		; convert ASCII to Morse pattern in R1		

EndZero
		MOV			R6, #0x8000		; Init R6 with the value for the bit, 15th, which we wish to test
		LSL			R1, #1			; shift R1 left by 1, store in R1
		SUBS		R9, #1			; decrement character count by 1
		ANDS		R7, R1, R6		; R7 gets (R1 AND R6), Zero bit gets set telling us if the bit is 0 or 1
		BEQ			EndZero 		; branch TO EndZero if equal to zero
		BNE			ConvertToMorse	; branch to ConvertToMorse if it's not zero
ConvertToMorse
		ANDS		R7, R1, R6		; compare the first bit and update the flag
		BLEQ		LED_OFF			; branch link to LED_off if flag is zero, i.e. the first bit is zero
		BLNE		LED_ON			; branch link to LED_ON if first bit is 1
		MOV			R0, #1 			; set the multiple delay count as 1
		BL			DELAY 			; do the delay
		LSL			R1, R1, #1 		; shift left by 1
		SUBS		R9, #1 			; substract the character counter by 1
		ANDS		R7, R1, R6		; compare current first bit
		TEQ			R9, #0 			; check if the character counter reaches 0
		BNE			ConvertToMorse	; if not zero, loop back to ConvertToMorse
		BL			LED_OFF 		; turn off LED, letter is processed
		MOV			R0, #3 			; set multiple delay count as 3 (between letters)(3 dots of led off)
		
		BL			DELAY
		B 			NextChar  		; This is the end of the main program 

; Subroutines
;
;			convert ASCII character to Morse pattern
;			pass ASCII character in R0, output in R1
;			index into MorseLuT must be by steps of 2 bytes
CHAR2MORSE	STMFD		R13!,{R14}	; push Link Register (return address) on stack

		SUB			R0, #0x41 		; substract R0 by 0x41, so that it acts as an offset
		ADD			R0, R0 			; convert from half-word to word since we can only query words from LUT
		LDR			R8, =MorseLUT 	; get the address of MorseLUT and store it into R8
		LDRH		R1, [R8, R0]	; load the value from R8 with an offset by R0 and store it into R1
		MOV			R9, #0x10		; set up the character counter
		
		LDMFD		R13!,{R15}		; restore LR to R15 the Program Counter to return


; Turn the LED on, but deal with the stack in a simpler way
; NOTE: This method of returning from subroutine (BX  LR) does NOT work if subroutines are nested!!
;
LED_ON 	push 		{r3-r4}			; preserve R3 and R4 on the R13 stack

		MOV 		R3, #RED
		LDR 		R4, =GPIO_PORTF_DATA_R 		; pointer to Port F data register
		STR 		R3, [R4]					; write data to Port F to turn lights on and off

		pop 		{r3-r4}
		BX 			LR							; branch to the address in the Link Register.  Ie return to the caller

; Turn the LED off, but deal with the stack in the proper way
; the Link register gets pushed onto the stack so that subroutines can be nested
;
LED_OFF	STMFD		R13!,{R3, R14}	; push R3 and Link Register (return address) on stack
		MOV 		R3, #0
		LDR 		R4, =GPIO_PORTF_DATA_R 		; pointer to Port F data register
		STR 		R3, [R4]					; write data to Port F to turn lights on and off
		LDMFD		R13!,{R3, R15}				; restore R3 and LR to R15 the Program Counter to return

;	Delay 500ms * R0 times
;	Use the delay loop from Lab-1 but loop R0 times around
;
DELAY			STMFD		R13!,{R2, R14}
MultipleDelay		TEQ		R0, #0			; test R0 to see if it's 0 - set Zero flag so you can use BEQ, BNE

			MOVT		R10, #0X002C		;load R10 with 0x002C (delay time)(500 ms)
loop
			SUBS 		R10, #1 			;Decrement r10 and set N,Z,V,C status bits
			BNE 		loop				;once the counter reaches 0, exit this loop
			SUBS		R0, #1				;decrement the multiple delay counter
			TEQ			R0, #0				;check to see if the multiple delay counter if 0
			BNE			MultipleDelay		;if it is not equal 0, go back to MultipleDelay
			

exitDelay		LDMFD		R13!,{R2, R15}


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

    ALIGN                           ; make sure the end of this section is aligned

;
; Data used in the program
; DCB is Define Constant Byte size
; DCW is Define Constant Word (16-bit) size
; EQU is EQUate or assign a value.  This takes no memory but instead of typing the same address in many places one can just use an EQU
;
		ALIGN				; make sure things fall on word addresses

; One way to provide a data to convert to Morse code is to use a string in memory.
; Simply read bytes of the string until the NULL or "0" is hit.  This makes it very easy to loop until done.
;
InputLUT	DCB		"SCARZ", 0	; strings must be stored, and read, as BYTES,and puts a zero at the end

		ALIGN				; make sure things fall on word addresses
MorseLUT 
		DCW 	0x17, 0x1D5, 0x75D, 0x75 	; A, B, C, D
		DCW 	0x1, 0x15D, 0x1DD, 0x55 	; E, F, G, H
		DCW 	0x5, 0x1777, 0x1D7, 0x175 	; I, J, K, L
		DCW 	0x77, 0x1D, 0x777, 0x5DD 	; M, N, O, P
		DCW 	0x1DD7, 0x5D, 0x15, 0x7 	; Q, R, S, T
		DCW 	0x57, 0x157, 0x177, 0x757 	; U, V, W, X
		DCW 	0x1D77, 0x775 			; Y, Z

    END                             ; end of file
