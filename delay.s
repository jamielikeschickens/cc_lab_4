.syntax unified
.thumb
.global Delay_Asm

.extern TIM2_CNT
.equ THRESHOLD, 0x8000
.equ DELAY_THRESHOLD, 0xEFFF
	
Delay_Asm:
	LDR r0, =TIM2_CNT
	LDR r1, = 0xAFFF
	MOV r2, #0
	LDR r5, =DELAY_THRESHOLD

	B Delay_Loop

Delay_Loop:
	CMP r1, r2
	BX lr

	LDR r3, [r0]
	CMP r3, r5
	BGT Delay_Increment

	B Delay_Loop

Delay_Increment:
	SUB r1, r1, #1
	B Delay_Loop
