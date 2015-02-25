.section .text	
		
@ #defines
@ Clock enable register for the I/O Bus
.equ RCC_AHBENR, 0x4002381c
@ In/Out mode selection for GPIO ports
.equ GPIOA_MODER, 0x40020000
.equ GPIOB_MODER, 0x40020400
@ The data output register for GPIOB
.equ GPIOB_ODR, 0x40020414
@ The data input register for GPIOA
.equ GPIOA_IDR, 0x40020010
	
@ Clock enables for Timers2-7
.equ RCC_APB1ENR, 0x40023824

@ Timer2 control
.equ TIM2_CR1, 0x40000000
@ Timer2 pre-scaler
.equ TIM2_PSC, 0x40000028
@ The Timer2 count
.equ TIM2_CNT, 0x40000024
	
@ EXTI0 interrupt constants
.equ EXTI0_BASE, 0x40010400
.equ SYSCFG_EXTICR1, 0x40010008
.equ EXTI_IMR_offset, 0 @ (Interrupt mask)
.equ EXTI_RTSR_offset, 0x08 @(Rising trigger)
.equ EXTI_FTSR_offset, 0x0c @(falling trigger)
	
	@ make label globally visible for the linker
.global SystemInit
.global GPIOA_IDR
.global GPIOB_ODR
.global TIM2_CNT
.global Mode_Switch
		
.extern main
		
	@ This will be ThumbV2 code (since need to use thumbV2 code to access MSR register)
.thumb

SystemInit:
	@ intialise the LED pins to support flashing the LEDs
	@ Blue LED is on pin PB6@ Green LED is on pin PB7
	@ User switch is on pin PA0
	
	@ Enable GPIOA and GPIOB clocks
	LDR r0, =RCC_AHBENR
	MOV r1, #3
	STR r1, [r0]
	
	@ Configure GPIOA as input
	LDR r0, =GPIOA_MODER
	LDR r1, =0xA8000000
	STR r1, [r0]
	
	@ Configure GPIOB as output
	LDR r0, =GPIOB_MODER
	LDR r1, =0x55555780
	STR r1, [r0]
	
	@ Light only green LED
	LDR r0, =GPIOB_ODR
	LDR r1, =0x80
	STR r1, [r0]
	
	@ Set timers clock on
	LDR r0, =RCC_APB1ENR
	LDR r1, =0x3f
	STR r1, [r0]
	
	@ Set the pre-scaler to slow down the counter
	LDR r0, =TIM2_PSC
	MOV r1, #0x0f
	STR r1, [r0]
	
	@ Set Timer2 running
	LDR r0, =TIM2_CR1
	MOV r1, #0x01
	STR r1, [r0]
	
	@ Wait with blue LED off until
	@ User button is pressed
	@ Then light blue LED and continue
Button_Wait:
	LDR r2, =GPIOA_IDR
	LDR r1, [r2]
	MOV r3, #0x01
	AND r1, r1, r3
	BEQ Button_Wait
	
	@ Light both LEDs now
	LDR r0, =GPIOB_ODR
	LDR r1, =0xc0
	STR r1, [r0]
	B Setup_EXTI0

Setup_EXTI0:
	@ Configure the PA0 pin to cause an interrupt on EXTI0
	LDR r0, =SYSCFG_EXTICR1
	MOV r1, #0x00 @ set EXTI0 to come from PA0
	STR r1, [r0] 
	LDR r0, =EXTI0_BASE
	MOV r1, #0x01
	STR r1, [r0]
	STR r1, [r0, #EXTI_RTSR_offset] @ trigger on rising edge only
@	STR r1, [r0, #EXTI_FTSR_offset] @ trigger on falling edge


Init_End:
	BL __libc_init_array
	LDR r0, =main
	BX r0

	@ Return point is reached only after Lab code is completed
	BX lr 

Mode_Switch:
	MOV r0, #42
	PUSH {r0} @ push for example later
	MOV r0, #0x3 @ set stack to PSP and processor mode to unprivileged
	MSR CONTROL, r0  @ do it
	ISB              @ wait for it to be done
	LDR r0, =0x20002000  @ PSP stack away from MSP
	MOV sp, r0
	
.align
.end
