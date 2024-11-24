.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb

.equ RCC_BASE,		(0x40023800)
.equ RCC_AHB1ENR,	(RCC_BASE + 0x30)
.equ RCC_AHB1ENR_GPIOAEN,	(1 << 0)

.equ GPIOA_BASE,	(0x40020000)
.equ GPIOB_BASE,	(0x40020400)
.equ GPIOA_MODER,	(GPIOA_BASE + 0x00)
.equ GPIO_MODER_6_MASK,		(0b11 << 12)
.equ GPIO_MODER_6_OUTPUT,	(0b01 << 12)
.equ GPIOA_BSRR,	(GPIOA_BASE + 0x18)


.global main
main:
	@ Enable clock to GPIOA and GPIOB
	ldr		r0, =RCC_AHB1ENR
	ldr		r1, [r0]
	orr		r1, RCC_AHB1ENR_GPIOAEN | (1 << 1)  @ RCC_AHB1ENR_GPIOBEN
	str		r1, [r0]

	@ Set GPIOA5, GPIOA6, GPIOA7 as output
	ldr		r0, =GPIOA_MODER
	ldr		r1, [r0]
	and		r1, ~(0b11 << 10 | 0b11 << 12 | 0b11 << 14)  @ Clear mode bits for PA5, PA6, PA7
	orr		r1, (0b01 << 10 | 0b01 << 12 | 0b01 << 14)   @ Set mode to output for PA5, PA6, PA7
	str		r1, [r0]

	@ Set GPIOB6 as output
	ldr		r0, =GPIOB_BASE + 0x00  @ GPIOB_MODER
	ldr		r1, [r0]
	and		r1, ~(0b11 << 12)  @ Clear mode bits for PB6
	orr		r1, (0b01 << 12)   @ Set mode to output for PB6
	str		r1, [r0]

	loop:
		@ Turn off all LEDs
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (5 + 16) | 1 << (6 + 16) | 1 << (7 + 16))
		str		r1, [r0]

		ldr		r0, =GPIOB_BASE + 0x18  @ GPIOB_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (6 + 16))
		str		r1, [r0]

		@ Turn on GPIOA5 and GPIOA6
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << 5 | 1 << 6)
		str		r1, [r0]

		@ Delay of 500ms
		ldr		r0, =505000
		bl		delay

		@ Turn off GPIOA5, keep GPIOA6 on
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (5 + 16))
		str		r1, [r0]

		@ Turn on GPIOA7
		orr		r1, (1 << 7)
		str		r1, [r0]

		@ Delay of 500ms
		ldr		r0, =505000
		bl		delay

		@ Turn off GPIOA6, keep GPIOA7 on
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (6 + 16))
		str		r1, [r0]

		@ Turn on GPIOB6
		ldr		r0, =GPIOB_BASE + 0x18  @ GPIOB_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << 6)
		str		r1, [r0]

		@ Delay of 500ms
		ldr		r0, =505000
		bl		delay

		@ Turn off GPIOA7, keep GPIOB6 on
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (7 + 16))
		str		r1, [r0]

		@ Turn on GPIOA5
		orr		r1, (1 << 5)
		str		r1, [r0]

		@ Delay of 500ms
		ldr		r0, =505000
		bl		delay

		@ Turn off all LEDs
		ldr		r0, =GPIOA_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (5 + 16) | 1 << (6 + 16) | 1 << (7 + 16))
		str		r1, [r0]

		ldr		r0, =GPIOB_BASE + 0x18  @ GPIOB_BSRR
		ldr		r1, [r0]
		orr		r1, (1 << (6 + 16))
		str		r1, [r0]

		b	loop

delay:
delay_loop:
	subs	r0, #1
	bne		delay_loop
	bx		lr
