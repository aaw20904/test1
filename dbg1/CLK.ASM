;---clock library for STM8
;--Author: Andrii Androsovych
	;====P R O C E D U R E===turn on clk bus
	;@peripherial8
	;TIM1-$80,TIM3-$40,TIM2/5-$20,TIM4/6-$10,UART-see datasheet,
	;SPI-$2,I2C-1
	;STACK after return +1
clkBusPeripherial1
	PUSH A
	LD A, ($04,SP)
	LD CLK_PCKENR1, A
	POP A
	RET
	;====P R O C E D U R E===turn on clk bus
	;@peripherial8
	;CAN-$80, ADC-$08, AWU-$04
	;STACK after return +1
clkBusPeripherial2
	PUSH A
	LD A, ($04,SP)
	LD CLK_PCKENR2, A
	POP A
	RET	
	;===P R O C E D U R E=switch to Crystal
	;--NO PARAMS
	;STACK after return 0
clkSwitchToCrystal
	PUSH A
	;--tuurn on HSE oscillator
	LD A, HSEEN
	LD CLK_ECKR, A
clkSwitchToCrystal_hsi_rdy
		;--wait until crystal oscillator ready (HSERDY)
	BTJF CLK_ECKR, #$01, clkSwitchToCrystal_hsi_rdy
	;---Enable the switching mechanism
	LD A, CLK_SWCR
	OR A, SWEN
	LD CLK_SWCR, A
	;---select source clock
	;0xE1: HSI selected as master clock source (reset value)
	;0xD2: LSI selected as master clock source (only if LSI_EN
	;option bit is set)
	;0xB4: HSE selected as master clock source
	LD A, #$B4
	LD CLK_SWR, A
	NOP
	NOP
	NOP
	NOP
	NOP
	POP A
	RET
	
	
	;==P R O C E D U R E=="set HSI divider"
	;--@ char divider
	;STACK after return +1
clkSetHsiDivider
	;--store registers A,X,Y,CC (1+2+2+1=6Bytes)
	PUSH A
	;-read default value 
	LD A, CLK_CKDIVR
	;---clear all the hsi divider bits
	AND A, #$E7
	;--1st paprameter has offset 9 bytes
	; because A,X,Y,CC,SP has ben stored later 
	OR A, ($04,SP)
	;---update CLK_CKDIVR
	LD CLK_CKDIVR, A
	;--restore registers
	POP A
	RET
	
	;======P R O C e D U R E==="set CPU divider"
	;@ char divider 
	;STACK after return +1
clkSetCpuDivider
		;--store registers A,X,Y,CC (1+2+2+1=6Bytes)
	PUSH A
	;-read default value 
	LD A, CLK_CKDIVR
	;---clear all the hsi divider bits
	AND A, #$f8
	;--1st paprameter has offset 9 bytes
	; because A,X,Y,CC,SP has ben stored later
	OR A, ($04,SP)
	;---update CLK_CKDIVR
	LD CLK_CKDIVR, A
		;--restore registers
	POP A
	RET	
	