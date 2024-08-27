stm8/

	#include "mapping.inc"
	#include "stm8s003f3.inc"

	segment 'rom'     
main.l
	; initialize SP
	ldw X,#stack_end
	ldw SP,X

	#ifdef RAM0	
	; clear RAM0
ram0_start.b EQU $ram0_segment_start
ram0_end.b EQU $ram0_segment_end
	ldw X,#ram0_start
clear_ram0.l
	clr (X)
	incw X
	cpw X,#ram0_end	
	jrule clear_ram0
	#endif

	#ifdef RAM1
	; clear RAM1
ram1_start.w EQU $ram1_segment_start
ram1_end.w EQU $ram1_segment_end	
	ldw X,#ram1_start
clear_ram1.l
	clr (X)
	incw X
	cpw X,#ram1_end	
	jrule clear_ram1
	#endif

	; clear stack
stack_start.w EQU $stack_segment_start
stack_end.w EQU $stack_segment_end
	ldw X,#stack_start
clear_stack.l
	clr (X)
	incw X
	cpw X,#stack_end	
	jrule clear_stack
;______________________________________
;  _   _ ___  ___ _ __   _ __ ___   __ _(_)_ __  
; | | | / __|/ _ \ '__| | '_ ` _ \ / _` | | '_ \ 
; | |_| \__ \  __/ |    | | | | | | (_| | | | | |
;  \__,_|___/\___|_|    |_| |_| |_|\__,_|_|_| |_|
  LD A, #$00
	PUSH A
	CALL clkSetHsiDivider
	POP A
	LD A, #$FF
	LD CLK_PCKENR1, A
	LD A, #$8A
	LD CLK_PCKENR2, A
	;--turn on quartz
	CALL clkSwitchToCrystal
	 
	;--PC6-output 
	LD A, #$D8; 3,4,6,7  
	LD PC_DDR, A
	LD PC_CR1, A
	LD PC_CR2, A
	;--PA3
	 	LD A, #$00;  
	LD PA_DDR, A
	LD PA_CR1, A
	LD PA_CR2, A
	;--D
		LD A, #$00;  
	LD PD_DDR, A
	LD PD_CR1, A
	LD PD_CR2, A
  

	 
	;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	 
	 
	 LDW X, #$000F	;@presc16,  
	 PUSHW X
	 LDW X, #$03FF	;@base16, -> base of counter, ARRL
	 PUSHW X
	 LDW X, #$000f;@comp16, -> capture compare CCR1 content
	 PUSHW X
	 LD A, #$60	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
	 PUSH A
	 LD A, #$00;@polarity8 -> (CCER)  $00 active HI,$02 active low
	 PUSH A
	 LD A ,#$00;@preload8  ->  $80 preload enable, CR1 content
	 PUSH A	 
   call tim1PwmCh2Setup
   ADDW SP, #$09
	 	 LDW X, #$000F	;@presc16,  
	 PUSHW X
	 LDW X, #$03FF	;@base16, -> base of counter, ARRL
	 PUSHW X
	 LDW X, #$000f;@comp16, -> capture compare CCR1 content
	 PUSHW X
	 LD A, #$60	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
	 PUSH A
	 LD A, #$00;@polarity8 -> (CCER)  $00 active HI,$02 active low
	 PUSH A
	 LD A ,#$00;@preload8  ->  $80 preload enable, CR1 content
	 PUSH A	 
   call tim1PwmCh3Setup
   ADDW SP, #$09
	 	 LDW X, #$000F	;@presc16,  
	 PUSHW X
	 LDW X, #$03FF	;@base16, -> base of counter, ARRL
	 PUSHW X
	 LDW X, #$000f;@comp16, -> capture compare CCR1 content
	 PUSHW X
	 LD A, #$60	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
	 PUSH A
	 LD A, #$00;@polarity8 -> (CCER)  $00 active HI,$02 active low
	 PUSH A
	 LD A ,#$00;@preload8  ->  $80 preload enable, CR1 content
	 PUSH A	 
   call tim1PwmCh4Setup
   ADDW SP, #$09
	 	 LDW X, #$000F	;@presc16,  
	 PUSHW X
	 LDW X, #$03FF	;@base16, -> base of counter, ARRL
	 PUSHW X
	 LDW X, #$000f;@comp16, -> capture compare CCR1 content
	 PUSHW X
	 LD A, #$60	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
	 PUSH A
	 LD A, #$00;@polarity8 -> (CCER)  $00 active HI,$02 active low
	 PUSH A
	 LD A ,#$00;@preload8  ->  $80 preload enable, CR1 content
	 PUSH A	 
   call tim1PwmCh1Setup
   ADDW SP, #$09
	  
	 ;----ADC
	LD A, #$06;@channel8, $00->AIN0, $01->AIN1, #$0F->AIN15
	PUSH A
	LD A, #$00;@prescaler, $00->/2, $10->/3, $20->/4, $
	PUSH A
  CALL adcSingleScanModeSetup
	ADDW SP, #$02
	;---interrupt enable
    LD A, #$01; UIE
	 LD TIM1_IER, A


infinite_loop.l
		
				wfi
	jra infinite_loop
     

	

;------------------------------	
;  _            _   
; | |_ ___  ___| |_ 
; | __/ _ \/ __| __|
; | ||  __/\__ \ |_ 
;  \__\___||___/\__|
;-------------------------------
	;--TEST PASSED!
		;===PROCEDURE 'tim1PwmCh1Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	; SP after RETURN +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim1PwmCh1Setup
		PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _006_v8a  $00 
	#define _006_prel $05
	#define _006_pol $06
	#define _006_mode $07
	#define _006_compH $08
	#define _006_compL $09
	#define _006_baseH $0A
	#define _006_baseL $0B
	#define _006_prescH $0C
	#define _006_prescL $0D
	;--disable timer
		;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_006_compH,SP)
	LD TIM1_CCR1H, A
	LD A, (_006_compL,SP)
	LD TIM1_CCR1L, A
		;--prescaler high byte firstly
	LD A, (_006_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_006_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_006_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_006_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_006_mode,SP)
	LD TIM1_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM1_CCER1
	AND A, #$F0
	LD (_006_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_006_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_006_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
	 BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_006_prel,SP)
	LD TIM1_CR1, A
  BSET TIM1_CR1, #$00
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	
	
	;--TEST PASSED!
  ;===PROCEDURE 'tim1PwmCh2Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
	                       ;$70 iPWM_MODE2
	                       ;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP after RETURN +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim1PwmCh2Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _007_v8a $00
	#define _007_prel $05
	#define _007_pol $06
	#define _007_mode $07
	#define _007_compH $08
	#define _007_compL $09
	#define _007_baseH $0A
	#define _007_baseL $0B
	#define _007_prescH $0C
	#define _007_prescL $0D
			;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_007_compH,SP)
	LD TIM1_CCR2H, A
	LD A, (_007_compL,SP)
	LD TIM1_CCR2L, A
		;--prescaler high byte firstly
	LD A, (_007_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_007_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_007_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_007_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR2
	LD A, (_007_mode,SP)
	LD TIM1_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER1
	AND A, #$0F
	LD (_007_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_007_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_007_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_007_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$00
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	
	;--TEST PASSED!	 
	;===PROCEDURE 'tim1PwmCh3Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
		      ;$70 iPWM_MODE2
		  ;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim1PwmCh3Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _008_v8a $00
	#define _008_prel $05
	#define _008_pol $06
	#define _008_mode $07
	#define _008_compH $08
	#define _008_compL $09
	#define _008_baseH $0A
	#define _008_baseL $0B
	#define _008_prescH $0C
	#define _008_prescL $0D
			;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_008_compH,SP)
	LD TIM1_CCR3H, A
	LD A, (_008_compL,SP)
	LD TIM1_CCR3L, A
		;--prescaler high byte firstly
	LD A, (_008_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_008_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_008_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_008_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_008_mode,SP)
	LD TIM1_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$F0
	LD (_008_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_008_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_008_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_008_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$00
	;--free memory
	ADDW SP, #$01
	POP A
	RET

	
	
	;--TEST PASSED!	
		;===PROCEDURE 'tim1PwmCh4Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim1PwmCh4Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _009_v8a $00
	#define _009_prel $05
	#define _009_pol $06
	#define _009_mode $07
	#define _009_compH $08
	#define _009_compL $09
	#define _009_baseH $0A
	#define _009_baseL $0B
	#define _009_prescH $0C
	#define _009_prescL $0D
			;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_009_compH,SP)
	LD TIM1_CCR4H, A
	LD A, (_009_compL,SP)
	LD TIM1_CCR4L, A
		;--prescaler high byte firstly
	LD A, (_009_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_009_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_009_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_009_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR
	LD A, (_009_mode,SP)
	LD TIM1_CCMR4, A
	;--polarity
; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$0F
	LD (_009_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_009_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A,  (_009_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	 BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_009_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	
startAdcSingleScan MACRO max_adc_ch
	LD A, ADC_CSR
	AND A, #$7F; clear EOC
	OR A, max_adc_ch
	LD ADC_CSR, A
	BSET ADC_CR1, #$00
	MEND
	
	;---STM8 ADC initialization library
;Author : Andrii Androsovych
	;====P R O C E D U R E===adcSingleModeSetup
	;@channel8, $00->AIN0, $01->AIN1, #$0F->AIN15
	;@prescaler, $00->/2, $10->/3, $20->/4, $30->/6, $40->/8, $50->/10
	;ADC DATA is right aligned!
	;SP+2
	;----------
	;stack frame
	;[A|RET|prescaler|channel]
adcSingleModeSetup
	#define _001_ch $05
	#define _001_presc $04
	;--store 
	PUSH A
	;--disable ADC
	BRES ADC_CR1, #$00
	;--load channel
	LD A, (_001_ch,SP)
	AND A, #$0F
	LD ADC_CSR, A
	;-result alignment- right
	BSET ADC_CR2, #$03; ALIGN bit
	;--prescaler
	LD A, (_001_presc,SP)
	AND A, #$F0
	OR A, #$01; ADON
	LD ADC_CR1, A
	;--restore
	POP A
	RET
 
	;====P R O C E D U R E===adcSingleScanModeSetup,
	;@channel8, $00->AIN0, $01->AIN1, #$0F->AIN15
	;@prescaler, $00->/2, $10->/3, $20->/4, $30->/6, $40->/8, $50->/10
	;ADC DATA is right aligned!
	;SP+2
	;----------
	;stack frame
	;[A|RET|prescaler|channel]
	;--NOTE: result stores in ADC_DBxRH, ADC_DBxRL
adcSingleScanModeSetup
	#define _001_ch $05
	#define _001_presc $04
	;--store 
	PUSH A
	;--disable ADC
	BRES ADC_CR1, #$00
	;--load channel
	LD A, (_001_ch,SP)
	AND A, #$0F
	LD ADC_CSR, A
	;- right alignment  scan
	BSET ADC_CR2, #$03; ALIGN bit
	BSET ADC_CR2, #$01; SCAN bit
	;--prescaler
	LD A, (_001_presc,SP)
	AND A, #$F0
	OR A, #$01; ADON
	LD ADC_CR1, A
	BSET ADC_CR1, #$00; start
	;--restore
	POP A
	RET
	
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
	
	
  
;------I  S  R----------

tim4Isr
  ;--clear flag
	BRES TIM4_SR, #$00
	BCPL PC_ODR, #$06
	IRET	
	
 
	
onTim1Update
	  ;--clear flag
	LD A, #$00
	LD TIM1_SR1, A
	;--load ADC To PWM register
	;;AIN 3, 4,5,6
	 LD A, ADC_DB3RH
   LD TIM1_CCR1H, A
	 LD A, ADC_DB3RL
   LD TIM1_CCR1L, A
	 ;--4
	 LD A, ADC_DB4RH
   LD TIM1_CCR2H, A
	 LD A, ADC_DB4RL
   LD TIM1_CCR2L, A
	;--5
	 LD A, ADC_DB5RH
   LD TIM1_CCR3H, A
	 LD A, ADC_DB5RL
   LD TIM1_CCR3L, A
	 ;--6
	 LD A, ADC_DB6RH
   LD TIM1_CCR4H, A
	 LD A, ADC_DB6RL
   LD TIM1_CCR4L, A
		;--start ADC
  startAdcSingleScan #$06
	
	IRET

	interrupt NonHandledInterrupt
NonHandledInterrupt.l
	iret
	segment 'vectit'
	dc.l {$82000000+main}									; reset
	dc.l {$82000000+NonHandledInterrupt}	; trap
	dc.l {$82000000+NonHandledInterrupt}	; irq0
	dc.l {$82000000+NonHandledInterrupt}	; irq1
	dc.l {$82000000+NonHandledInterrupt}	; irq2
	dc.l {$82000000+NonHandledInterrupt}	; irq3
	dc.l {$82000000+NonHandledInterrupt}	; irq4
	dc.l {$82000000+NonHandledInterrupt}	; irq5
	dc.l {$82000000+NonHandledInterrupt}	; irq6
	dc.l {$82000000+NonHandledInterrupt}	; irq7
	dc.l {$82000000+NonHandledInterrupt}	; irq8
	dc.l {$82000000+NonHandledInterrupt}	; irq9
	dc.l {$82000000+NonHandledInterrupt}	; irq10
	dc.l {$82000000+onTim1Update}	; irq11
	dc.l {$82000000+NonHandledInterrupt}	; irq12
	dc.l {$82000000+NonHandledInterrupt}	; irq13
	dc.l {$82000000+NonHandledInterrupt}	; irq14
	dc.l {$82000000+NonHandledInterrupt}	; irq15
	dc.l {$82000000+NonHandledInterrupt}	; irq16
	dc.l {$82000000+NonHandledInterrupt}	; irq17
	dc.l {$82000000+NonHandledInterrupt}	; irq18
	dc.l {$82000000+NonHandledInterrupt}	; irq19
	dc.l {$82000000+NonHandledInterrupt}	; irq20
	dc.l {$82000000+NonHandledInterrupt}	; irq21
	dc.l {$82000000+NonHandledInterrupt}	; irq22
	dc.l {$82000000+tim4Isr}	; irq23
	dc.l {$82000000+NonHandledInterrupt}	; irq24
	dc.l {$82000000+NonHandledInterrupt}	; irq25
	dc.l {$82000000+NonHandledInterrupt}	; irq26
	dc.l {$82000000+NonHandledInterrupt}	; irq27
	dc.l {$82000000+NonHandledInterrupt}	; irq28
	dc.l {$82000000+NonHandledInterrupt}	; irq29

	end
