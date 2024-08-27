;STM8 assembler Timer library
;Author: Andrii Androsowych

;  _____ ___ __  __ _ 
; |_   _|_ _|  \/  / |
;   | |  | || |\/| | |
;   | |  | || |  | | |
;   |_| |___|_|  |_|_|
;                     
;=====P R O C E D U R E=='tim1BaseModeSetup'=
;===initializing TIM1 in base mode
; @prescaler16, @base16, @mode8
; After return: SP-5
;-modes can (TIM1_CR1) be ;
;-TIM_EDGE_ALIGNED ($00)
;- TIM_CENTER_ALINGN_1 ($20),
;- TIM_CENTER_ALINGN_2 ($40),
;- TIM_CENTER_ALINGN_3 ($60),
;count mode can be:
;- TIM_COUNTS_UP ($00)
;- TIM_COUNTS_DOWN ($10)
;--stack frame:
;   0   1  2 3  4    5  6  7  8 
 ;[| A |  RET |mode| base | presc| 

tim1BaseModeSetup
		;--disable timer
	BRES TIM1_CR1,#$00
	;--store registers
	PUSH A
	#define _000_presH $07 ;prescaler
	#define _000_presL $08
	#define _000_baseH $05 ;base ,counter
	#define _000_baseL $06
	#define _000_mode $04  ; parameters
	;load prescaler HI firstly:
	LD A,(_000_presH,SP)
	LD TIM1_PSCRH, A
	LD A,(_000_presL,SP)
	LD TIM1_PSCRL, A
	;load base, HI first:
	LD A, (_000_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_000_baseL,SP)
	LD TIM1_ARRL, A
  ;load mod in TIM1_CR1 (p.188 ref manual)
  LD A, (_000_mode,SP)
  LD TIM1_CR1, A
	;--start timer and GO! 
	LD A, TIM1_CR1
	OR A, #$01
	LD TIM1_CR1, A
	;--restore registers
	POP A
	RET
	
;=====P R O C E D U R E=='tim1BaseModeSetupIT'=
;===initializing TIM1 in base mode with interrupts
; @prescaler16, @base16, @mode8
; After return: SP-5
;-modes can (TIM1_CR1) be ;
;-TIM_CR1_EDGE_ALIGNED ($00)
;- TIM_CR1_CENTER_ALINGN_1 ($20),
;- TIM_CR1_CENTER_ALINGN_2 ($40),
;- TIM_CR1_CENTER_ALINGN_3 ($60),
;count mode can be:
;- TIM_CR1_COUNTS_UP ($00)
;- TIM_CR1_COUNTS_DOWN ($10)
;   0   1  2 3  4    5  6  7  8 
 ;[| A |  RET |mode| base | presc| 

tim1BaseModeSetupIT
	;--store registers
	PUSH A
	#define _001_presH $07;prescaler
	#define _001_presL $08
	#define _001_baseH $05 ;base ,counter
	#define _001_baseL $06
	#define _001_mode $04  ; parameters
	;load prescaler HI firstly:
	LD A,(_001_presH,SP)
	LD TIM1_PSCRH, A
	LD A,(_001_presL,SP)
	LD TIM1_PSCRL, A
	;load base, HI first:
	LD A, (_001_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_001_baseL,SP)
	LD TIM1_ARRL, A
	;--write interrupts (p.194):
	LD A, #$01
	LD TIM1_IER, A
	;--start timer and GO! 
	LD A, TIM1_CR1
	OR A, #$01
	LD TIM1_CR1, A
	;--restore registers
	POP A
	RET

  ;--TEST PASSED!
		;===PROCEDURE 'tim1OutputCompareCh1Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$40 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	; SP after RETURN +9
	;stack frame:
	;  0  1  2 3   4   5    6  7 8  9 A  A C
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OutputCompareCh1Setup
		PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _002_v8a  $00 
	#define _002_prel $05
	#define _002_pol $06
	#define _002_mode $07
	#define _002_compH $08
	#define _002_compL $09
	#define _002_baseH $0A
	#define _002_baseL $0B
	#define _002_prescH $0C
	#define _002_prescL $0D
	;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_002_compH,SP)
	LD TIM1_CCR1H, A
	LD A, (_002_compL,SP)
	LD TIM1_CCR1L, A
		;--prescaler high byte firstly
	LD A, (_002_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_002_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_002_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_002_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_002_mode,SP)
	LD TIM1_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM1_CCER1
	AND A, #$F0
	LD (_002_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_002_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_002_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
  BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_002_prel,SP)
	LD TIM1_CR1, A
	OR A, #$01;
	LD TIM1_CR1, A
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	;--TEST PASSED!
		;===PROCEDURE 'tim1OutputCompareCh2Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	; SP after RETURN +9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OutputCompareCh2Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _003_v8a $00
	#define _003_prel $05
	#define _003_pol $06
	#define _003_mode $07
	#define _003_compH $08
	#define _003_compL $09
	#define _003_baseH $0A
	#define _003_baseL $0B
	#define _003_prescH $0C
	#define _003_prescL $0D
		;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_003_compH,SP)
	LD TIM1_CCR2H, A
	LD A, (_003_compL,SP)
	LD TIM1_CCR2L, A
		;--prescaler high byte firstly
	LD A, (_003_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_003_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_003_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_003_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR2
	LD A, (_003_mode,SP)
	LD TIM1_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER1
	AND A, #$0F
	LD (_003_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_003_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_003_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
	 BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_003_prel,SP)
	LD TIM1_CR1, A
	OR A, #$01;
	LD TIM1_CR1, A
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	;--TEST PASSED!
			;===PROCEDURE 'tim1OutputCompareCh3Setup'
	;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	; SP after RETURN +9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OutputCompareCh3Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _004_v8a $00
	#define _004_prel $05
	#define _004_pol $06
	#define _004_mode $07
	#define _004_compH $08
	#define _004_compL $09
	#define _004_baseH $0A
	#define _004_baseL $0B
	#define _004_prescH $0C
	#define _004_prescL $0D
		;--disable timer
		;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_004_compH,SP)
	LD TIM1_CCR3H, A
	LD A, (_004_compL,SP)
	LD TIM1_CCR3L, A
		;--prescaler high byte firstly
	LD A, (_004_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_004_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_004_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_004_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_004_mode,SP)
	LD TIM1_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$F0
	LD (_004_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_004_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_004_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	 BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_004_prel,SP)
	LD TIM1_CR1, A
	OR A, #$01;
	LD TIM1_CR1, A
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	;--TEST PASSED!
		;===PROCEDURE 'tim1OutputCompareCh4Setup'
	;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	; SP after RETURN +9
	;stack frame:
	;[v16a|A|return|prel|pol|mode|comp|base|presc]
tim1OutputCompareCh4Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _005_v8a $00
	#define _005_prel $05
	#define _005_pol $06
	#define _005_mode $07
	#define _005_compH $08
	#define _005_compL $09
	#define _005_baseH $0A
	#define _005_baseL $0B
	#define _005_prescH $0C
	#define _005_prescL $0D
			;--disable timer
	BRES TIM1_CR1,#$00
	;--load comparand, Hi firstly
	LD A, (_005_compH,SP)
	LD TIM1_CCR4H, A
	LD A, (_005_compL,SP)
	LD TIM1_CCR4L, A
		;--prescaler high byte firstly
	LD A, (_005_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_005_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_005_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_005_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR
	LD A, (_005_mode,SP)
	LD TIM1_CCMR4, A
	;--polarity
; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$0F
	LD (_005_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_005_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A,  (_005_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	 BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_005_prel,SP)
	LD TIM1_CR1, A
	OR A, #$01;
	LD TIM1_CR1, A
	;--free mem
	ADDW SP, #$01
	POP A
	RET
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
	
	;--TEST PASSED!
	;=P R O C E D UR E======tim1OnePulseModeStart=======
	;start one pulse mode
	;SP afer RETURN nas not been changed 
tim1OnePulseModeStart
	;///ONE PULSE MODE + Go
	BSET TIM1_CR1, #$08; ONE PULSE MODE
	BSET TIM1_CR1, #$01; START
	RET
		;--TEST PASSED!
			;===PROCEDURE 'tim1OnePulsePwmCh1Setup'
	;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;after return: SP+9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OnePulsePwmCh1Setup
	PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _010_v8a  $00 
	#define _010_prel $05
	#define _010_pol $06
	#define _010_mode $07
	#define _010_compH $08
	#define _010_compL $09
	#define _010_baseH $0A
	#define _010_baseL $0B
	#define _010_prescH $0C
	#define _010_prescL $0D
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_010_compH,SP)
	LD TIM1_CCR1H, A
	LD A, (_010_compL,SP)
	LD TIM1_CCR1L, A
		;--prescaler high byte firstly
	LD A, (_010_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_010_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_010_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_010_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_010_mode,SP)
	LD TIM1_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM1_CCER1
	AND A, #$F0
	LD (_010_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_010_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_010_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_010_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$03;//one pulse mode ,stopped
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	
	;--TEST PASSED!
		;===PROCEDURE 'tim1OnePulsePwmCh2Setup'
		;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
		;after return: SP+9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OnePulsePwmCh2Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _011_v8a $00
	#define _011_prel $05
	#define _011_pol $06
	#define _011_mode $07
	#define _011_compH $08
	#define _011_compL $09
	#define _011_baseH $0A
	#define _011_baseL $0B
	#define _011_prescH $0C
	#define _011_prescL $0D
		;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_011_compH,SP)
	LD TIM1_CCR2H, A
	LD A, (_011_compL,SP)
	LD TIM1_CCR2L, A
		;--prescaler high byte firstly
	LD A, (_011_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_011_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_011_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_011_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR2
	LD A, (_011_mode,SP)
	LD TIM1_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER1
	AND A, #$0F
	LD (_011_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_011_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_011_v8a,SP)
	LD TIM1_CCER1, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_011_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$03;//one pulse mode ,stopped   
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	
	;--TEST PASSED!
	;===PROCEDURE 'tim1OnePulsePwmCh3Setup'
	;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
		;after return: SP+9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OnePulsePwmCh3Setup
			PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _012_v8a $00
	#define _012_prel $05
	#define _012_pol $06
	#define _012_mode $07
	#define _012_compH $08
	#define _012_compL $09
	#define _012_baseH $0A
	#define _012_baseL $0B
	#define _012_prescH $0C
	#define _012_prescL $0D
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_012_compH,SP)
	LD TIM1_CCR3H, A
	LD A, (_012_compL,SP)
	LD TIM1_CCR3L, A
		;--prescaler high byte firstly
	LD A, (_012_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_012_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_012_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_012_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_012_mode,SP)
	LD TIM1_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$F0
	LD (_012_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_012_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_012_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_012_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$03;//one pulse mode ,stopped
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	
	;--TEST PASSED!
	;===PROCEDURE 'tim1OnePulsePwmCh4Setup'
	;timer is stopped!To start, call - 'tim1OnePulseModeStart'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
		;after return: SP+9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1OnePulsePwmCh4Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _013_v8a $00
	#define _013_prel $05
	#define _013_pol $06
	#define _013_mode $07
	#define _013_compH $08
	#define _013_compL $09
	#define _013_baseH $0A
	#define _013_baseL $0B
	#define _013_prescH $0C
	#define _013_prescL $0D
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_013_compH,SP)
	LD TIM1_CCR4H, A
	LD A, (_013_compL,SP)
	LD TIM1_CCR4L, A
		;--prescaler high byte firstly
	LD A, (_013_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_013_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_013_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_013_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR
	LD A, (_013_mode,SP)
	LD TIM1_CCMR4, A
	;--polarity
; 1)store content of another channels
	LD A, TIM1_CCER2
	AND A, #$0F
	LD (_013_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_013_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A,  (_013_v8a,SP)
	LD TIM1_CCER2, A
	;--turn on main channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_013_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$03;//one pulse mode ,stopped
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	
	
  ;---TEST PASSED!
			;===PROCEDURE 'tim1ComplementaryPwmCh1Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, -> pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  PosCh:$02 active LOW,
                       ;	NegCh: $08 active LOW
	;@preload8  -> (CR1) $80 preload , $20-center align.mode 1
	;@deadtime8 -> TIM1_DTR content
	; SP +10 after ret
	;stack frame:
	;[v8a|A|return|deadtime|prel|pol|mode|comp|base|presc]
tim1ComplementaryPwmCh1Setup
		PUSH A
	;-variables
	#define _014_v8a $00
	#define _014_deadtime $05
	#define _014_prel $06
	#define _014_pol $07
	#define _014_mode $08
	#define _014_compH $09
	#define _014_compL $0a
	#define _014_baseH $0b
	#define _014_baseL $0c
	#define _014_prescH $0d
	#define _014_prescL $0e
	;;allocate memory
	SUBW SP, #$01
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_014_compH,SP)
	LD TIM1_CCR1H, A
	LD A, (_014_compL,SP)
	LD TIM1_CCR1L, A
		;--prescaler high byte firstly
	LD A, (_014_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_014_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_014_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_014_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_014_mode,SP)
	LD TIM1_CCMR1, A
	;--polarity
	;1)store an another channel CCER1
	LD A, TIM1_CCER1
	AND A, #$F0
	LD (_014_v8a,SP), A
	;--2)apply saved another channel
	LD A, (_014_pol,SP)
	OR A, #$05; Pos and Neg channels ON
	OR A, (_014_v8a,SP)
	LD TIM1_CCER1, A
	;--deadtime
	LD A, (_014_deadtime,SP)
	LD TIM1_DTR, A
	;--turn on channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_014_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$00;//start
	;--free memory
	ADDW SP, #$01
	POP A
	RET 

	;---TEST PASSED!
	  ;===PROCEDURE 'tim1ComplementaryPwmCh2Setup'
  ;@presc16,  
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR) : $60 PWM_MODE1
							       ;$70 iPWM_MODE2
						;additional:  $04 fast, $08 preload en.
	;@polarity8 -> (CCER)  PosCh:$02 active LOW,
                  ;	NegCh: $08 active LOW
	;@preload8  -> (CR1) $80 preload , $20-center align.mode 1
	;@deadtime8 -> TIM1_DTR content
	; SP +10 after ret
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim1ComplementaryPwmCh2Setup
		PUSH A
	;-variables
	#define _015_v8a $00
	#define _015_deadtime $05
	#define _015_prel $06
	#define _015_pol $07
	#define _015_mode $08
	#define _015_compH $09
	#define _015_compL $0a
	#define _015_baseH $0b
	#define _015_baseL $0c
	#define _015_prescH $0d
	#define _015_prescL $0e
	;;allocate memory
	SUBW SP, #$01
	;--disable timer
	BRES TIM1_CR1, #$00
	;--load comparand, Hi firstly
	LD A, (_015_compH,SP)
	LD TIM1_CCR2H, A
	LD A, (_015_compL,SP)
	LD TIM1_CCR2L, A
		;--prescaler high byte firstly
	LD A, (_015_prescH,SP)
	LD TIM1_PSCRH, A	
	LD A, (_015_prescL,SP)
	LD TIM1_PSCRL, A
	;--load base, high byte first
	LD A, (_015_baseH,SP)
	LD TIM1_ARRH, A
	LD A, (_015_baseL,SP)
	LD TIM1_ARRL, A
	;--load CCMR1
	LD A, (_015_mode,SP)
	LD TIM1_CCMR2, A
	;--polarity
	;1)store an another channel CCER1
	LD A, TIM1_CCER1
	AND A, #$0F
	LD (_015_v8a,SP), A
	;--2)apply saved another channel
	LD A, (_015_pol,SP)
	OR A, #$05; Ch and ChN are turns ON
	SWAP A; A << 4 , because channel 2
	OR A, (_015_v8a,SP)
	LD TIM1_CCER1, A
	;--deadtime
	LD A, (_015_deadtime,SP)
	LD TIM1_DTR, A
	;--turn on channels
	BSET TIM1_BKR, #$07; MOE bit
	;--CR1
	LD A, (_015_prel,SP)
	LD TIM1_CR1, A
	BSET TIM1_CR1, #$00;//start
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	 
 ;--TEST PASSED!
	;-=====PROCEDURE===tim1InputCaptureCh1Setup
	;@prescaler8, $04->/2, $08->/4, ;
						;$0C->8,$0-NONE
	;@filter8, see ref.manual
	;@evt_condition8, $00-RISE,HIGH, $02-FALL,LOW
  ;------------
	;SP +5
	;stack frame:
 ;[v8a|A|RET|evt_condition|filter|prescaler|period]
tim1InputCaptureCh1Setup
  #define _016_v1a $00
	#define _016_evt_condition $05
	#define _016_filter $06
	#define _016_prescaler $07
	#define _016_periodH $08
	#define _016_periodL $09
  PUSH A;
	SUBW SP, #$01
	;--disable timer
	BRES TIM1_CR1, #$00
	;--CCMR1-
	LD A, (_016_filter,SP)
	OR A, (_016_prescaler,SP)
	OR A, #$01
	LD TIM1_CCMR1, A
	;---CCER1--
	;--1)save other channel state
	LD A, TIM1_CCER1
  AND A, #$F0 
	LD (_016_v1a,SP), A
	;--2)load value for tthis channel
	LD A, (_016_evt_condition,SP)
	;--3)aplly other channel to current 
	OR A, (_016_v1a,SP)
	LD TIM1_CCER1, A
	;--turn on CH1
	BSET TIM1_CCER1, #$00
	;---period
	LD A, (_016_periodH,SP)
	LD TIM1_ARRH, A
	LD A, (_016_periodL,SP)
	LD TIM1_ARRL, A
	;--turn on timer
	BSET, TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	;--TEST PASSED!
	;-=====PROCEDURE===tim1InputCaptureCh2Setup
	;@period16
	;@prescaler8, $04->/2, $08->/4, ;
						;$0C->8,$0-NONE
	;@filter8, see ref.manual
	;@evt_condition8, $00-RISE,HIGH, $02-FALL,LOW
  ;------------
	;SP +5
	;stack frame:
 ;[v8a|A|RET|evt_condition|filter|prescaler|period]
tim1InputCaptureCh2Setup
  #define _017_v1a $00
	#define _017_evt_condition $05
	#define _017_filter $06
	#define _017_prescaler $07
	#define _017_periodH $08
	#define _017_periodL $09
	
  PUSH A;
	SUBW SP, #$01
		;--disable timer
	BRES TIM1_CR1, #$00
	;--CCMR1-
	LD A, (_017_filter,SP)
	OR A, (_017_prescaler,SP)
	OR A, #$01
	LD TIM1_CCMR2, A
	;---CCER1--
	;--1)save other channel state
	LD A, TIM1_CCER1
  AND A, #$0F 
	LD (_017_v1a,SP), A
	;--2)load value for tthis channel
	LD A, (_017_evt_condition,SP)
	SWAP A ;<<4 because CH2
	;--3)aplly other channel to current 
	OR A, (_017_v1a,SP)
	LD TIM1_CCER1, A
	;--turn on CH2
	BSET TIM1_CCER2, #$03;//start
	;---period
	LD A, (_017_periodH,SP)
	LD TIM1_ARRH, A
	LD A, (_017_periodL,SP)
	LD TIM1_ARRL, A
	;--turn on timer
	BSET, TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	;***34*****
	
	 ;--TEST PASSED!
	;-=====PROCEDURE===tim1InputCaptureCh3Setup
	;@prescaler8, $04->/2, $08->/4, ;
						;$0C->8,$0-NONE
	;@filter8, see ref.manual
	;@evt_condition8, $00-RISE,HIGH, $02-FALL,LOW
  ;------------
	;SP +5
	;stack frame:
 ;[v8a|A|RET|evt_condition|filter|prescaler|period]
tim1InputCaptureCh3Setup
  #define _018_v1a $00
	#define _018_evt_condition $05
	#define _018_filter $06
	#define _018_prescaler $07
	#define _018_periodH $08
	#define _018_periodL $09
  PUSH A;
	SUBW SP, #$01
		;--disable timer
	BRES TIM1_CR1, #$00
	;--CCMR3-
	LD A, (_018_filter,SP)
	OR A, (_018_prescaler,SP)
	OR A, #$01
	LD TIM1_CCMR3, A
	;---CCER1--
	;--1)save other channel state
	LD A, TIM1_CCER2
  AND A, #$F0 
	LD (_018_v1a,SP), A
	;--2)load value for tthis channel
	LD A, (_018_evt_condition,SP)
	;--3)aplly other channel to current 
	OR A, (_018_v1a,SP)
	LD TIM1_CCER2, A
	;--turn on CH3
	BSET TIM1_CCER2, #$00
	;---period
	LD A, (_018_periodH,SP)
	LD TIM1_ARRH, A
	LD A, (_018_periodL,SP)
	LD TIM1_ARRL, A
	;--turn on timer
	BSET, TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	;--TEST PASSED!
	;-=====PROCEDURE===tim1InputCaptureCh4Setup
	;@period16
	;@prescaler8, $04->/2, $08->/4, ;
						;$0C->8,$0-NONE
	;@filter8, see ref.manual
	;@evt_condition8, $00-RISE,HIGH, $02-FALL,LOW
  ;------------
	;SP +5
	;stack frame:
 ;[v8a|A|RET|evt_condition|filter|prescaler|period]
tim1InputCaptureCh4Setup
  #define _019_v1a $00
	#define _019_evt_condition $05
	#define _019_filter $06
	#define _019_prescaler $07
	#define _019_periodH $08
	#define _019_periodL $09
	
  PUSH A;
	SUBW SP, #$01
		;--disable timer
	BRES TIM1_CR1, #$00
	;--CCMR1-
	LD A, (_019_filter,SP)
	OR A, (_019_prescaler,SP)
	OR A, #$01
	LD TIM1_CCMR4, A
	;---CCER1--
	;--1)save other channel state
	LD A, TIM1_CCER2
  AND A, #$0F 
	LD (_019_v1a,SP), A
	;--2)load value for tthis channel
	LD A, (_019_evt_condition,SP)
	SWAP A ;<<4 because CH2
	;--3)aplly other channel to current 
	OR A, (_019_v1a,SP)
	LD TIM1_CCER2, A
	;--turn on CH2
	BSET TIM1_CCER2, #$03
	;---period
	LD A, (_019_periodH,SP)
	LD TIM1_ARRH, A
	LD A, (_019_periodL,SP)
	LD TIM1_ARRL, A
	;--turn on timer
	BSET, TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	POP A
	RET
	
	;---TEST PASSED!
	;==========P R O C E D U R E===="tim1EncoderSetup"
  ;@band16,
	;@mode8, must be: $01 ->mod1, $02->mod2, $03->mod3	
	;@filter8, $00->filter_off, 
	             ;please see ref.manual TIM1_CCMR1, 2
  ;@polarity8,$00-Ch1 and Ch2-rise-edge-pos,
						  ;$02->fall-edge-neg-CH1 $20->fall-edge-neg-CH2
	;-------------------
	;after retutrn SP + 5
	;Stack frame;
	;[v8a | A | RET  | polarity |filter  |mode | band]
tim1EncoderSetup
	#define _020_v8a $00
	#define _020_mode $07
	#define _020_filter $06
	#define _020_polarity $05
	#define _020_bandH $08
	#define _020_bandL $09
	;save A
	PUSH A
	;allocate memory
	SUBW SP, #$01
	;--disable timer
	BRES TIM1_CR1, #$00
	;--SMCR
	LD A, (_020_mode,SP)
	LD TIM1_SMCR, A
	;--TIM1_CCMR1
	LD A,(_020_filter,SP)
	OR A, #$01; 
	LD TIM1_CCMR1, A
	;--TIM1_CCMR2
	LD A,(_020_filter,SP)
	OR A, #$01; 
	LD TIM1_CCMR2, A
	;--TIM_CCER1
	LD A, (_020_polarity,SP)
	OR A, #$11; 
  LD TIM1_CCER1, A
  ;--load band 
  LD A, (_020_bandH,SP)
  LD TIM1_ARRH, A
  LD A, (_020_bandL,SP)
  LD TIM1_ARRL, A	
	;--turn on 
	BSET TIM1_CR1, #$00
	;--free mem
	ADDW SP, #$01
	;--restore A
	POP A;
	RET
	
	;------C A P T U R E   C O M P A R E    I N T E R R U P T S 
	;====PROCDEDURE 'tim1CaptureCompareSetupInterrupts
	;turn on capture/compare interrupt enable for given channel
	;@channel8
	; must be $02->CH1, $04->CH2, $08->CH3, $1->CH4
	; to enable UPDATE interrupt - OR $01 
  ;after RETURN: SP +1
	;stack frame
	; 0  1 2 3      4
  ;[A |RETURN| channel]
tim1CaptureCompareSetupInterrupts
	PUSH A
	LD A , ($04,SP)
	LD TIM1_IER, A
	POP A
	RET

;  _____ ___ __  __ ____  
; |_   _|_ _|  \/  |___ \ 
;   | |  | || |\/| | __) |
;   | |  | || |  | |/ __/ 
;   |_| |___|_|  |_|_____|


;---T E S T   P A S S E D!
;=====P R O C E D U R E=='tim2BaseModeSetup'=
;===initializing TIM1 in base mode
; @prescaler16 (!ONLY LOW byte), @base16, @mode8
; After return: SP-5
;-modes can (TIM1_CR1) be ;
;-TIM_EDGE_ALIGNED ($00)
;- TIM_CENTER_ALINGN_1 ($20),
;- TIM_CENTER_ALINGN_2 ($40),
;- TIM_CENTER_ALINGN_3 ($60),
;count mode can be:
;- TIM_COUNTS_UP ($00)
;- TIM_COUNTS_DOWN ($10)
;--stack frame:
;   0   1  2 3  4    5  6  7  8 
 ;[| A |  RET |mode| base | presc| 

tim2BaseModeSetup
	;--store registers
	PUSH A
	#define _021_presH $07 ;prescaler
	#define _021_presL $08
	#define _021_baseH $05 ;base ,counter
	#define _021_baseL $06
	#define _021_mode $04  ; parameters
	;--disable timer
	BRES TIM2_CR1, #$00
	;load prescaler:
	LD A,(_021_presL,SP)
	LD TIM2_PSCR, A
	;load base, HI first:
	LD A, (_021_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_021_baseL,SP)
	LD TIM2_ARRL, A
  ;load mod in TIM1_CR1 (p.188 ref manual)
  LD A, (_021_mode,SP)
  LD TIM2_CR1, A
	;--start timer and GO! 
	BSET TIM2_CR1, #$00
	;--restore registers
	POP A
	RET
	
;--note! insert WFI inside main loop to allow interrups


;---T E S T   P A S S E D!
;=====P R O C E D U R E=='tim2BaseModeSetupIT'=
;===initializing TIM1 in base mode with interrupts
; @prescaler16 (!ONLY LOW byte), @base16, @mode8
; After return: SP-5
;-modes can (TIM1_CR1) be ;
;-TIM_EDGE_ALIGNED ($00)
;- TIM_CENTER_ALINGN_1 ($20),
;- TIM_CENTER_ALINGN_2 ($40),
;- TIM_CENTER_ALINGN_3 ($60),
;count mode can be:
;- TIM_COUNTS_UP ($00)
;- TIM_COUNTS_DOWN ($10)
;--stack frame:
;   0   1  2 3  4    5  6  7  8 
 ;[| A |  RET |mode| base | presc| 

tim2BaseModeSetupIT
	;--store registers
	PUSH A
	#define _022_presH $07 ;prescaler
	#define _022_presL $08
	#define _022_baseH $05 ;base ,counter
	#define _022_baseL $06
	#define _022_mode $04  ; parameters
	;--disable timer
	BRES TIM2_CR1, #$00
	;set interrupt update event enable
	BSET TIM2_IER, #$00
	;load prescaler:
	LD A,(_022_presL,SP)
	LD TIM2_PSCR, A
	;load base, HI first:
	LD A, (_022_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_022_baseL,SP)
	LD TIM2_ARRL, A
  ;load mod in TIM1_CR1 (p.188 ref manual)
  LD A, (_022_mode,SP)
  LD TIM2_CR1, A
	;--start timer and GO! 
	BSET TIM2_CR1, #$00

	;--restore registers
	POP A
	RET

	;====PROCDEDURE 'tim2CaptureCompareSetupInterrupts
	;turn on capture/compare interrupt enable for given channel
	;@channel8
	; must be $02->CH1, $04->CH2, $08->CH3, 
	; to enable UPDATE interrupt - OR $01
  ;after  RETURN: SP +1
	;stack frame
	; 0  1 2 3      4
  ;[A |RETURN| channel]
tim2CaptureCompareSetupInterrupts
	PUSH A
	LD A , ($04,SP)
	LD TIM2_IER, A
	POP A
	RET

;----T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2OutputCompareCh1Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;after ret SP +9
	;stack frame:
	;[v8a|A|return|prel|pol|mode|comp|base|presc]
tim2OutputCompareCh1Setup
		PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _023_v8a  $00 
	#define _023_prel $05
	#define _023_pol $06
	#define _023_mode $07
	#define _023_compH $08
	#define _023_compL $09
	#define _023_baseH $0A
	#define _023_baseL $0B
	#define _023_prescH $0C
	#define _023_prescL $0D
	;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_023_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_023_compH,SP)
	LD TIM2_CCR1H, A
	LD A, (_023_compL,SP)
	LD TIM2_CCR1L, A
	;--load base, high byte first
	LD A, (_023_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_023_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_023_mode,SP)
	LD TIM2_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM2_CCER1
	AND A, #$F0
	LD (_023_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_023_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_023_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_023_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	
  ;---T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2OutputCompareCh2Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2OutputCompareCh2Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _024_v8a $00
	#define _024_prel $05
	#define _024_pol $06
	#define _024_mode $07
	#define _024_compH $08
	#define _024_compL $09
	#define _024_baseH $0A
	#define _024_baseL $0B
	#define _024_prescH $0C
	#define _024_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_024_compH,SP)
	LD TIM2_CCR2H, A
	LD A, (_024_compL,SP)
	LD TIM2_CCR2L, A
	;--load base, high byte first
	LD A, (_024_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_024_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR2
	LD A, (_024_mode,SP)
	LD TIM2_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER1
	AND A, #$0F
	LD (_024_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_024_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_024_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_024_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	
	;--T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2OutputCompareCh3Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, -> capture compare CCR1 content
	;@mode8,  -> (CCMR)PIN  beheviavour: $10 active on match
							;$20 inactive on muatch $30 toggle pin
							;$30 force inactive Level $50 force active level
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2OutputCompareCh3Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _025_v8a $00
	#define _025_prel $05
	#define _025_pol $06
	#define _025_mode $07
	#define _025_compH $08
	#define _025_compL $09
	#define _025_baseH $0A
	#define _025_baseL $0B
	#define _025_prescH $0C
	#define _025_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_025_compH,SP)
	LD TIM2_CCR3H, A
	LD A, (_025_compL,SP)
	LD TIM2_CCR3L, A
 
	;--load base, high byte first
	LD A, (_025_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_025_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_025_mode,SP)
	LD TIM2_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER2
	AND A, #$F0
	LD (_025_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_025_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_025_v8a,SP)
	LD TIM2_CCER2, A
	;--CR1
	LD A, (_025_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	
	;----T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2PwmCh1Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2PwmCh1Setup
		PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _026_v8a  $00 
	#define _026_prel $05
	#define _026_pol $06
	#define _026_mode $07
	#define _026_compH $08
	#define _026_compL $09
	#define _026_baseH $0A
	#define _026_baseL $0B
	#define _026_prescH $0C
	#define _026_prescL $0D
	;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_026_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_026_compH,SP)
	LD TIM2_CCR1H, A
	LD A, (_026_compL,SP)
	LD TIM2_CCR1L, A
	;--load base, high byte first
	LD A, (_026_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_026_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_026_mode,SP)
	LD TIM2_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM2_CCER1
	AND A, #$F0
	LD (_026_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_026_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_026_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_026_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	;---T E S T   P A S S E D!
			;=====P R O C E D U R E==='tim2PwmCh2Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, -> c pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2PwmCh2Setup
		PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _027_v8a $00
	#define _027_prel $05
	#define _027_pol $06
	#define _027_mode $07
	#define _027_compH $08
	#define _027_compL $09
	#define _027_baseH $0A
	#define _027_baseL $0B
	#define _027_prescH $0C
	#define _027_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_027_compH,SP)
	LD TIM2_CCR2H, A
	LD A, (_027_compL,SP)
	LD TIM2_CCR2L, A
	;--load base, high byte first
	LD A, (_027_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_027_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR2
	LD A, (_027_mode,SP)
	LD TIM2_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER1
	AND A, #$0F
	LD (_027_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_027_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_027_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_027_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	
	;--T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2PwmCh3Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $00 active HI,$02 active low
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2PwmCh3Setup
			PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _028_v8a $00
	#define _028_prel $05
	#define _028_pol $06
	#define _028_mode $07
	#define _028_compH $08
	#define _028_compL $09
	#define _028_baseH $0A
	#define _028_baseL $0B
	#define _028_prescH $0C
	#define _028_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_028_compH,SP)
	LD TIM2_CCR3H, A
	LD A, (_028_compL,SP)
	LD TIM2_CCR3L, A
 
	;--load base, high byte first
	LD A, (_028_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_028_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_028_mode,SP)
	LD TIM2_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER2
	AND A, #$F0
	LD (_028_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_028_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_028_v8a,SP)
	LD TIM2_CCER2, A
	;--CR1
	LD A, (_028_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$00
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	
	;--T E S T   P A S S E D!
		;=====P R O C E D U R E==='tim2OnePulsePwmCh1Setup'
		;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $00 active HI,$02 active LO
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2OnePulsePwmCh1Setup
		PUSH A
	;allocate memory
  SUBW SP, #$01	
	;-variables
	#define _029_v8a  $00 
	#define _029_prel $05
	#define _029_pol $06
	#define _029_mode $07
	#define _029_compH $08
	#define _029_compL $09
	#define _029_baseH $0A
	#define _029_baseL $0B
	#define _029_prescH $0C
	#define _029_prescL $0D
	;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_029_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_029_compH,SP)
	LD TIM2_CCR1H, A
	LD A, (_029_compL,SP)
	LD TIM2_CCR1L, A
	;--load base, high byte first
	LD A, (_029_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_029_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_029_mode,SP)
	LD TIM2_CCMR1, A
	;--polarity
	;--1) read and store 
	; content of another channels
	LD A, TIM2_CCER1
	AND A, #$F0
	LD (_029_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_029_pol,SP)
	OR A, #$01; turn on channel 1
	;--3)apply others regs
	OR A, (_029_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_029_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$03;//ONE PULSE MODE, stopped
	;free memory
	ADDW SP, #$01
	POP A
	RET 
	
	;--T E S T   P A S S E D!
	;=====P R O C E D U R E==='tim2OnePulsePwmCh2Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $02 active LI,$00 active Hi
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2OnePulsePwmCh2Setup
	PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _030_v8a $00
	#define _030_prel $05
	#define _030_pol $06
	#define _030_mode $07
	#define _030_compH $08
	#define _030_compL $09
	#define _030_baseH $0A
	#define _030_baseL $0B
	#define _030_prescH $0C
	#define _030_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_030_compH,SP)
	LD TIM2_CCR2H, A
	LD A, (_030_compL,SP)
	LD TIM2_CCR2L, A
	;--load base, high byte first
	LD A, (_030_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_030_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR2
	LD A, (_030_mode,SP)
	LD TIM2_CCMR2, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER1
	AND A, #$0F
	LD (_030_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_030_pol,SP)
	OR A, #$01; turn on channel 
	SWAP A;  A << 4
	;--3) apply another regs
	OR A, (_030_v8a,SP)
	LD TIM2_CCER1, A
	;--CR1
	LD A, (_030_prel,SP)
	LD TIM2_CR1, A
  BSET TIM2_CR1, #$03;//one pulse mode, stopped
	;--fre memory
	ADDW SP, #$01
	POP A
	RET
	
	;--T E S T   P A S S E D !
;=====P R O C E D U R E==='tim2OnePulsePwmCh3Setup'
	;@presc16, !!! must be in TIM2 from list  (only LOW byte):
				  ;$01->/2, $02->/4, $03->/8 $04->/16 
					;$05->/32, $06->/64, $07->/128, $08->/256
					;$09->/512 $0A->/1024, $0B->/2048, $0C->/4096
					;$0D->/8192, $0E->/16384, $0F->/32768 
	;@base16, -> base of counter, ARRL
	;@comp16, ->  pulse width, CCR1 content
	;@mode8,  -> (CCMR)PWM mode: $60 pwm_mode_1, $70 pwm_mode_2
							;additional parameters (use OR) $04 -fast_mode
							; $80 - prelopad enable
	;@polarity8 -> (CCER)  $02 active LO,$00 active Hi
	;@preload8  ->  $80 preload enable, CR1 content
	;SP +9
	;stack frame:
	;[v16a|return|prel|pol|mode|comp|base|presc]
tim2OnePulsePwmCh3Setup
				PUSH A
		;--allocate memory
		SUBW SP , #$01
	;-variables
	#define _031_v8a $00
	#define _031_prel $05
	#define _031_pol $06
	#define _031_mode $07
	#define _031_compH $08
	#define _031_compL $09
	#define _031_baseH $0A
	#define _031_baseL $0B
	#define _031_prescH $0C
	#define _031_prescL $0D
		;--disable timer
	BRES TIM2_CR1, #$00
	;--prescaler only low	
	LD A, (_002_prescL,SP)
	LD TIM2_PSCR, A
	LD A, TIM2_PSCR
	;--load comparand, Hi firstly
	LD A, (_031_compH,SP)
	LD TIM2_CCR3H, A
	LD A, (_031_compL,SP)
	LD TIM2_CCR3L, A
 
	;--load base, high byte first
	LD A, (_031_baseH,SP)
	LD TIM2_ARRH, A
	LD A, (_031_baseL,SP)
	LD TIM2_ARRL, A
	;--load CCMR1
	LD A, (_031_mode,SP)
	LD TIM2_CCMR3, A
	;--polarity
	; 1)store content of another channels
	LD A, TIM2_CCER2
	AND A, #$F0
	LD (_031_v8a,SP), A; store
	;--2)load data for loading in register
	LD A, (_031_pol,SP)
	OR A, #$01; turn on channel 
	;--3)apply another regs
	OR A, (_031_v8a,SP)
	LD TIM2_CCER2, A
	;--CR1
	LD A, (_031_prel,SP)
	LD TIM2_CR1, A
	BSET TIM2_CR1, #$03;//ONE PULSE mode, stopped
	;--free memory
	ADDW SP, #$01
	POP A
	RET
	;--TEST PASSED!
	;=P R O C E D UR E======tim1OnePulseModeStart=======
	;start one pulse mode
	;SP afer RETURN nas not been changed 
tim2OnePulseModeStart
	LD TIM2_CR1, A
	OR A, #$09; ///ONE PULSE MODE + Go
	LD TIM2_CR1, A
	RET
	;----
;  _____ ___ __  __ _ _  
; |_   _|_ _|  \/  | | | 
;   | |  | || |\/| |_  _|
;   |_| |___|_|  |_| |_|
	
	 ;-TEST PASSED!
 ;===PROCEDURE==tim4BaseInit
 ;@prescaler8,   must be in range $00-$07
 ;				    so, divider=2 in power(prescaler)
 ;@counter8,     counter value
 ;@options8,   CR1 content
 ;---
 ;SP +3
 ;stack frame:
 ;[A|RET|options|counter|prescaler]
tim4BaseInit 
	#define _032_options $04
	#define _032_counter $05
	#define _032_prescaler $06
	;--save A
	PUSH A
	;--disable timer
	BRES TIM4_CR1, #$00
	;--prescaler 
	LD A, (_032_prescaler,SP)
	LD TIM4_PSCR, A
	;--counter 
	LD A, (_032_counter,SP)
	LD TIM4_ARR, A
	;--options
	LD A,(_032_options,SP)
	OR A, #$01 ;ON
	LD TIM4_CR1, A
	POP A
	RET
 ;--TESRT PASSED!
 ;===PROCEDURE==tim4BaseInitIT
 ;@prescaler8,   must be in range $00-$07
 ;				    so, divider=2 in power(prescaler)
 ;@counter8,     counter value
 ;@options8,   CR1 content
 ;---
 ;SP +3
 ;stack frame:
 ;[A|RET|options|counter|prescaler]
tim4BaseInitIT 
	#define _033_options $04
	#define _033_counter $05
	#define _033_prescaler $06
	;--save A
	PUSH A
	;--disable timer
	BRES TIM4_CR1, #$00
	;--enable interrupts
	LD A, #$01
	LD TIM4_IER, A
	;--prescaler 
	LD A, (_033_prescaler,SP)
	LD TIM4_PSCR, A
	;--counter 
	LD A, (_033_counter,SP)
	LD TIM4_ARR, A
	;--options
	LD A,(_033_options,SP)
	OR A, #$01 ;ON
	LD TIM4_CR1, A
	POP A
	RET