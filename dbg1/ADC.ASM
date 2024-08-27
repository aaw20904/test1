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
	
		;====P R O C E D U R E===adcContModeSetup, continuous mode
	;@channel8, $00->AIN0, $01->AIN1, #$0F->AIN15
	;@prescaler, $00->/2, $10->/3, $20->/4, $30->/6, $40->/8, $50->/10
	;ADC DATA is right aligned!
	;SP+2
	;----------
	;stack frame
	;[A|RET|prescaler|channel]
adcContinModeSetup
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
	OR A, #$03; ADON, CONT
	LD ADC_CR1, A
	BSET ADC_CR1, #$00; start
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
	
		;====P R O C E D U R E===adcContinScanModeSetup,
	;@channel8, $00->AIN0, $01->AIN1, #$0F->AIN15
	;@prescaler, $00->/2, $10->/3, $20->/4, $30->/6, $40->/8, $50->/10
	;ADC DATA is right aligned!
	;SP+2
	;----------
	;stack frame
	;[A|RET|prescaler|channel]
	;--NOTE: result stores in ADC_DBxRH, ADC_DBxRL
adcContinScanModeSetup
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
	OR A, #$03; ADON CONT
	LD ADC_CR1, A
	BSET ADC_CR1, #$00; start
	;--restore
	POP A
	RET
	
	;---MACRO---
	;--starting ADC in single scan mode
	;ADC must be configurated before
startAdcScan MACRO max_adc_ch
	LD A, ADC_CSR
	AND A, #$7F; clear EOC
	OR A, max_adc_ch
	LD ADC_CSR, A
	BSET ADC_CR1, #$00
	MEND
	