stm8/

	#include "mapping.inc"
	#include "stm8s003f3.inc"
	
	segment   at 0000 'USER_RAM'
	;--USER VARIABLES--
  ;pointer for receiver	$0000
_RxPtr_ equ $0000
	;pointer for transmitter $0002
_TxPtr_ equ $0002
 ; bit0-sybol has been received
 ; bit1 - it is a sybol for transmission
_semaphore equ $0004
  ;bufers for transmission and reception
_RxBuffer32 equ $0010
  #define  _RxBuffer32_end $002f ;last element
_TxBuffer32 equ $0030
  #define  _TxBuffer32_end $004f ; last element
_received_char equ $0050
  ;stringbuffer $0004
str1  equ $0051		
	;-----------------
	segment 'rom'     
_hello STRING "_PUTIN HUJLO",'\n','\r'

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

  LDW X, #_TxBuffer32;
;--global pointer UART Rx bufffer init
;[*P=$0000][$0002...$0012]
  LD A, str1;
  LDW X, #$0002
  LDW $0000, X
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
	 
	;--PC3,4-output 
	LD A, #$38; 3,4,5 
	LD PC_DDR, A
	LD PC_CR1, A
	LD PC_CR2, A
	;--PA3
	 	LD A, #$00;  
	LD PA_DDR, A
	LD PA_CR1, A
	LD PA_CR2, A
	;--D
		LD A, #$20;  
	LD PD_DDR, A;   PD5 output,
	BSET PD_ODR, #$05 ; high level PD5 
	LD PD_CR1, A; push-pull,
	LD PD_CR1, A; up to 10MHz.
	;dumy delay
   NOP
	 NOP
	 NOP
 
	
	;@baudRate16, divider
  LDW X, #$0209
	PUSHW X
;@dataLength8, ;
	;$10->9bits (1stop 1 start) 
	;, $00->8bits (set  ;manually below)
	LD A, #$00
	PUSH A
;@stopBits8  $00->1bit, $20->2bits
  LD A, #$00
	PUSH A
	LD A, #$00 ;@parity8, $00-disable, 
	;$04-enable: ($02-odd OR $00-even)
	PUSH A
	CALL  uart1DuplexSetupIT
;-----------
;SP +5
  ADDW SP, #$05
	LD A,#$31
	nop
	nop
  nop
	nop

 
	 
   ;--L     o o    Pppp
	 ;  L    o   o   P   p
	 ;  L    o   o   Ppp 
	 ;  L L   o o    P
	 ;;init pointers
	 LDW X, #_RxBuffer32
	 LDW _RxPtr_, X
	 LDW X, #_TxBuffer32
	 LDW _TxPtr_, X
infinite_loop.l
			;are there  symbol?
			BTJF _semaphore, #$0, _no_sym_reeived
			;when symbol received:
			BRES _semaphore, #$0 ;clear semaphore
			LDW X, _RxPtr_ ; load a pointer
			LD A, _received_char ;read symbol from Rx
			LD (X), A ; store symbol in an arrray
			;when the symbol last - assign to pointer first cell
			LDW Y, X
			SUBW Y, #_RxBuffer32_end 
			JREQ _last_rx_item
			;when character (byte) not last - increment & store pointer
			INCW X
			LDW _RxPtr_, X
			JP _no_sym_reeived    
_last_rx_item
      ;when the symbol is  last - clear pointer
			LDW X, #_RxBuffer32
			LDW _RxPtr_, X
			;LD UART1_DR, A
_no_sym_reeived  

				WFI
	jra infinite_loop
     

	

;------------------------------	
;  _            _   
; | |_ ___  ___| |_ 
; | __/ _ \/ __| __|
; | ||  __/\__ \ |_ 
;  \__\___||___/\__|
;-------------------------------

;==PROCEDURE==uart1ReceiverSetup
;@baudRate16, divider
;@dataLength8, $10->9bits (1stop 1 start) , $00->8bits
          ;(set  manually below)
;@stopBits8, active only when 8 bits: $00->1bit, $20->2bits
;@parity8, $00-disable, $04-enable: ($02-odd OR $00-even)
;-----------
;SP +5
;  0  1 2 4    5         6      7      8 
;[v8a|A|RET|stopBits|dataLength|baudRate]

uart1ReceiverSetup
  ;--store A
	PUSH A
	;--allocate 1 byte
	SUBW SP, #$01
	#define _U1000_v8a $00 
	#define _U1000_parity $05
	#define _U1000_stopBits $06 
	#define _U1000_dataLength $07
	#define _U1000_baudRateH $08
	#define _U1000_baudRateL $09
	;--disable Rx, Tx
	BRES UART1_CR2, #$2; REN flag
	BRES UART1_CR2, #$3; TEN flag
  ;--when the length 9 bits-jump below
	LD A, #$10
	AND A, (_U1000_dataLength,SP)
	JRNE L_U1000_nsb ;when 9 bits-go to label
	;--when 8 bit set stop bits
	LD A, (_U1000_stopBits,SP)
	LD UART1_CR3, A
L_U1000_nsb
  ;--write data length and parity
	LD A,(_U1000_dataLength,SP)
	OR A, (_U1000_parity,SP)
	LD UART1_CR1, A
	;prepare UART1_BRR2
	;n4 n1
	LD A, (_U1000_baudRateL,SP)
	AND A, #$0F;
	LD ( _U1000_v8a,SP),A; store nibble 1
	LD A, (_U1000_baudRateH,SP)
	AND A, #$f0 ; nibble 4
	OR A, (_U1000_v8a,SP) ; n4+n1
	LD ( _U1000_v8a,SP),A; store BRR2 [n4,n1]
	;;--send -TO REGISTER BRR2
	LD UART1_BRR2 , A
	;----n3 n2
	LD A, (_U1000_baudRateH,SP)
	SWAP A
	AND A, #$F0
	LD ( _U1000_v8a,SP),A; store n3
	LD A, (_U1000_baudRateL,SP)
	SWAP A
	AND A, #$0f
	OR A, ( _U1000_v8a,SP)
	;--send to register BRR1
	LD UART1_BRR1, A
	; RIEN interrupt on receive
	BSET UART1_CR2, #$5 
	;--setting REN bit "Receiver enable"
	BSET UART1_CR2 , #$2
	;--restore stack
	ADDW SP, #$01
	POP A
	RET
	;==PROCEDURE==uart1TransmitterSetup
 
;@baudRate16, divider
;@dataLength8, $10->9bits (1stop 1 start) , $00->8bits
          ;(set  manually below)
;@stopBits8, active only when 8 bits: $00->1bit, $20->2bits
;@parity8, $00-disable, $04-enable: ($02-odd OR $00-even)
;-----------
;SP +5
;  0  1 2 4    5         6      7      8 
;[v8a|A|RET|stopBits|dataLength|baudRate]

uart1TransmitterSetup
  ;--store A
	PUSH A
	;--allocate 1 byte
	SUBW SP, #$01
	#define _U1001_v8a $00 
	#define _U1001_parity $05
	#define _U1001_stopBits $06 
	#define _U1001_dataLength $07
	#define _U1001_baudRateH $08
	#define _U1001_baudRateL $09
	;--disable Rx, Tx
	BRES UART1_CR2, #$2; REN flag
	BRES UART1_CR2, #$3; TEN flag
  ;--when the length 9 bits-jump below
	LD A, #$10
	AND A, (_U1001_dataLength,SP)
	JRNE L_U1001_nsb ;when 9 bits-go to label
	;--when 8 bit set stop bits
	LD A, (_U1001_stopBits,SP)
	LD UART1_CR3, A
L_U1001_nsb
  ;--write data length and parity
	LD A,(_U1001_dataLength,SP)
	OR A, (_U1001_parity,SP)
	LD UART1_CR1, A
	;prepare UART1_BRR2
	;n4 n1
	LD A, (_U1001_baudRateL,SP)
	AND A, #$0F;
	LD ( _U1001_v8a,SP),A; store nibble 1
	LD A, (_U1001_baudRateH,SP)
	AND A, #$f0 ; nibble 4
	OR A, (_U1001_v8a,SP) ; n4+n1
	LD ( _U1001_v8a,SP),A; store BRR2 [n4,n1]
	;;--send -TO REGISTER BRR2
	LD UART1_BRR2 , A
	;----n3 n2
	LD A, (_U1001_baudRateH,SP)
	SWAP A
	AND A, #$F0
	LD ( _U1001_v8a,SP),A; store n3
	LD A, (_U1001_baudRateL,SP)
	SWAP A
	AND A, #$0f
	OR A, ( _U1001_v8a,SP)
	;--send to register BRR1
	LD UART1_BRR1, A
	; TCIEN interrupt on tx complete
	BSET UART1_CR2, #$6 
	;--setting TEN bit "Transmitter enable"
	BSET UART1_CR2 , #$3
	;--restore stack
	ADDW SP, #$01
	POP A
	RET
;==PROCEDURE==uart1DuplexSetupIT
;--set up Receiver and Transmitter
; Turn on  TC, RXNE interrupts.
;---------------------
;@baudRate16, divider
;@dataLength8, $10->9bits (1stop 1 start) , $00->8bits
          ;(set  manually below)
;@stopBits8, active only when 8 bits: $00->1bit, $20->2bits
;@parity8, $00-disable, $04-enable: ($02-odd OR $00-even)
;-----------
;SP +5
;  0  1 2 4    5         6      7      8 
;[v8a|A|RET|stopBits|dataLength|baudRate]

uart1DuplexSetupIT
  ;--store A
	PUSH A
	;--allocate 1 byte
	SUBW SP, #$01
	#define _U1003_v8a $00 
	#define _U1003_parity $05
	#define _U1003_stopBits $06 
	#define _U1003_dataLength $07
	#define _U1003_baudRateH $08
	#define _U1003_baudRateL $09
	;--disable Rx, Tx
	BRES UART1_CR2, #$2; REN flag
	BRES UART1_CR2, #$3; TEN flag
  ;--when the length 9 bits-jump below
	LD A, #$10
	AND A, (_U1003_dataLength,SP)
	JRNE L_U1003_nsb ;when 9 bits-go to label
	;--when 8 bit set stop bits
	LD A, (_U1003_stopBits,SP)
	LD UART1_CR3, A
L_U1003_nsb
  ;--write data length and parity
	LD A,(_U1003_dataLength,SP)
	OR A, (_U1003_parity,SP)
	LD UART1_CR1, A
	;prepare UART1_BRR2
	;n4 n1
	LD A, (_U1003_baudRateL,SP)
	AND A, #$0F;
	LD ( _U1003_v8a,SP),A; store nibble 1
	LD A, (_U1003_baudRateH,SP)
	AND A, #$f0 ; nibble 4
	OR A, (_U1003_v8a,SP) ; n4+n1
	LD ( _U1003_v8a,SP),A; store BRR2 [n4,n1]
	;;--send -TO REGISTER BRR2
	LD UART1_BRR2 , A
	;----n3 n2
	LD A, (_U1003_baudRateH,SP)
	SWAP A
	AND A, #$F0
	LD ( _U1003_v8a,SP),A; store n3
	LD A, (_U1003_baudRateL,SP)
	SWAP A
	AND A, #$0f
	OR A, ( _U1003_v8a,SP)
	;--send to register BRR1
	LD UART1_BRR1, A
	;turn on interrupt on tx complete, Rx not empty
	BSET UART1_CR2, #$6; TCIEN
	BSET UART1_CR2, #$5; RIEN
	;--set  "Transmitter enable", "receiver enable"
	BSET UART1_CR2 , #$3
	BSET UART1_CR2 , #$2
	;--restore stack
	ADDW SP, #$01
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
	IRET	
	
 
	
onTim1Update
	IRET

uart1OnReceive
  BSET PC_ODR, #$3
  LD A, UART1_DR 
	BSET _semaphore, #$0 ;;set semaphore
	LD _received_char, A ;store symbol for echo
	;--test on errors
	LD A, UART1_SR
	AND A, #$0F
	BRES PC_ODR, #$5 ;turn OFF error LED
	JREQ _NO_ERR_RX  ; ? are there no errors?
	BSET PC_ODR, #$5 ;turn ON error LED
_NO_ERR_RX
	;when data has been transmitted
	BTJF UART1_SR, #$6, NO_DATA_IN_TC
WAIT_UNTIL_TC
		;wait until transmission complete
		BTJF UART1_SR, #$6, WAIT_UNTIL_TC
NO_DATA_IN_TC
  NOP

	BRES PC_ODR, #$3
	NOP
  IRET
	
	
uart1OnTransmit
   BSET PC_ODR, #$4
	 BRES PC_ODR, #$7
   LD A, UART1_SR
	 BRES UART1_SR, #$6
	 BRES UART1_SR, #$7
	 ;--test on errors
	 AND A, #$0F
	 BRES PC_ODR, #$5 ;turn OFF error LED
	 JREQ _NO_ERR_TX  ; ? are there no errors?
	 BSET PC_ODR, #$5 ;turn ON error LED
_NO_ERR_TX
	 NOP
	 NOP
	 BRES PC_ODR, #$4
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
	dc.l {$82000000+uart1OnTransmit}	; irq17
	dc.l {$82000000+uart1OnReceive}	; irq18
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
