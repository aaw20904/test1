;---Author: Andrii Androsovych
;initialization  UART
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