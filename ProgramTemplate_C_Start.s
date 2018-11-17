            TTL Program Title for Listing Header Goes Here
;****************************************************************
;Descriptive comment header goes here.
;(What does the program do?)
;Name:  <Your name here>
;Date:  <Date completed here>
;Class:  CMPE-250
;Section:  <Your lab section, day, and time here>
;---------------------------------------------------------------
;Keil Template for KL46 Assembly with Keil C startup
;R. W. Melton
;November 13, 2017
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL46Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;---------------------------------------------------------------
;Characters
CR          EQU  0x0D
LF          EQU  0x0A
NULL        EQU  0x00
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port A
PORT_PCR_SET_PTA1_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTA2_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->09:Port A clock gate control (enabled)
;Use provided SIM_SCGC5_PORTA_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select
;         (PLLFLLSEL determines MCGFLLCLK' or MCGPLLCLK/2)
; 1=   16:PLLFLLSEL=PLL/FLL clock select (MCGPLLCLK/2)
SIM_SOPT2_UART0SRC_MCGPLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
SIM_SOPT2_UART0_MCGPLLCLK_DIV2 EQU \
    (SIM_SOPT2_UART0SRC_MCGPLLCLK :OR: SIM_SOPT2_PLLFLLSEL_MASK)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;0x38->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGPLLCLK/2
;MCGPLLCLK is 96 MHz
;MCGPLLCLK/2 is 48 MHz
;SBR = 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------

IN_PTR		EQU		0
OUT_PTR		EQU		4
BUF_STRT	EQU		8
BUF_PAST	EQU		12
BUF_SIZE	EQU		16
NUM_ENQD	EQU		17	
MAXSIZE		EQU		80
DAC0_STEPS	EQU		4096
SERVO_POSITIONS	EQU	5
;TPM0_C4V symbols
TPM_CnV_PWM_DUTY_2ms EQU 6500
TPM_CnV_PWM_DUTY_1ms EQU 2800 
PWM_2ms EQU TPM_CnV_PWM_DUTY_2ms
PWM_1ms EQU TPM_CnV_PWM_DUTY_1ms 

;****************************************************************
;MACRO
;---------------------------------------------------------------
			MACRO
			RMVC
			PUSH {R0-R1}
			MRS  R0, APSR	;starting to set the C flag to 0.
			MOVS R1, #0x20
			LSLS R1, R1, #24
			BICS R0, R0, R1
			MSR  APSR, R0
			POP  {R0-R1}
			MEND
;----------------------------------------------------------------
			MACRO
			SETC
			PUSH {R0-R1}
			MRS  R0, APSR	;starting to set the C flag to 1.
			MOVS R1, #0x20
			LSLS R1, R1, #24
			ORRS R0, R0, R1
			MSR  APSR, R0
			POP  {R0-R1}
			MEND
			
;----------------------------------------------------------------
			MACRO
			NewLine
			PUSH  {R0-R1}
			MRS   R1, APSR
			MOVS  R0, #10					;move a new line
			BL    PutChar					;put that new line in
			MOVS  R0, #13					;sets back to the original
			BL	  PutChar					;put that on to the terminal
			MSR   APSR, R1
			POP	  {R0-R1}
			MEND
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
			EXPORT	GetChar
			EXPORT	GetStringSB
			EXPORT	Init_UART0_IRQ
			EXPORT	PutChar
			EXPORT	PutNumHex
			EXPORT	PutNumUB
			EXPORT	PutStringSB
			EXPORT	UART0_IRQHandler
;>>>>> begin subroutine code <<<<<
Init_UART0_IRQ	PROC {R0-R14}
;initializes all the control registers to the correct value
;Parameters
;	Inputs:
;			no inputs
;	Outputs:
;			no outputs
;	Register modifcation list:
;			no registers

		
			PUSH {R0-R3, LR}
			
			;initalize the recieve buffer and the transmit buffer through initqueue
			LDR  R0, =RxQBuffer		;load the address of the recieve buffer
			LDR  R1, =RxQRecord		;load the address of the recieve record
			MOVS R2, #MAXSIZE 		;load the size of the buffer queue
			BL   InitQueue			;branch to initqueue
			LDR  R0, =TxQBuffer		;load the address of the transmit buffer
			LDR  R1, =TxQRecord		;load the address of the transmit record
			BL   InitQueue			;branch to initqueue
			
			
			
		;Select MCGPLLCK / 2 as UART0 clock source
			LDR  R0, =SIM_SOPT2
			LDR  R1, =SIM_SOPT2_UART0SRC_MASK
			LDR  R2, [R0, #0]
			BICS R2, R2, R1
			LDR  R1, =SIM_SOPT2_UART0_MCGPLLCLK_DIV2
			ORRS R2, R2, R1
			STR  R2, [R0, #0]
			
			;Enable externl connection for UART0
			LDR  R0, =SIM_SOPT5
			LDR  R1, =SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR  R2, [R0, #0]
			BICS R2, R2, R1
			STR  R2, [R0, #0]
			
			;Enable clock for UART0 module
			LDR  R0, =SIM_SCGC4
			LDR  R1, =SIM_SCGC4_UART0_MASK
			LDR  R2, [R0, #0]
			ORRS R2, R2, R1
			STR  R2, [R0, #0]
			
			;Enable clock for Port A Module
			LDR	 R0, =SIM_SCGC5
			LDR  R1, =SIM_SCGC5_PORTA_MASK
			LDR  R2, [R0, #0]
			ORRS R2, R2, R1
			STR  R2, [R0, #0]
			
			;Connect PORT A Pin 1 (PTA1) to UART0 Rx (J1 Pin 02)
			LDR R0, =PORTA_PCR1
			LDR R1, =PORT_PCR_SET_PTA1_UART0_RX
			STR R1, [R0, #0]
			
			;Connect PORT A Pin 2 (PTA2) to UART0 Tx (J1 Pin 04)
			LDR R0, =PORTA_PCR2
			LDR R1, =PORT_PCR_SET_PTA2_UART0_TX
			STR R1, [R0, #0]
			
			
			;loading the base UART0 and disabling reciever and transmitter
			LDR  R0, =UART0_BASE				;load the UART0 base
						
			;setting the high baud rate
			MOVS R1, #UART0_BDH_9600			;move in the equate for the baud high rate
			STRB R1, [R0, #UART0_BDH_OFFSET]	;store that value into the baud high offset
			
			;setting the low baud rate
			MOVS R1, #UART0_BDL_9600			;move in the equate for the baud low rate
			STRB R1, [R0, #UART0_BDL_OFFSET]	;store that value into the baud low offset
			
			;set UART0 IRQ priority
			LDR  R0, =UART0_IPR					;load the uart0 IPR into R0
			LDR  R2, =NVIC_IPR_UART0_PRI_3		;load the IPR_UART0_PRI_3
			LDR  R3, [R0,#0]					;load that the offset 0 value in R0 into R3
			ORRS R3, R3, R2						;ORRS it with R2
			STR  R3, [R0,#0]					;store that value into the offset 0 value in R0
			
			;clear any pending UART0 interrupts
			LDR  R0, =NVIC_ICPR					;load the R0 NVIC_ICPR
			LDR  R1, =NVIC_ICPR_UART0_MASK		;load the mask 
			STR  R1, [R0,#0]					;store the mask into the NVIC_ICPR
			
			;Unmask UART0 interrupts
			LDR  R0, =NVIC_ISER					;load NVIC_ISER
			LDR  R1, =NVIC_ISER_UART0_MASK		;load the mask
			STR  R1,[R0,#0]						;store the mask into NVIC_ISER
			
			
			;set control register 1
			MOVS R1, #UART0_C1_8N1  				;moving in the bits to set control 0
			STRB R1, [R0, #UART0_C1_OFFSET]  		;storing the value of R1 into R0
			
			;set control register 2
			MOVS R1, #UART0_C2_T_R	   				;moving in the bits to set control 0
			STRB R1, [R0, #UART0_C2_OFFSET]  		;storing the value of R1 into R0
			
			;set control register 3
			MOVS R1, #UART0_C3_NO_TXINV				;moving in the bits to set control 0
			STRB R1, [R0, #UART0_C3_OFFSET]			;storing the value of R1 into R0
			
			;set control register 4
			MOVS R1, #UART0_C4_NO_MATCH_OSR_16		;moving in the bits to set control 0
			STRB R1, [R0, #UART0_C4_OFFSET]			;storing the value of R1 into R0
			
			
			;set control register 5
			MOVS R1, #UART0_C5_NO_DMA_SSR_SYNC		;moving in the bits to set control 0
			STRB R1, [R0, #UART0_C5_OFFSET]			;storing the value of R1 into R0
			
			
			;set status
			MOVS R1, #UART0_S1_CLEAR_FLAGS			;moving in the bits to set control 0
			STRB R1, [R0, #UART0_S1_OFFSET] 		;storing the value of R1 into R0
			
			MOVS R1, #UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS 
			STRB R1, [R0, #UART0_S2_OFFSET]
			
			;enabling reciever and transmitter
			MOVS R1, #UART0_C2_T_RI
			STRB R1, [R0, #UART0_C2_OFFSET]
			POP {R0-R3, PC}
	
			ENDP
				

UART0_IRQHandler
UART0_ISR	PROC {R4-R13}
;this subroutine handles UART0 transmit and recieve interrupts
;Parameters
;	Inputs:
;			
;	Outputs:
;			No outputs
;	Register modifcation list:
;			no registers modified
			CPSID I								;mask all KL46 interrupts
			PUSH {LR}							;save all modified register
			;if TxIRQ is enabled 	
			LDR	  R2, =UART0_BASE				;load the uart0base address into R2
			MOVS  R0, #UART0_C2_TIE_MASK		;get the C2 mask
			LDRB  R3, [R2, #UART0_C2_OFFSET]	;gets the byte of the offset
			TST   R3, R0						;AND them together but don't save the information of the AND
			BEQ   RxCheck						;branch to RxCheck which will check for the RDRF
			MOVS  R0, #UART0_S1_TDRE_MASK		;if it's not 0 then get the TDRE mask
			LDRB  R3, [R2, #UART0_S1_OFFSET]	;get the byte of the offset for Status 1
			TST   R3, R0						;AND the TDRE mask the S1_Offset 
			BEQ   RxCheck						;if it's 0 then branch to RxCheck
			LDR   R1, =TxQRecord				;load the TxQRecord address into R1
			BL    Dequeue						;branch to Dequeue
			BCC   TxSucc						;branch to TxSucc which means dequeue succeeded
			MOVS  R1, #UART0_C2_T_RI			;disables TxInterrupt
			STRB  R1, [R2, #UART0_C2_OFFSET]	;store it into the C2 offset which disables the interrupt
			B	  IsrFinish
			
RxCheck		MOVS  R0, #UART0_S1_RDRF_MASK		;get the S1 mask
			LDRB  R3, [R2, #UART0_S1_OFFSET]	;gets the byte of the offset
			TST   R3, R0						;AND them together but don't save the information of the AND
			BEQ   IsrFinish						;branch to IsrFinish if it's 0
			LDRB  R0, [R2, #UART0_D_OFFSET]		;get the character from the user to enqueue
			LDR   R1, =RxQRecord				;load the TxQRecord address into R1
			BL    Enqueue						;branch to enqueue
			B	  IsrFinish						;branch to IsrFinish

TxSucc		STRB  R0, [R2, #UART0_D_OFFSET]
						
			
IsrFinish	CPSIE I
	
			POP  {PC}
			ENDP
				
			
PutChar		PROC {R1-R14}
;transmit a character into UART0 TDR when UART0 TDRE is 1 which means it's ready to recieve a character and displays it
;Parameters
;	Inputs:
;			R0: the character that wants to be displayed
;	Outputs:
;			No outputs
;	Register modifcation list:
;			no registers modified

			PUSH {R1-R4, LR}					;pushes registers R0, R1, and R2 so the original value is stored
			CPSID I								;mask the interrupts
loop		LDR   R1, =TxQRecord				;load the TxQRecord address into R1
			BL    Enqueue						;enqueue the character
			CPSIE I								;unmask the interrupts
			BCS	  loop							;branch to loop if enqueue failed
			LDR   R3, =UART0_BASE
			LDRB  R4, [R3, #UART0_C2_OFFSET]	
			MOVS  R2, #UART0_C2_TI_RI			;enable TxInterrupt
			ORRS  R2, R2, R4
			STRB  R2, [R3, #UART0_C2_OFFSET]	;store it into the C2 offset which enables the interrupt
			POP	{R1-R4, PC}						;pop the registers
			ENDP								;end the subroutine
				
GetChar		PROC {R1-R14}		
;When UART0's status register recieves a character RDRF is set which means the register can read the character.
;Parameters
;	Inputs:
;			no inputs
;	Outputs:
;			R0: the character that is received.
;	Register modifcation list:
;			R0
			PUSH {R1-R3, LR}			;pushes registers R0, R1, and R2 so the original value is stored
			
			CPSID I						;mask the interrupts
loop2		LDR   R1, =RxQRecord		;load the RxQRecord
			BL    Dequeue				;dequeue the character
			CPSIE I						;unmask the interrupts
			BCS   loop2					;branch back to loop if Carry is set
			POP	{R1-R3, PC}				;pop the registers
			ENDP						;end the subroutine


DIVU		PROC {R2-R14}
;Performs a dvision operation. The equation is R0 = R1/R0 and remainder = R0
;Parameters
;	Inputs:
;		R1: Dividend, the number that will be divided
;		R0: Divisor, the number that will be doing the dividing.
;	Outputs:
;		R1: The remainder from the division
;		R0: The final answer from the division.
			PUSH {R3}		;stores the orignal R3 value
			MOVS R3, #0		;The number that will become the final answer of the division.
			CMP R0, #0		;comparing R0 to 0
			BEQ DIV0		;checking if the divisor is equal to 0
			CMP R1, #0		;comparing R1 to 0
			BEQ ANS0		;checking if the dividend is equal to 0
DIVLOOP		CMP R1, R0		;comparing the dividend and the divisor
			BLO FINISH		;comparing if the divisor is greater than the dividend
			ADDS R3, R3, #1	;add a value to the quotient
			SUBS R1, R1, R0	;subtracting the dividend by the divisor
			B	DIVLOOP		;loops back
			
			
DIV0		MOVS R1, #0		;since it divided by 0 or the dividend was 0 it sets R1 to 0
			MOVS R0, #0		;since it divided by 0 or the dividend was 0 it sets R0 to 0
			MRS  R0, APSR	;starting to set the C flag to 1.
			MOVS R1, #0x20
			LSLS R1, R1, #24
			ORRS R0, R0, R1
			MSR  APSR, R0
			POP	 {R3}
			BX	 LR			;branch back to the main code
			
ANS0		
			MRS  R0, APSR	;starting to set the C flag to 0.
			MOVS R1, #0x20
			LSLS R1, R1, #24
			BICS R0, R0, R1
			MSR  APSR, R0
			MOVS R1, #0		;since it divided by 0 or the dividend was 0 it sets R1 to 0
			MOVS R0, #0		;since it divided by 0 or the dividend was 0 it sets R0 to 0
			POP	 {R3}		;Sets R4 back to the original value
			BX	 LR			;branch back to the main code

			
FINISH		
			MOVS R0, R3		;move the valu of the quotient into R0 which is the result of the division
			POP	 {R3}		;Sets R4 back to the original value
			BX	 LR			;branches back to the main code
			ENDP			;ends the function.

GetStringSB	PROC {R0-R14}
;performs GetStringSB which prevents an overrun of the buffer capacity specified reads a string from the terminal keyboard to memory starting from the address 
;R0 and adds null termination
;Parameters
;	Inputs:
;		no inputs
;	Outputs:
;		no outputs

			PUSH {R0-R3, LR}  ;pushing R0-R3
			LDR  R2, =STRING  ;loading the string pointer to R2
			ADDS R3, R2, R1	  ;adding the string pointer and the buffer into R3
STRT		CMP  R3, R2		  ;comparing R3 and R2
			BLS  WAITENT	  ;ending the loop if it's greater than the string buffer
			BL 	 GetChar	  ;branching to the GetChar subroutine which will get the char from the terminal
			BL   PutChar
			CMP  R0, #13	  ;comparing the character and the enter character in ascii value
			BEQ  ENDLOOP	  ;if it's equal to 0 then you can branch out.
			STRB R0, [R2, #0] ;storing the character into R2
			ADDS R2, R2, #1	  ;incrementing the pointer by 1
			B    STRT		  ;branching back to the start
			
WAITENT     
			BL   GetChar
			CMP  R0, #13	  ;comparingw the character and the enter character in ascii value
			BEQ  ENDLOOP	  ;if it's equal to 0 then you can branch out.
			B    WAITENT
			
ENDLOOP		MOVS R0, #0		  ;moves 0 whichs is the null into R0
			STRB R0, [R2, #0] ;stores the null terminator into R2
			MOVS R0, #13	  ;moves 13 which is enter into R0
			BL   PutChar	  ;Puts the enter into R0
			MOVS R0, #10	  ;moves 10 which is new line into R0
			BL	 PutChar	  ;puts 10 into the terminal which makes a new line
			POP	 {R0-R3, PC}  ;pops the registers
			ENDP			  ;ends the subroutine
				
PutStringSB	PROC {R0-R14}			
;performs PutStingSB which prevents an overrun of the buffer capacity specified and displays a null-terminated string to the terminal screen. 
;Parameters
;	Inputs:
;		R0 : The null terminated string 
;	Outputs:
;		no outputs
			PUSH  {R0-R3, LR}  ;push the values of R0-R3 and LR so they dont get modified at the end
			MOVS  R2, R0	   ;Move R0 into R2

			ADDS  R3, R2, R1   ;add the string and the buffer
PUTSTRING	CMP   R3, R2       ;compare R3 and R2
			BLS   ENDPUTLOOP   ;check R3 is lower than R2 and if it is then end the loop
			LDRB  R0, [R2, #0] ;load a byte into R0 from R2
			CMP   R0, #0	   ;compare R0 and 0
			BEQ   ENDPUTLOOP   ;if it's 0 then branch to end the loop
			BL 	  PutChar      ;put the character onto the terminal
			ADDS  R2, R2, #1   ;add R2 so it goes to the next address
			B     PUTSTRING    ;branch back to the start of the loop
			
ENDPUTLOOP	
			;BL PutChar		   ;put 1 more character to the screen
			POP {R0-R3, PC}    ;pop the registers and PC
			ENDP
				
PutNumU		PROC {R0-R14}
;takes a number in hex and converts it into decimal and prints out onto the terminal screen
;Parameters
;	Inputs:
;		R0 : The number that is putting converted to put onto terminal screen
;	Outputs:
;		no outputs			
			PUSH {R0-R4, LR}	;push R0-R4 and LR
			MOVS R3, #0			;make a counter
PUTNUMLP	ADDS R3, #1			;add 1 to the counter
			MOVS R1, R0			;move R0 into R1 to perform the division
			MOVS R0, #10		;move 10 into R0 which is the number that will do he dividing
			BL	 DIVU			;perform the division
			PUSH {R1}			;push the value of R1 into a stack
			CMP  R0, #0			;compare to check if R0 is 0
			BEQ  CHECK0			;if it is then branch to the finish
			B    PUTNUMLP		;if not then branch back to putnum for another iteration
			
			
CHECK0		CMP  R3, #0			;compare R3 with 0 to check if the loop is finished
			BEQ  REALEND		;if it's 0 then branch to the realending of the subroutine
			POP  {R0}			;pop the top item of the stack
			ADDS R0, #48	 	;adds R0 by 48 to match ascii table
			BL   PutChar		;print out the character onto te terminal
			SUBS R3, #1			;downcounting the counter
			B    CHECK0			;branch to check0 for another iteration

			
REALEND		
			POP  {R0-R4, PC}	;pop the registers and PC
			ENDP				;end the subroutine.

		
InitQueue	PROC {R0-R14}
;initalizes the queue
;Parameters
;	Inputs:
;		R2:	the size of the queue
;		R1: R1 is the starting address of the queue
;		R0: The actual data in R1. 
;	Outputs:
;		no outputs

			
			PUSH	{R0-R3}
			STR		R0, [R1, #IN_PTR]		;storing the starting address of R0 into in pointer
			STR		R0, [R1, #OUT_PTR]		;storing the starting address of R0 into out pointer
			STR		R0, [R1, #BUF_STRT]		;storing the starting address of R0 into buffer start
			ADDS    R3, R0, R2				;adding the buffer size and the start address of R0 to get to the address where it goes over the queue size
			STR		R3, [R1, #BUF_PAST]		;storing that address into the buffer past
			STRB	R2, [R1, #BUF_SIZE]		;storing the size of the queue into R1
			MOVS    R3, #0x00				;moving 0x00 into R3
			STRB	R3, [R1, #NUM_ENQD]		;initalizeing the num_enqd to 0
			POP		{R0-R3}
			
			BX 		LR
			ENDP
				
Enqueue		PROC {R0-R14}		
;enqueues an item into the queue if the queue is not full.
;Parameters
;	Inputs:
;		R0: The actual data in R1.
;		R1: R1 is the starting address of the queue		
;	Outputs:
;		PSR C FLAG: Success(0) or Failure(1)
;	Modify:
;		APSR

 
		
			PUSH 	{R0-R7, LR}
			LDRB 	R2, [R1, #NUM_ENQD]		;gets the number of enqueued
			LDRB	R3, [R1, #BUF_SIZE]		;gets the buffer size
			CMP		R2, R3					;compares the number of enqueue and buffer size
			BHS		FAILC					;branch to FAILC which sets C flag to 1
			ADDS	R2, #1					;adds number enqueued by 1
			STRB	R2, [R1, #NUM_ENQD]		;storing the new number enqueued 
			LDR		R2, [R1, #IN_PTR]		;loading the in_ptr address
			STRB	R0, [R2, #0]			;enqueue R0 into the queue
			LDR		R3, [R1, #BUF_PAST]		;loads the address of buf_past
			ADDS    R2,	#1					;add 1 to the address of in pointer
			STR     R2, [R1, #IN_PTR]		;store the next address of in pointer in the queue record
			CMP     R3, R2					;comparing the buffer past and the in pointer
			BHI		FinishQue				;if the buffer past is higher than the in pointer than branch to finish
			LDR		R2, [R1, #BUF_STRT]		;loads the value of buf start 
			STR		R2, [R1, #IN_PTR]		;saves the value from buf start into in pointer
			B		FinishQue

FAILC
			SETC
			B		EndQue
FinishQue	
			RMVC	
EndQue
			POP		{R0-R7, PC}
			ENDP

Dequeue 	PROC {R1-R14}
;dequeues an item into the queue if the queue is not full.
;Parameters
;	Inputs:
;		R1: R1 is the starting address of the queue		
;	Outputs:
;		R0: character dequeued
;		PSR C FLAG: Success(0) or Failure(1)
;	Modify:
;		R0
;		APSR

			PUSH	{R1-R7, LR}
			LDRB	R2, [R1, #NUM_ENQD]		;gets the number of items enqueued the queue
			CMP		R2, #0					;compare R2 and 0
			BLS		GetOut					;branch to GetOut which is when item in queue is 0.
			LDR		R3, [R1, #OUT_PTR]		;get the address of the out pointer
			LDRB	R0, [R3, #0]			;get the value of the thing that's being dequeued
			SUBS	R2, #1					;subtracting the number enqueued by 1
			STRB	R2, [R1, #NUM_ENQD]		;store the new value of number enqueued
		
			ADDS    R3, #1					;increment the out_ptr to the next address
			LDR		R4, [R1, #BUF_PAST]		;load the address of the buffer past
			STR		R3, [R1, #OUT_PTR]		;store the new adress to out pointer
			CMP		R4, R3					;compares the out pointer and the buffer past address
			BHI		Success					;branch to Success which takes you out of the subroutine
			LDR		R2, [R1, #BUF_STRT]		;loads the value of buf start 
			STR		R2, [R1, #OUT_PTR]		;saves the value from buf start into out pointer if the out pointer is higher than the buff past
			B		Success
		
		
GetOut
			SETC							;setting C flag
			B		FinDeque				;branch to FinDeque which will take you out of the subroutine
Success 
			RMVC							;clearing the C flag
FinDeque
			POP		{R1-R7, PC}
			ENDP

PutNumHex 	PROC {R0-R14}
;gets a register and prints the value in hex
;Parameters
;	Inputs:
;		R0: Unsigned word value to print in hex
;	Outputs:
;		PSR Flags
;	Modify:
;		R0
;		APSR
			PUSH	{R0-R7, LR}				;pushing all the registers 
			MOVS	R3, #0					;making a counter
			MOVS    R2, R0					;store R0 into R2
NextHex		CMP     R3, #8					;comparing if the loop performed 8 times
			BEQ		Print					;branch to print which prints the stack
			MOVS    R0, R2
			MOVS	R1, #0xF				;moving F into R1 to perform a masking
			ANDS	R0, R0, R1				;masking R0 to get the nibble value
			CMP     R0, #10					;compare if the value is less than 10
			BLO		AddLow					;branch to addlow which pushes the value of R0
			ADDS	R0, #55					;converting the decimal value to ascii value
			LSRS	R2, R2, #4				;shift the word value right once
			ADDS	R3, #1					;increment the counter by 1
			PUSH	{R0}					;push the value of converted value into the register
			B		NextHex					;branch to NextHex

AddLow		LSRS	R2, R2, #4				;shift the word value right once
			ADDS	R3, #1					;increment the counter by 1
			ADDS    R0, #48
			PUSH	{R0}					;push the value of converted value into the register
			B		NextHex					;branch to NextHex

Print		CMP		R3, #0					;compare R3 and 0
			BEQ		FinNum					;branch to FinNum if the the loop had gone 8 times
			POP		{R0}					;pop the top value of the stack
			BL		PutChar					;branch to PutChar which puts the char into the terminal
			SUBS    R3, #1
			B		Print					;branch back to print

FinNum
			POP		{R0-R7,PC}
			ENDP
				
PutNumUB	PROC {R0-R14}
;prints the text decimal representation of unsigned byte
;Parameters
;	Inputs:
;		R0: Unsigned word value to print the byte value
;	Outputs:
;		PSR Flags
;	Modify:
;		APSR
			PUSH {R0-R7, LR}				;Push the registers 
			MOVS R1, #0xFF					;move FF into R1 for masking
			ANDS R0, R0, R1					;mask R0 to get only 1 byte
			BL	 PutNumU					;branch to PutNumU to print to terminal
			POP	 {R0-R7, PC}				;pop the registers back
			ENDP
			
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
			EXPORT	CONST
			EXPORT	LENGTH
			EXPORT	FAILURE
			EXPORT	ENQUEUECHAR
			EXPORT	SUCCESS
			EXPORT	HELP
			EXPORT	PRINTIN
			EXPORT	PRINTOUT
			EXPORT	PRINTNUM
			EXPORT	STATUS
			EXPORT	DAC0_table_0
			EXPORT	PWM_duty_table_0
;>>>>> begin constants here <<<<<
DAC0_table_0
DAC0_table
			DCW	((DAC0_STEPS - 1) / (SERVO_POSITIONS * 2))
			DCW	(((DAC0_STEPS - 1) * 3) / (SERVO_POSITIONS * 2))
			DCW	(((DAC0_STEPS - 1) * 5)	/ (SERVO_POSITIONS * 2))
			DCW	(((DAC0_STEPS - 1) * 7) / (SERVO_POSITIONS * 2))
			DCW	(((DAC0_STEPS - 1) * 9) / (SERVO_POSITIONS * 2))
			
PWM_duty_table
PWM_duty_table_0 									;include if accessed from C
													;Servo positions from 1 (leftmost) to 5 (rightmost)
			DCW PWM_2ms 							;-50% of range
			DCW ((3*(PWM_2ms-PWM_1ms)/4)+PWM_1ms)	;-25% of range
			DCW (((PWM_2ms-PWM_1ms)/2)+PWM_1ms) 	; 0% of range
			DCW (((PWM_2ms-PWM_1ms)/4)+PWM_1ms) 	;+25% of range
			DCW PWM_1ms 							;+50% of range			

CONST		DCB		"Type a string command (D,E,H,P,S):", NULL		
LENGTH      DCB		"Length:", NULL
FAILURE     DCB     "Failure:", NULL
ENQUEUECHAR DCB		"Character to enqueue:", NULL
SUCCESS		DCB		"Success:", NULL
HELP        DCB		"d (dequeue), e (enqueue), h (help), p (print), s (status)", NULL
PRINTIN		DCB		"In=0x", NULL
PRINTOUT	DCB		"Out=0x", NULL
PRINTNUM    DCB		"Num=", NULL
STATUS      DCB     "Status:", NULL
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Queue		SPACE   4
			ALIGN
Queuerecord	SPACE   18
TxQBuffer   SPACE   MAXSIZE
			ALIGN
TxQRecord   SPACE   18	
RxQBuffer   SPACE   MAXSIZE
			ALIGN	
RxQRecord   SPACE   18
STRING		SPACE 	MAXSIZE

;>>>>>   end variables here <<<<<
            ALIGN
            END