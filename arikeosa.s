		AREA |.text|,CODE,READONLY,ALIGN=2
		THUMB
		EXTERN currentThread
		EXPORT osSchedulerLaunch
		EXPORT SysTick_Handler


;Thread switcher
;save r0,r1,r2,r3,r12,lr,pc,psr
SysTick_Handler

  CPSID  I				     ;Disable interrupt
  PUSH  {R4-R11} 		     ;save r4-r11

;Load R1 with currentPtr
  LDR  R0, = currentThread   ;r0 = pointer to currentPtr
  LDR  R1, [R0]			     ;r1 = currentPtr

  STR  SP, [R1]
  LDR  R1, [R1,#4]		     ;r1 = currentPtr - nextPtr

  STR  R1, [R0]			     ;currentPtr = r1
  STR  R1, [R0]			     ;currentPtr = r1

  LDR  SP, [R1]			     ;SP = currentPtr->stackPtr

  POP  {R4-R11}		         ;restore r4-r11
  CPSIE  I				     ;Enable interrupt

  BX  LR				     ;restore r0,r1,r2,r3,r12,lr,pc,psr

;executed when scheduler is launched
osSchedulerLaunch

  LDR 	R0, = currentThread	 ;r0 = pointer to currentPtr
  LDR 	R2,[R0]			     ;r2 = currentPtr

  LDR  SP, [R2]			     ;SP = currentPtr->stackPtr

  POP  {R4-R11}
  POP  {R0-R3}
  POP  {R12}
  ADD  SP,SP,#4		         ;Discard LR from initial stack
  POP  {LR}			         ;tart location
  ADD  SP,SP,#4		         ;Discard PSR

  CPSIE	I
  BX  LR				     ;start first thread

	ALIGN
	END
