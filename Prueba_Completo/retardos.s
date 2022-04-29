.include "p30F4013.inc"
    
.GLOBAL	_RETARDO_5ms
.GLOBAL	_RETARDO_30ms
.GLOBAL	_RETARDO_100ms
.GLOBAL	_RETARDO_1s
.GLOBAL _RETARDO_50us
    
_RETARDO_50us:
	REPEAT	#100
	    NOP
    
	RETURN
	    
    
;;******************************************************************************
;;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 30ms
;;PARAMETROS: 	NINGUNO
;;RETORNO: 	NINGUNO
;;******************************************************************************
_RETARDO_30ms:

    	    CALL	_RETARDO_5ms
	    CALL	_RETARDO_5ms
	    CALL	_RETARDO_5ms
	    CALL	_RETARDO_5ms
	    CALL	_RETARDO_5ms
	    CALL	_RETARDO_5ms

	    RETURN
		
;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 100ms
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_100ms:

        DO  #19,    CICLO_RETARDO_100ms
        CALL	    _RETARDO_5ms
CICLO_RETARDO_100ms:
        NOP

	RETURN
;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 5ms
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_5ms:
	    PUSH	W0

	    MOV		#3276,			W0
CICLO1_5ms:
	    DEC		W0,				W0
	    BRA		NZ,				CICLO1_5ms

	    POP		W0

	    RETURN

;******************************************************************************
;DESCRICION:	ESTA RUTINA GENERA UN RETARDO DE 1S
;PARAMETROS: 	NINGUNO
;RETORNO: 	NINGUNO
;******************************************************************************
_RETARDO_1s:
	    PUSH	W0
	    PUSH	W1

	    MOV		#10,			W1
CICLO2_1S:
	    CLR		W0
CICLO1_1S:
	    DEC		W0,				W0
	    BRA		NZ,				CICLO1_1S

	    DEC		W1,				W1
	    BRA		NZ,				CICLO2_1S

	    POP		W1
	    POP		W0

	    RETURN

