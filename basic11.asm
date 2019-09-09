;*******************************************************************************
;* Program   : BASIC11.ASM
;* Programmer: Gordon Doughman
;* Purpose   : BASIC interpreter for the 68HC11
;* Language  : Motorola/Freescale/NXP 68HC11 Assembly Language (aspisys.com/ASM11)
;* Note(s)   : This version has no object code differences from the original.
;*           : [Bug(s) not removed unless assembled with BUGFIX conditional.]
;*******************************************************************************
; Adapted to ASM11 by Tony Papadimitriou <tonyp@acm.org> and did the following:
;
; 1. Combined all files into (this) one
; 2. Added macros to emulate all unknown directives of the original assembler
; 3. Reformatted using FASM.EXE utility (http://aspisys.com/fasm.exe)
; 4. Manually improved formatting further
; 5. Changed IF ELSE ENDIF FAIL to ASM11 syntax (#IF #ELSE #ENDIF #FATAL)
; 6. Corrected case for a few symbols, so it also compiles with case-sensitivity
; 7. Replaced some instruction sequences with ASM11 internal macros for clarity
; 8. Added CRC verification display message at the end of this file
; 9. Slowly converting to 100% ASM11 idiomatic syntax [WORK-IN-PROGRESS]
;10. Placed on GitHub https://github.com/tonypdmtr/BASIC11/blob/master/basic11.asm
;===============================================================================
; I have had many, many requests from M68HC11 users over the
; years for the source code for BASIC11. I was very reluctant to
; release the source code for a number of reasons. First, because
; BASIC11 was nothing more than a skunk works project, most of
; the programming was done late at night and on weekends.
; Consequently, no documentation exists on the internal operation
; of the interpreter and proper source code documentation
; practices were completely ignored. I would not approach a
; project like this in a similar manner today.
;
; Second, because my job as a Field Applications Engineer is to
; provide support to a specific group of customers, I feared that
; releasing the source code would produce an avalanche of support
; questions that I would not have time to answer. In turn this
; might upset the customers whose questions I could not answer.
;
; However, after much consideration, I have decided to release
; the BASIC11 source code to the public. Please understand that I
; cannot provide any help or assistance in adapting BASIC11 to
; another hardware environment or provide help in understanding
; the internal workings of BASIC11. You're on your own.
;
; Regards,
; Gordon Doughman
;===============================================================================

#ifdef ?
  #Hint +===================================================
  #Hint | Available conditionals (for use with -Dx option)
  #Hint +===================================================
  #Hint | BUGFIX: Fix any known bugs
  #Hint +===================================================
  #Fatal Run ASM11 -Dx (where x is any of the above)
#endif
                    #Macro
                    #SpacesOff
                    #ExtraOn
                    #CaseOn
                    #OptRelOff
                    #OptRtsOff

Title               macro
                    mset      #
                    #Message  Processing \@~1~\@
                    #Page
                    endm

;*******************************************************************************
; Macros
;*******************************************************************************

token               macro     ['String'],Token,Subroutine
                    mreq      1,2:['String'],Token,Subroutine
          #if :nn = 3
                    fcs       ~1~
                    mdel      1
          #endif
          #ifstr ~1~
                    fcs       ~1~
          #else
                    fcb       ~1~
          #endif
                    fdb       ~2~
                    endm

;*******************************************************************************

VERSION             equ       155
BUS_KHZ             def       2000

;*******************************************************************************
;
;                   OPT       NOL

HC11                def       1

;                   include   'defines.inc'
                    title     DEFINES

          #ifnz HC11                              ;HC11EVB defines
ROMBEG              equ       $E000
ACIAST              equ       $9800
          #else                                   ;6809/FLEX development defines
ROMBEG              equ       $8000
ACIAST              equ       $E010
          #endif
ROMSIZE             equ       $2000
ACIADT              equ       ACIAST+1
SWPRE               equ       2                   ; SOFTWARE PRESCALER VALUE.

                    #temp
                    next      :temp,2             ;SBASBEG
SBASEND             next      :temp,2
SVARBEG             next      :temp,2
SVAREND             next      :temp,2
                    next      :temp,2             ;SHILINE
AUTOSTF             next      :temp
SSTART              next      :temp

;***** hc11 (device dependant) defines *****/

EEPBASAD            equ       $B600               ; EEPROM base address
MAXEESUB            equ       255                 ; maximum EEP subscript

;          I/O Register Offsets From The Base Address

PPROG               equ       $3B                 ; EEPROM programing control register
ADCTL               equ       $30                 ; A-TO-D control/status register
ADR1                equ       $31                 ; A/D result register 1
ADR2                equ       $32                 ; A/D result register 2
ADR3                equ       $33                 ; A/D result register 3
ADR4                equ       $34                 ; A/D result register 4
PORTAIO             equ       $00                 ; PORTA I/O register
PORTBIO             equ       $04                 ; PORTB I/O register
PORTCIO             equ       $03                 ; PORTC I/O register
PORTDIO             equ       $08                 ; PORTD I/O register
PORTEIO             equ       $0A                 ; PORTE I/O register
TCNT                equ       $0E                 ; TIMER/COUNTER register
TOC1REG             equ       $16                 ; TIMER Output Compare 1 register
TFLAG1              equ       $23                 ; TIMER Flag #1 register
TMSK1               equ       $22                 ; TIMER Mask #1 register
TMSK2               equ       $24                 ; TIMER Mask #2 register
OPTION              equ       $39                 ; OPTION select register
BAUD                equ       $2B                 ; SCI baud rate select register
SCCR1               equ       $2C                 ; SCI control register #1
SCCR2               equ       $2D                 ; SCI control register #2
SCSR                equ       $2E                 ; SCI status register
SCDR                equ       $2F                 ; SCI transmit/recieve data register
PACNT               equ       $27                 ; PACC count register
PACTL               equ       $26                 ; PACC control register
TFLG2               equ       $25                 ; TIMER Flag #2 register
INIT                equ       $3D                 ; INIT (Base address of RAM & I/O Regs) Register

;***** misc. defines *****/

EOL                 equ       13                  ; end of line marker
CR                  equ       13                  ; same as EOL
LF                  equ       10                  ; linefeed character
BS                  equ       8                   ; backspace character
SPC                 equ       ' '                 ; space character
MIDEOL              equ       ':'                 ; mid EOL character
COMMA               equ       ','                 ; comma
SEMI                equ       ';'                 ; semicolon
NUM                 equ       1                   ; getvar return flag
STRING              equ       2                   ; getvar return flag
NULL                equ       0                   ; null value
CTRL_C              equ       3                   ; control-c (break character)

IBUFLEN             equ       80                  ; input buffer max length
TBUFLEN             equ       128                 ; token buffer max length
SWSTKSize           equ       592

OPSLEN              equ       30                  ; operator stack length
NUMSLEN             equ       60                  ; operand stack length
FORSLEN             equ       80                  ; FOR..NEXT stack length
WHSLEN              equ       16                  ; WHILE..ENDWH stack length
GOSLEN              equ       16                  ; GOSUB stack length

;***** define error codes *****/

                    #temp     1
LINRANG             next      :temp               ; line number range error
SYTXERR             next      :temp               ; syntax error
IVEXPERR            next      :temp               ; invalid expression error
UPARNERR            next      :temp               ; unbalanced parentheses error
DTMISERR            next      :temp               ; data type mismatch error
OPRTRERR            next      :temp               ; illegal operator error
ILVARERR            next      :temp               ; illegal variable error
ILTOKERR            next      :temp               ; illegal token error
OMEMERR             next      :temp               ; out of memory error
INTOVERR            next      :temp               ; integer overflow error
IVHEXERR            next      :temp               ; invalid hex digit error
HEXOVERR            next      :temp               ; hex number overflow
MISQUERR            next      :temp               ; missing quote error
MPARNERR            next      :temp               ; missing open or closing parenthisis
IONSYERR            next      :temp               ; "ON" syntax error
MTHENERR            next      :temp               ; missing "THEN" in "IF" statement
MTOERR              next      :temp               ; missing "TO" in "FOR" statement
LINENERR            next      :temp               ; line number error
IDTYERR             next      :temp               ; illegal data type error
EXPCXERR            next      :temp               ; expression too complex (xlator token buff ovf.)
MCOMAERR            next      :temp               ; missing comma
MCMSMERR            next      :temp               ; missing comma or semicolon
MSTKOERR            next      :temp               ; math stack overflow error
UNDIMERR            next      :temp               ; undimentioned array error
SUBORERR            next      :temp               ; subscript out of range error
ZDIVERR             next      :temp               ; divide by zero error
LNFERR              next      :temp               ; line not found error
GOSOVERR            next      :temp               ; too many nested GOSUB's
RWOGERR             next      :temp               ; RETURN w/o GOSUB error
WHSOVERR            next      :temp               ; too many active WHILE's
ENDWHERR            next      :temp               ; ENDWH statement w/o WHILE
ONARGERR            next      :temp               ; ON argument is negative, zero, or too large
NOSUBERR            next      :temp               ; non-subscriptable variable found in DIM statem.
REDIMERR            next      :temp               ; variable has already been DIMensioned
FORNXERR            next      :temp               ; too many active FOR -- NEXT loops active
MFRNXERR            next      :temp               ; mismatched FOR -- NEXT statements.
CNTCNERR            next      :temp               ; can't continue
ODRDERR             next      :temp               ; out of data in read or restore statement
NEGSUBER            next      :temp               ; negative subscripts not allowed
EESUBERR            next      :temp               ; EEP() subscript negative or > 200
PRFUNERR            next      :temp               ; function only allowed in print statement
TABARGER            next      :temp               ; argument <0 or >255 in TAB() function
CHRARGER            next      :temp               ; argument <0 or >255 in CHR$() function
OVDV0ERR            next      :temp               ; overflow or /0 error in FDIV() function
INVCHERR            next      :temp               ; invalid channel number in ADC() function
PRTASERR            next      :temp               ; tried to assign a value <0 or >255 to PORT(X)
ILPRTERR            next      :temp               ; illegal port error
ILLIOERR            next      :temp               ; illegal I/O vector number <0 or >7
UNINIERR            next      :temp               ; uninitialized I/O vector
HEX2AERR            next      :temp               ; argument <0 or >255 in HEX2 function
NOTALERR            next      :temp               ; statement not allowed in direct mode
NOTINTER            next      :temp               ; an RETI statement executed when not in interrupt
PACCARGE            next      :temp               ; tried to assign a value of <0 or >255 to PACC
INTMODER            next      :temp               ; interrupt or count mode error in ONPACC
EETOSMAL            next      :temp               ; program storage EEPROM is Too Small

; mathematical operator tokens

OPARNTOK            equ       $10                 ; '('
CPARNTOK            equ       $11                 ; ')'
ANDTOK              equ       $20                 ; 'AND'
ORTOK               equ       $21                 ; 'OR'
EORTOK              equ       $22                 ; 'EOR'
LTTOK               equ       $30                 ; '<'
GTTOK               equ       $31                 ; '>
LTEQTOK             equ       $32                 ; '<='
GTEQTOK             equ       $33                 ; '>='
EQTOK               equ       $34                 ; '='
NOTEQTOK            equ       $35                 ; '<>'
PLUSTOK             equ       $40                 ; '+'
MINUSTOK            equ       $41                 ; '-'
SPLUSTOK            equ       $42                 ; '+'
MULTTOK             equ       $50                 ; '*'
DIVTOK              equ       $51                 ; '/'
MODTOK              equ       $52                 ; '%'
PWRTOK              equ       $60                 ; '^'
INDIRTOK            equ       $70                 ; '@'
NOTTOK              equ       $71                 ; 'NOT'
NEGTOK              equ       $72                 ; '-' (uniary minus)

; keyword tokens

LETTOK              equ       $01                 ; LET
IMLETTOK            equ       $02                 ; implied LET
PRINTTOK            equ       $03                 ; PRINT
FORTOK              equ       $04                 ; FOR
NEXTTOK             equ       $05                 ; NEXT
TRONTOK             equ       $06                 ; TRON
TROFFTOK            equ       $07                 ; TROFF
POKETOK             equ       $08                 ; POKE
DIMTOK              equ       $09                 ; DIM
REMTOK              equ       $0A                 ; REM
PACCTOK             equ       $0B                 ; PACC
DATATOK             equ       $0C                 ; DATA
READTOK             equ       $0D                 ; READ
RESTRTOK            equ       $0E                 ; RESTORE
GOSUBTOK            equ       $0F                 ; GOSUB
GOTOTOK             equ       $12                 ; GOTO
ONTOK               equ       $13                 ; ON
RETNTOK             equ       $14                 ; RETURN
IFTOK               equ       $15                 ; IF
INPUTTOK            equ       $16                 ; INPUT
STOPTOK             equ       $17                 ; STOP
ENDTOK              equ       $18                 ; END
WHILETOK            equ       $19                 ; WHILE
ENDWHTOK            equ       $1A                 ; ENDWH
EEPTOK              equ       $1B                 ; EEP
PORTATOK            equ       $1C                 ; PORTA
PORTBTOK            equ       $1D                 ; PORTB
PORTCTOK            equ       $1E                 ; PORTC
PORTDTOK            equ       $1F                 ; PORTD
INBYTTOK            equ       $23                 ; INBYTE
TIMETOK             equ       $24                 ; TIME
ONTIMTOK            equ       $25                 ; ONTIME
ONIRQTOK            equ       $26                 ; ONIRQ
RETITOK             equ       $27                 ; RETI
ONPACTOK            equ       $28                 ; ONPACC
SLEEPTOK            equ       $29                 ; SLEEP
RTIMETOK            equ       $2A                 ; RTIME
FUNCTFLG            equ       $36                 ; function flag byte
TOTOK               equ       $37                 ; TO
THENTOK             equ       $38                 ; THEN
ELSETOK             equ       $39                 ; ELSE
STEPTOK             equ       $3A                 ; STEP

; function tokens
                    #temp     1
FDIVTOK             next      :temp               ; FDIV
CHRTOK              next      :temp               ; CHR$
ADCTOK              next      :temp               ; ADC
ABSTOK              next      :temp               ; ABS
RNDTOK              next      :temp               ; RND
SGNTOK              next      :temp               ; SGN
TABTOK              next      :temp               ; TAB
CALLTOK             next      :temp               ; CALL
PEEKTOK             next      :temp               ; PEEK
FEEPTOK             next      :temp               ; EEP
HEXTOK              next      :temp               ; HEX
FPRTATOK            next      :temp               ; PORTA
FPRTBTOK            next      :temp               ; PORTB
FPRTCTOK            next      :temp               ; PORTC
FPRTDTOK            next      :temp               ; PORTD
FPRTETOK            next      :temp               ; PORTE
FTIMETOK            next      :temp               ; TIME
HEX2TOK             next      :temp               ; HEX2
FPACCTOK            next      :temp               ; PACC

; numerical/variable tokens

FVARTOK             equ       $81                 ; floating point variable address
SVARTOK             equ       $82                 ; string variable address
IVARTOK             equ       $84                 ; integer variable address

FAVARTOK            equ       $91                 ; floating point array
SAVARTOK            equ       $92                 ; string array
IAVARTOK            equ       $94                 ; integer array

FCONTOK             equ       $A1                 ; floating point constant
SCONTOK             equ       $A2                 ; string constant
LCONTOK             equ       $A8                 ; line # constant
ICONTOK             equ       $A4                 ; integer constant

ISIZ                equ       2                   ; number of bytes in integer variable
SSIZ                equ       3                   ; number of bytes in string variable
FSIZ                equ       5                   ; number of bytes in f.p. variable
ASIZ                equ       2                   ; number of bytes for array variable in dictionary

; misc. tokens

MSCNTOK             equ       $7F                 ; multiple space count token
SSCNTOK             equ       $7E                 ; single space token
EOLTOK              equ       $7D                 ; end of line token
COMMATOK            equ       $7C                 ; ,
SEMITOK             equ       $7B                 ; ;
MEOLTOK             equ       $7A                 ; :
EQUALTOK            equ       $79                 ; '='
PNUMTOK             equ       $78                 ; '#'

JMPOP               equ       $7E                 ; OP-CODE FOR "JMP" (USED TO INITIALIZE INTERRUPT TABLE)

;*******************************************************************************
                    #RAM                          ; define variables
;*******************************************************************************
                    org       $0000

;               char

IBUFPTR             rmb       2                   ; input buffer pointer
TBUFPTR             rmb       2                   ; token buffer pointer

;          the next 5 variables must remain grouped togeather

BASBEG              rmb       2                   ; start of basic program area
BASEND              rmb       2                   ; end of basic program
VARBEGIN            rmb       2                   ; start of variable storage area
VAREND              rmb       2                   ; end of variable storage area
HILINE              rmb       2                   ; highest line number in program buffer

BASMEND             rmb       2                   ; physical end of basic program memory
VARMEND             rmb       2                   ; physical end of variable memory

;               int

FIRSTLIN            rmb       2                   ; first line to list
LASTLIN             rmb       2                   ; last line to list
INTPTR              rmb       2                   ; integer pointer

;               short

ERRCODE             rmb       1                   ; error code status byte
IMMED               rmb       1                   ; immediate mode flag
BREAKCNT            equ       *                   ; also use for break check count
COUNT               equ       *                   ; count used in ESAVE & ELOAD routines
IFWHFLAG            rmb       1                   ; translating IF flag
TRFLAG              rmb       1                   ; trace mode flag
CONTFLAG            rmb       1                   ; continue flag
RUNFLAG             rmb       1                   ; indicates we are in the run mode
PRINTPOS            rmb       1                   ; current print position
NUMSTACK            rmb       2                   ; numeric operand stack pointer
OPSTACK             rmb       2                   ; operator stack pointer
FORSTACK            rmb       2                   ; FOR stack pointer
WHSTACK             rmb       2                   ; WHILE stack pointer
GOSTACK             rmb       2                   ; GOSUB stack pointer
CURLINE             rmb       2                   ; line # that we are currently interpreting
ADRNXLIN            rmb       2                   ; address of the next line
STRASTG             rmb       2                   ; dynamic string/array pool pointer
FENCE               rmb       2                   ; varend fence in case of an error in xlation
IPSAVE              rmb       2                   ; interpretive pointer save for "BREAK"
DATAPTR             rmb       2                   ; pointer to data for read statement
RANDOM              rmb       2                   ; random number/seed
DEVNUM              rmb       1                   ; I/O device number
TIMEREG             rmb       2                   ; TIME register
TIMECMP             rmb       2                   ; TIME compare register
TIMEPRE             rmb       1                   ; software prescaler for TIME
ONTIMLIN            rmb       2                   ; ONTIME line number to goto
ONIRQLIN            rmb       2                   ; ONIRQ line number to goto
ONPACLIN            rmb       2                   ; ONPACC line number to goto
XONCH               rmb       1                   ; XON character for printer
XOFFCH              rmb       1                   ; XOFF character for printer
SCURLINE            rmb       2                   ; used to save CURLINE during int. processing
SADRNXLN            rmb       2                   ; used to save ADRNXLIN during int. processing
INBUFFS             rmb       2                   ; pointer to the start of the input buffer
TKNBUFS             rmb       2                   ; pointer to the start of the token buffer

EOPSTK              rmb       2                   ; end of operator stack
STOPS               rmb       2                   ; start of operator stack
ENUMSTK             rmb       2                   ; end of operand stack
STNUMS              rmb       2                   ; start of operand stack
EFORSTK             rmb       2                   ; end of FOR - NEXT stack
STFORSTK            rmb       2                   ; start of FOR - NEXT stack
EWHSTK              rmb       2                   ; end of WHILE stack
STWHSTK             rmb       2                   ; start of WHILE stack
EGOSTK              rmb       2                   ; end of GOSUB stack
STGOSTK             rmb       2                   ; start of GOSUB stack
IOBaseV             rmb       2                   ; Address vector for I/O Registers
DNAME               rmb       3                   ; Place to put the variable name when doing a dump command
SUBMAX              rmb       2                   ;
SUBCNT              rmb       2                   ;
TOKPTR              rmb       2                   ; token pointer (used for list command)
VarSize             rmb       2                   ; used by the line editor. size of the variable table
          #if * > $9E
                    #Fatal    Ran out of zero-page RAM
          #endif
                    org       $009E

CONSTAT             rmb       3                   ; GET CONSOLE STATUS FOR BREAK ROUTINE.
INCONNE             rmb       3                   ; GET BYTE DIRECTLY FROM CONSOLE FOR BREAK ROUTINE.

INTABLE             rmb       2*8                 ; RESERVE SPACE FOR 8 DIFFERENT INPUT ROUTINES.
OUTABLE             rmb       2*8                 ; RESERVE SPACE FOR 8 DIFFERENT OUTPUT ROUTINES.

; START OF RAM INTERRUPT VECTORS.

RAMVECTS            equ       *
SCISS               rmb       3                   ; SCI SERIAL SYSTEM.
SPITC               rmb       3                   ; SPI TRANSFER COMPLETE.
PACCIE              rmb       3                   ; PULSE ACCUMULATOR INPUT EDGE.
PACCOVF             rmb       3                   ; PULSE ACCUMULATOR OVERFLOW.
TIMEROVF            rmb       3                   ; TIMER OVERFLOW.
TOC5                rmb       3                   ; TIMER OUTPUT COMPARE 5.
TOC4                rmb       3                   ; TIMER OUTPUT COMPARE 4.
TOC3                rmb       3                   ; TIMER OUTPUT COMPARE 3.
TOC2                rmb       3                   ; TIMER OUTPUT COMPARE 2.
TOC1                rmb       3                   ; TIMER OUTPUT COMPARE 1.
TIC3                rmb       3                   ; TIMER INPUT CAPTURE 3.
TIC2                rmb       3                   ; TIMER INPUT CAPTURE 2.
TIC1                rmb       3                   ; TIMER INPUT CAPTURE 1.
REALTIMI            rmb       3                   ; REAL TIME INTERRUPT.
IRQI                rmb       3                   ; IRQ INTERRUPT.
XIRQ                rmb       3                   ; XIRQ INTERRUPT.
SWII                rmb       3                   ; SOFTWARE INTERRUPT.
ILLOP               rmb       3                   ; ILLEGAL OPCODE TRAP.
COP                 rmb       3                   ; WATCH DOG TIMER FAIL.
CMF                 rmb       3                   ; CLOCK MONITOR FAIL.
;                   include   'basiclb1.mod'
;*******************************************************************************
                    #ROM
;*******************************************************************************
                    org       ROMBEG
                    title     BASICLB1
;*******************************************************************************
;                                                                              *
;                      MC68HC11 BASIC INTERPRETER                              *
;                                                                              *
;                             WRITTEN BY:                                      *
;                                                                              *
;                           GORDON DOUGHMAN                                    *
;                                                                              *
;                        COPYRIGHT 1985-1990 BY                                *
;                                                                              *
;                           GORDON DOUGHMAN                                    *
;                                                                              *
;*******************************************************************************
;
;
;
;       include "1.DEFINES.C"
;
; *main()
; *{
; initvars();            initialize all variables & pointers
; outheader();           send startup message to console
; outrdy();              output ready message

                    jmp       POWERUP

;*******************************************************************************

MAIN                proc
                    jsr       INITVARS            ; INITIALIZE ALL INTERNAL VARIABLES.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; get the auto start flag.
                    cmpa      #$55
                    bne       NormalStart@@
                    cli                           ; ALLOW ALL INTERRUPTS TO BE SERVICED.
                    jsr       CRUN
NormalStart@@       jsr       OUTHEADR            ; PRINT HEADER.
MainLoop            equ       *
Loop@@              ldd       RAMStart            ; RESET STACK VALUE.
                    addd      RAMSize
                    xgdx
                    txs
                    cli                           ; (J.I.C.)
                    clrd
                    std       TIMECMP             ; DON'T ALLOW "ONTIME" INTERRUPTS TO OCCUR.
                    std       ONIRQLIN            ; DON'T ALLOW "ONIRQ" INTERRUPTS TO OCCUR.
                    std       ONPACLIN            ; DON'T ALLOW "PACC" INTERRUPTS TO OCCUR.
                    jsr       OUTRDY              ; PRINT READY MESSAGE.

; while(1)               do forever
; {
;  outprmpt();           output prompt
;  getline();            getline from console
;  skipspcs();           ignore leading spaces in input buffer
;  if(chckcmds()) continue;           check for commands
;  parse();              translate/execute line
;  if(errcode) rpterr(); if an error occured somewhere, report it.
;  errcode=0;            clear error status
; }
; *}

Prompt@@            clr       IMMED               ; CLEAR THE IMMEDIATE MODE FLAG.
                    clr       ERRCODE             ; CLEAR THE ERROR CODE BYTE.
                    clr       RUNFLAG             ; CLEAR THE RUN MODE FLAG.
                    jsr       OUTPRMPT            ; SEND PROMPT TO CONSOLE.
                    jsr       GETLINE             ; GO GET LINE FROM OPERATOR.
                    jsr       SKIPSPCS            ; SKIP ANY LEADING SPACES.
                    jsr       CHCKCMDS            ; GO CHECK FOR ANY COMMANDS.
                    cpd       #0                  ; WERE THERE ANY?
                    bne       Loop@@              ; YES. CONTINUE.
                    ldx       VAREND              ; SAVE CURRENT END OF VARIABLE AREA IN CASE LINE
                    stx       FENCE               ; HAS AN ERROR IN IT. (SO NO SPURIOUS VARIABLES
                                                  ; ARE ALLOCATED)
                    jsr       PARSE
                    tst       IMMED               ; DID WE EXECUTE IN IMMEDIATE MODE?
                    bne       Loop@@              ; YES. PRINT READY MESSAGE.
                    bra       Prompt@@            ; NO. JUST PRINT PROMPT.

MAIN3               ldx       FENCE               ; GET THE VAREND FENCE.
                    clr       ,x                  ; MARK "OLD" END OF VARIABLE AREA IN CASE ANY
                                                  ; VARIABLES WERE ALLOCATED.
                    stx       VAREND              ; RESTORE THE POINTER.
                    bra       Loop@@              ; CONTINUE AFTER ERROR.

;*******************************************************************************
;        ***** skipspcs() *****
;
; *skipspcs()
; *{
; while(*ibufptr==SPC) ++ibufptr;
; return;
; *}

SKIPSPCS            proc
Loop@@              jsr       GETCHR
                    cmpa      #SPC
                    bne       Done@@
                    jsr       INCIBP
                    bra       Loop@@
Done@@              rts

;*******************************************************************************
;        ***** outheader *****
;
; *outheader()
; *{
; pl("BASIC11 v1.1");
; nl();
; nl();
; pl("Written by Gordon Doughman");
; nl();
; *}

OUTHEADR            proc
                    ldx       #Msg@@
                    jmp       PL

Msg@@               fcb       CR,LF,CR,LF
                    fcc       'BASIC11 v{VERSION(2)}',CR,LF
                    fcc       'Copyright 1985-1990 by',CR,LF
                    fcs       'Gordon Doughman',CR,LF

;*******************************************************************************
;        ***** outrdy() *****
;
; *outrdy()
; *{
; nl();
; pl("READY");
; return;
; *}

OUTRDY              proc
                    ldx       #Msg@@
                    jmp       PL

Msg@@               fcs       CR,LF,'READY',CR,LF

;*******************************************************************************
;        ***** getline() *****

; *getline()
; *{
; short chrcnt;
; char c;
; chrcnt=IBUFLEN;
; ibufptr=inbuff;

GETLINE             proc
                    ldb       #IBUFLEN-1
                    ldx       INBUFFS

; while((c=inbyte())!=EOL && (chrcnt>0))
; {

Loop@@              jsr       INBYTE
                    cmpa      #EOL
                    beq       Done@@
                    tstb
                    beq       Done@@

;  if(c==BS)
;  {
;   if(ibufptr!=inbuff)
;   {
;    ibufptr-=1;         point to previous buffer location
;    outbyte(SPC);        erase character on screen
;    outbyte(BS);        back cursor up
;    chrcnt+=1;
;   }
;   else
;    continue;
;   }
                    cmpa      #BS
                    bne       Break?@@
                    cmpb      #IBUFLEN-1
                    beq       Done@@
                    dex
                    lda       #SPC
                    jsr       OUTBYTE
                    lda       #BS
                    jsr       OUTBYTE
                    incb

;  if(c < SPC) continue;  ignore all other control characters

Break?@@            cmpa      #CTRL_C             ; IS IT A "BREAK"?
                    bne       Go@@                ; NO. GO PROCESS THE CHARACTER.
                    inc       CONTFLAG            ; DONT ALLOW A "CONT".
                    jmp       ?DoBreak            ; GO DO A BREAK.

Go@@                cmpa      #SPC
                    blo       Loop@@

;  *ibufptr=c;            save character in input buffer
;  ibufptr+=1;           point to next location
;  chrcnt-=1;
; }
                    sta       ,x
                    inx
                    decb
                    bne       Loop@@

; *ibufptr=EOL;          put EOL in input buffer
; ibufptr=inbuff;        initialize the input buffer pointer
; return;
; *}

Done@@              lda       #EOL
                    sta       ,x
                    ldx       INBUFFS
                    stx       IBUFPTR
                    rts

;*******************************************************************************

ToUpper             proc
                    cmpa      #'a'                ; is the character less than a lower case 'a'?
                    blo       Done@@
                    cmpa      #'z'
                    bhi       Done@@
                    anda      #$df
Done@@              rts

;*******************************************************************************
;        ***** outprmpt() *****

; *outprmpt()
; *{
; nl();          go to new line
; outbyte('#');  send prompt to console
; return;
; *}

OUTPRMPT            proc
                    ldx       #Msg@@
                    bra       PL

Msg@@               fcs       CR,LF,'#'

;*******************************************************************************
;        ***** nl() *****

; nl()                    send CR/LF to console
; *{
; outbyte(CR);
; outbyte(LF);
; return;
; *}

NL2                 proc
                    bsr       NL
;                   bra       NL

;*******************************************************************************

NL                  proc
                    ldx       #Msg@@
                    bsr       PL
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    rts

Msg@@               fcs       LF,CR

;*******************************************************************************
;        ***** pl() *****
;
; *pl(ptr)                 send line to console
; *char *ptr;
; *{
; int k; char c;
; k=0;
; while(c=ptr[k++]) outbyte(c);
; return;
; *}

PL                  proc
Loop@@              lda       ,x
                    beq       Done@@
                    jsr       OUTBYTE
                    inx
                    bra       Loop@@
Done@@              rts

;*******************************************************************************
;        ***** parse() *****

; *parse()
; *{
; int num;
; tbufptr=tknbuf;                initialize the token buffer pointer

PARSE               proc
                    ldx       TKNBUFS             ; Get the start of the token buffer
                    stx       TBUFPTR

; if(num=getlinum())             get line number if present
; {
;  if(*ibufptr==EOL)             was line # followed by CR?
;  {                             yes.
;   delline(num);                go delete the line from the prog buffer
;   return;
;  }
;  immed=0;                      flag as not immediate
;  putlinum(num);                put line number in buffer
; }
;  else immed=1;                  if no line # flag as immediate*/

                    bsr       GETLINUM
                    bcc       Eol?@@
                    psha
                    jsr       GETCHR
                    cmpa      #EOL
                    pula
                    bne       Go@@
                    jsr       DELLINE
                    ldx       VAREND
                    inx
                    stx       STRASTG
Done@@              rts

Go@@                clr       IMMED
Loop@@              bsr       PUTLINUM
                    bra       RunOrSave@@

Eol?@@              jsr       GETCHR
                    cmpa      #EOL
                    beq       Done@@
                    lda       #1
                    sta       IMMED
                    ldd       #0
                    bra       Loop@@

; if(errcode) return;             if line number error, return
; xlate();                        if translation error, return
; if(errcode) return;
; if(immed) runline();            if immediate mode run 1 line
;  else storlin();                if not store pgm line
; return;                         go get next line
; *}

RunOrSave@@         jsr       XLATE
                    tst       IMMED
                    beq       Save@@
                    jmp       RUNLINE             ; GO RUN THE LINE & RETURN.

Save@@              jsr       STORLIN             ; GO STORE LINE & RETURN.
                    ldx       VAREND
                    inx
                    stx       STRASTG
                    rts

;*******************************************************************************
;        ***** getlinum *****

; *getlinum()
; *{
; int num;
; num=0;

GETLINUM            proc
                    pshy
                    clra
                    psha:2
                    tsy

; if(numeric(*ibufptr)==0) return(0);    if 1st char not numeric, rtn 0

                    ldx       IBUFPTR
                    lda       ,x
                    bsr       NUMERIC
                    bcc       NotNum@@

; while(numeric(*ibufptr))       while *ibufptr is numeric
; {
;  num=num*10+(*ibufptr-'0');    get a digit
;  ibufptr++;                    advance input buffer pointer
;  if(num<=0) { errcode=LINRANG; return(0); }
; }
; return(num);
; *}

Loop@@              lda       ,x
                    bsr       NUMERIC
                    bcs       Cont@@
                    sec
                    ldd       ,y
                    bne       NotNum@@
                    lda       #LINENERR
                    bra       Fail@@

NotNum@@            ins:2
                    puly
                    stx       IBUFPTR
                    rts

Cont@@              bsr       ADDDIG
                    bpl       Loop@@
                    lda       #LINRANG
Fail@@              jmp       RPTERR

;*******************************************************************************

ADDDIG              proc
                    ldd       ,y
                    asld:2
                    addd      ,y
                    asld
                    std       ,y
                    ldb       ,x
                    inx
                    subb      #'0'
                    clra
                    addd      ,y
                    std       ,y
                    rts

;*******************************************************************************
;        ***** putlinum *****

; *putlinum(lnum)
; *int lnum;
; *{
; putint(lnum);          put line # in token buffer
; *tbufptr++=0;          hold place for length of line
; return;
; *}

PUTLINUM            proc
                    jsr       PUTDTOK
                    clra
                    jmp       PUTTOK

;*******************************************************************************
;        ***** numeric() *****

; *numeric(c)
; *char c;
; *{
; c=c&127;
; return((c>='0')&(c<='9'));
; *}

NUMERIC             proc
                    cmpa      #'0'
                    blo       No@@
                    cmpa      #'9'
                    bhi       No@@
                    sec
                    rts

No@@                clc
                    rts

;*******************************************************************************
;        ***** alpha() *****

; *alpha(c)
; *char c;
; *{
; c=c&127;
; return((c>='A')&(c<='Z'));
; *}

ALPHA               proc
                    cmpa      #'A'
                    blo       No@@
                    cmpa      #'Z'
                    bls       Yes@@
                    cmpa      #'a'
                    blo       No@@
                    cmpa      #'z'
                    bhi       No@@
Yes@@               sec
                    rts

No@@                clc
                    rts

;*******************************************************************************
;        ***** alphanum *****

; *alphanum(c)
; *char c;
; *{ return ((alpha(c)) | (numeric(c))); }

ALPHANUM            proc
                    bsr       ALPHA
                    bcc       Numeric@@
                    rts

Numeric@@           bra       NUMERIC

;*******************************************************************************
;              xlate()
; translate the input buffer into tokenized
; form placing the results into tknbuf
;******************************************/
;
; *xlate()
; *{
; while(*ibufptr!=EOL)
; {
;  ifwhflag=0;              set IF flag to zero
;  blanks();              skip all blanks
;  if(match("DATA")) xdata();
;  else if(match("LET")) xlet();
;  else if(match("READ")) xread();
;  else if(match("RESTORE")) xrestore();
;  else if(match("GOSUB")) xgosub();
;  else if(match("GOTO")) xgoto();
;  else if(match("ON")) xon();
;  else if(match("RETURN")) xreturn();
;  else if(match("IF")) xif();
;  else if(match("INPUT")) xinput();
;  else if(match("PRINT")) xprint();
;  else if(match("FOR")) xfor();
;  else if(match("NEXT")) xnext();
;  else if(match("STOP")) xstop();
;  else if(match("ENDWH")) xendwh();
;  else if(match("END")) xend();
;  else if(match("REM")) xrem();
;*else if(match("SWAP")) xswap();*/
;  else if(match("TRON")) xtron();
;  else if(match("TROFF")) xtroff();
;  else if(match("WHILE")) xwhile();
;*else if(match("ONIRQ")) xonirq();*/
;  else ximplet();                 if no keyword, assume implied LET
;  if(errcode) return;
;  blanks();
;  if(*ibufptr==MIDEOL) { xmideol(); continue; }
;  if(*ibufptr!=EOL) { errcode=SYTXERR; return; }
;  }
; *tbufptr=EOLTOK;               put token eol in token buffer
; tknbuf[2]=tbufptr-tknbuf+1;    put line length into tokenized line
; return;
; *}

XLATE               proc
                    jsr       GETCHR              ; GET NEXT CHAR.
                    cmpa      #EOL                ; AT THE END OF THE LINE?
                    beq       Done@@              ; YES.
                    clr       IFWHFLAG            ; NOT XLATING "IF" OR "WHILE"
                    jsr       BLANKS              ; SKIP BLANKS.
                    ldx       #Keywords@@         ; POINT TO KEYWORD TABLE.
Loop@@              jsr       STREQ               ; IS KEYWORD IS IN THE INPUT BUFFER?
                    bcs       Tokenize@@          ; YES GO PROCESS IT.
SkipWord@@          inx                           ; NO. POINT TO NEXT CHAR.
                    lda       ,x                  ; AT THE END OF THIS KEYWORD?
                    bne       SkipWord@@          ; NO.
                    ldb       #4                  ; NUMBER OF BYTES TO SKIP.
                    abx
                    tst       ,x                  ; AT THE END OF THE TABLE?
                    bne       Loop@@              ; NO. CHCK FOR NEXT KEYWORD.
                    lda       #IMLETTOK           ; ASSUME AN IMPLIED LET.
                    ldx       #XIMPLET            ; GET ADDR OF XLATION ROUTINE.
                    bra       SaveToken@@

Tokenize@@          lda       1,x                 ; GET KEYWORD TOKEN.
                    ldx       2,x                 ; GET ADDR OF XLATION ROUTINE.
SaveToken@@         jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    cmpa      #DATATOK            ; SPECIAL CASE, DONT SKIP BLANKS AFTER KEYWORD.
                    beq       Go@@
                    cmpa      #REMTOK             ; SAME SPECIAL CASE AS FOR DATA.
                    beq       Go@@
                    jsr       BLANKS              ; SKIP BLANKS BETWEEN KEYWORD & NEXT OBJECT.
Go@@                jsr       ,x                  ; GO DO IT.
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       GETNXCHR            ; GET NEXT CHAR.
                    cmpa      #MIDEOL             ; IS IT A MID EOL?
                    bne       Eol?@@              ; NO. CHCK FOR EOL.
                    lda       #MEOLTOK            ; GET MID EOL TOKEN.
                    jsr       PUTTOK              ; PUT IT IN BUFFER.
                    bra       XLATE               ; CONTINUE.

Eol?@@              cmpa      #EOL                ; EOL?
                    beq       Done@@              ; YES. FINISH UP.
                    lda       #SYTXERR            ; NO. SYNTAX ERROR.
                    jmp       RPTERR              ; REPORT XLATION ERROR.

Done@@              lda       #EOLTOK             ; GET EOL TOKEN.
                    jsr       PUTTOK              ; PUT IT IN BUFFER.
                    ldd       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    subd      TKNBUFS             ; Compute the TOKEN BUFFER LENGTH.
                    ldx       TKNBUFS             ; POINT TO BUFFER.
                    stb       2,x                 ; STORE LENGTH.
                    rts                           ; RETURN.

;-------------------------------------------------------------------------------
; KEYWORD LOOK UP TABLE
;-------------------------------------------------------------------------------

Keywords@@          equ       *
DATA                @token    'DATA',DATATOK,XDATA
LET                 @token    'LET',LETTOK,XLET
READ                @token    'READ',READTOK,XREAD
RESTORE             @token    'RESTORE',RESTRTOK,XRESTORE
GOSUB               @token    'GOSUB',GOSUBTOK,XGOSUB
GOTO                @token    'GOTO',GOTOTOK,XGOTO
ONTIME              @token    'ONTIME',ONTIMTOK,XONTIME
ONIRQ               @token    'ONIRQ',ONIRQTOK,XONIRQ
ONPACC              @token    'ONPACC',ONPACTOK,XONPACC
ON                  @token    'ON',ONTOK,XON
RETURN              @token    'RETURN',RETNTOK,XRETURN
IIF                 @token    'IF',IFTOK,XIF
INPUT               @token    'INPUT',INPUTTOK,XINPUT
PRINT               @token    'PRINT',PRINTTOK,XPRINT
                    @token    '?',PRINTTOK,XPRINT
FOR                 @token    'FOR',FORTOK,XFOR
NEXT                @token    'NEXT',NEXTTOK,XNEXT
STOPSS              @token    'STOP',STOPTOK,XSTOP
ENDWH               @token    'ENDWH',ENDWHTOK,XENDWH
ENDS                @token    'END',ENDTOK,XEND
REM                 @token    'REM',REMTOK,XREM
TRON                @token    'TRON',TRONTOK,XTRON
TROFF               @token    'TROFF',TROFFTOK,XTROFF
WHILE               @token    'WHILE',WHILETOK,XWHILE
POKE                @token    'POKE',POKETOK,XPOKE
DIM                 @token    'DIM',DIMTOK,XDIM
EEP                 @token    'EEP',EEPTOK,XEEP
PORTA               @token    'PORTA',PORTATOK,XPORTA
PORTB               @token    'PORTB',PORTBTOK,XPORTB
PORTC               @token    'PORTC',PORTCTOK,XPORTC
PORTD               @token    'PORTD',PORTDTOK,XPORTD
INBYTES             @token    'INBYTE',INBYTTOK,XINBYTE
TIME                @token    'TIME',TIMETOK,XTIME
RETI                @token    'RETI',RETITOK,XRETI
PACC                @token    'PACC',PACCTOK,XPACC
SLEEP               @token    'SLEEP',SLEEPTOK,XSLEEP
RTIMES              @token    'RTIME',RTIMETOK,XRTIME
                    fcb       0                   ; END OF TABLE MARKER.

;*******************************************************************************
; *blanks()
; *{
; short spcnt;
; spcnt=0;
; while(*ibufptr==SPC) { ibufptr++; spcnt++; }

BLANKS              proc
                    pshx
                    ldx       IBUFPTR
                    clrb
Loop@@              lda       ,x
                    cmpa      #SPC
                    bne       NonSpace@@
                    incb
                    inx
                    bra       Loop@@

; if(spcnt==0) return;

NonSpace@@          tstb
                    bne       Spaces@@
                    pulx
                    rts

; if(spcnt>1)
;  {
;   *tbufptr++=MSCNTOK;
;   *tbufptr++=spcnt;
;  }

Spaces@@            stx       IBUFPTR
                    ldx       TBUFPTR
                    cmpb      #1
                    beq       Token@@
                    lda       #MSCNTOK
                    sta       ,x
                    inx
Save@@              stb       ,x
                    inx
                    stx       TBUFPTR
                    pulx
                    rts

; else
;  {
;   *tbufptr++=SSCNTOK;
;  }
; return;
; *}

Token@@             ldb       #SSCNTOK
                    bra       Save@@

; *<><><><><><><> NOTE: THIS FUNCTION NOT NEEDED <><><><><><><>
;
;**************************************
;             match()
; try to find match between *lit and
; *ibufptr. if match found, ibufptr is
; advanced to point beyond *lit. the
; string pointed to by lit must be null
; terminated.
;***************************************/
;
; *match(lit)
; *char *lit;
; *{
; int k;
; if(k=streq(ibufptr,lit))
; {
;  ibufptr+=k;
;  return(1);
; }
; return(0);
; *}

;*******************************************************************************
;****************************************
;               streq()
; compare srt1 to str2. str2 must be null
; terminated.
;*****************************************/
;
; *streq(str1,str2)
; *char *str1,*str2;
; *{
; int k;
; k=0;
; while(str2[k])         we're not at the end of string2
; {
;  if((str1[k])!=(str2[k])) return(0);
;  k++;
; }
; return(k);
; *}

STREQ               proc
                    ldd       IBUFPTR             ; SAVE VALUE OF POINTER.
                    pshd
Loop@@              lda       ,x
                    beq       Done@@
                    bsr       GETNXCHR
                    jsr       ToUpper             ; Make the character upper case.
                    cmpa      ,x
                    beq       Cont@@
                    puld
                    std       IBUFPTR
                    clc
                    rts

Cont@@              inx
                    bra       Loop@@

Done@@              puld
                    sec
                    rts

;*******************************************************************************
; THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER.

GETCHR              proc
                    pshx                          ; SAVE THE X REGISTER.
                    ldx       IBUFPTR             ; GET POINTER.
                    lda       ,x                  ; GET A CHARACTER.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

;*******************************************************************************
; THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER
; AND ADVANCES THE POINTER TO POINT TO THE NEXT CHARACTER.

GETNXCHR            proc
                    bsr       GETCHR
;                   bra       INCIBP

;*******************************************************************************
; THIS ROUTINE JUST INCREMENTS THE INPUT BUFFER POINTER.

INCIBP              proc
                    pshx                          ; SAVE X.
                    ldx       IBUFPTR             ; GET POINTER.
                    inx                           ; ADVANCE POINTER.
                    stx       IBUFPTR             ; UPDATE POINTER.
PulxRts             pulx                          ; RESTORE X
                    rts                           ; RETURN.

;*******************************************************************************
; THIS ROUTINE PUTS THE WORD IN THE D-REG. INTO THE TOKEN BUFFER
; AND ADVANCES THE TOKEN BUFFER POINTER.

PUTDTOK             proc
                    bsr       PUTTOK              ; PUT THE FIRST BYTE INTO THE TOKEN BUFFER.
                    tba                           ; PUT THE 2ND BYTE INTO A.
;                   bra       PUTTOK

;*******************************************************************************
; THIS ROUTINE PUTS THE CHARACTER IN THE A-REG. INTO THE TOKEN
; BUFFER AND ADVANCES THE TOKEN BUFFER POINTER.

PUTTOK              proc
                    pshx                          ; SAVE X.
                    pshd                          ; (9/12/89).
                    ldx       TBUFPTR             ; GET POINTER.
                    sta       ,x                  ; PUT CHARACTER.
                    inx                           ; ADVANCE POINTER.
                    stx       TBUFPTR             ; SAVE POINTER.

                    ldd       TKNBUFS             ; get the starting address of the token buffer.
                    addd      #TBUFLEN            ; add the length of the buffer to it.
                    cpd       TBUFPTR             ; IS THE TOKEN BUFFER FULL?
                    puld                          ; (9/12/89). restore the b reg.
                    bhi       PulxRts             ; NO. RESTORE X AND RETURN.
                    lda       #EXPCXERR           ; YES. FLAG THE ERROR.
                    jmp       RPTERR              ; GO REPORT IT.
;                   include   'basiclb2.mod'
                    title     BASICLB2

; *<><><><><><> NOTE: FUNCTION PERFORMED IN 'XLATE' <><><><><><>
;
;***** xmideol() *****/
;
; *xmideol()
; *{
; *tbufptr++=MEOLTOK;
; ++ibufptr;
; return;
; *}
;
;****** common code for GOSUB and GOTO *****/
;
; *xgo(gotok)
; *char gotok;
; *{
; int num;
; *tbufptr++=gotok;      put GOTO or GOSUB token in buffer
; blanks();              skip blanks before line number
; *tbufptr++=LCONTOK;    put line number constant token in buffer
; num=getlinum();        get line number
; if(num==0) errcode=LINENERR;   if 0, line number error
; if(errcode) return;    if error, return
; putint(num);           put line number in buffer
; return;
; *}

XGOSUB              proc
XGOTO               proc
;                   jsr       BLANKS
                    lda       #LCONTOK
                    bsr       PUTTOK
                    jsr       GETLINUM
                    bra       PUTDTOK

; *<><><><><><> ROUTINE NOT NEEDED <><><><><><>
;
;***** GOSUB *****/
;
; *xgosub()
; *{
; xgo(GOSUBTOK);
; return;
; *}
;
;
; *<><><><><><> ROUTINE NOT NEEDED <><><><><><>
;
;***** GOTO *****/
;
; *xgoto()
; *{
; xgo(GOTOTOK);
; return;
; *}
;***** RETURN *****/
;
; *xreturn()
; *{
; *tbufptr++=RETNTOK;    put RETURN token in buffer
; return;
; *}
;
;***** STOP *****/
;
; *xstop()
; *{
; *tbufptr++=STOPTOK;    put STOP token in buffer
; return;
; *}
;
;***** END *****/
;
; *xend()
; *{
; *tbufptr++=ENDTOK;     put end token in buffer
; return;
; *}
;
;***** TRON *****/
;
; *xtron()
; *{
; *tbufptr++=TRONTOK;    put TRON token in buffer
; return;
; *}
;
;***** TROFF *****/
;
; *xtroff()
; *{
; *tbufptr++=TROFFTOK;   put TROFF token in buffer
; return;
; *}

XRETURN             proc
XSTOP               proc
XEND                proc
XTRON               proc
XTROFF              proc
XRESTORE            proc
XENDWH              proc
XRETI               proc
XSLEEP              proc
XRTIME              proc
                    rts                           ; NULL FUNCTIONS BECAUSE TOKEN PLACEMENT IS DONE IN
                                                  ; XLATE FUNCTION.
;***** REM *****/
;
; *xrem()
; *{
; char c;
; *tbufptr++=REMTOK;     put rem token in buffer
; while(1)
; {
;  if((c=*ibufptr)==EOL) break;
;  *tbufptr++=c;
;  ++ibufptr;
; }
; return;
; *}
;
;
;***** xdata() *****/
;
; *xdata()
; *{
; char c;
; *tbufptr++=DATATOK;            put DATA token in buffer
; while((c=*ibufptr)!=EOL)
; {
;  if(c==',') *tbufptr++=COMMATOK;
;  else *tbufptr++=c;
;  ++ibufptr;
; }
; return;
; *}

XDATA               proc
XREM                proc
                    ldx       TBUFPTR             ; GET POINTER TO TOKEN BUFFER.
                    pshx                          ; SAVE IT. (POINTER TO LENGTH OF REM OR DATA)
                    lda       #0                  ; SAVE A BYTE FOR THE LENGTH.
                    bsr       PUTTOK
                    ldb       #2                  ; INITIALIZE LENGTH TO 2 (INCLUDES LENGTH & EOL.
Loop@@              bsr       GETCHR
                    cmpa      #EOL
                    beq       Done@@
                    bsr       PUTTOK
                    bsr       INCIBP
                    incb                          ; UP THE BYTE COUNT.
                    bra       Loop@@

Done@@              bsr       PUTTOK
                    pulx                          ; GET POINTER TO LENGTH BYTE.
                    stb       ,x                  ; PUT IT IN THE TOKEN BUFFER.
                    rts

;*******************************************************************************

XPORTA              proc
XPORTB              proc
XPORTC              proc
XPORTD              proc
                    ldb       #NUM                ; WE'RE XLATING A NUMERICAL STATEMENT.
                    bra       ASIGNMT1            ; GO DO IT LIKE AN ASIGNMENT STATEMENT.

;***** LET *****/
;
; *xlet()
; *{
; letcom(LETTOK);                pass LET token to common code
; return;
; *}
;
;***** implied LET *****/
;
; *ximplet()
; *{
; letcom(IMLETTOK);
; return;
; *}
;
;***** common code for explicit & implicit LET *****/
;
; *letcom(letok)
; *short letok;
; *{
; *tbufptr++=letok;              put LET token in buffer
; blanks();              skip blanks before assignment statement
; if(ibufptr=='@') { *tbufptr++=INDIRTOK; ++ibufptr; }
; asignmt();                     evaluate expression
; return;
; *}

XLET                proc
XIMPLET             proc
;                   jsr       BLANKS
;XLET1              jmp       ASIGNMT

;*******************************************************************************
;***** asignmt() *****/
;
; *asignmt()
; *{
; *short type;
; if((type=getvar())==0) return; get variable & return type
; if(errcode) return;
; if(*ibufptr++!='=') { errcode=IVEXPERR; return; } invalid expression
; *tbufptr++=EQUALTOK;           put equals token in buffer
; xexpres(type);                 build expression in token buffer
; return;
; *}

ASIGNMT             proc
                    jsr       GETVAR
                    tab
ASIGNMT1            bsr       GETNXCHR
                    cmpa      #'='
                    beq       Go@@
                    lda       #IVEXPERR
                    jmp       RPTERR

Go@@                lda       #EQUALTOK
                    bsr       PUTTOK
                    tba
;                   bra       XEXPRES

;*******************************************************************************
;***** xexpres() *****/
;
; *xexpres(type)
; *short type;
; *{
; char c;
; while(1)
; {
;  if(match("-")) *tbufptr++=NEGTOK;
;  else if(match("@")) *tbufptr++=INDIRTOK;
;  else if(match("NOT")) *tbufptr++=NOTTOK;

XEXPRES             proc
                    pshy
                    psha
                    tsy
Loop@@              ldx       #UINARYOP
                    jsr       TBLSRCH
                    bcc       LParen@@
                    bsr       PUTTOK

;  if(*ibufptr=='(')     open paren?
;  {
;   *tbufptr++=OPARNTOK; put in token buffer
;   ++ibufptr;           point to next char in input buffer
;   xexpres(type);       go get sub expression
;   if(errcode) return;
;   if(*ibufptr!=')') { errcode=UPARNERR; return; }
;   *tbufptr++=CPARNTOK; put it in the token buffer
;   ++ibufptr;           point to the next char in the input buffer
;   goto chkoprtr;
;  }

LParen@@            jsr       GETCHR
                    cmpa      #'('
                    bne       XEXPRS1
                    jsr       INCIBP
                    lda       #OPARNTOK
                    jsr       PUTTOK
                    lda       ,y
                    jsr       XEXPRES
                    jsr       GETNXCHR
                    cmpa      #')'
                    beq       RParen@@
                    lda       #UPARNERR
                    jmp       RPTERR

RParen@@            lda       #CPARNTOK
                    jsr       PUTTOK
                    jmp       CHKOPRTR

;  if((numeric(*ibufptr)) | (*ibufptr=='$') | (*ibufptr=='"'))
;  {
;   c=getcon();
;   if(errcode) return;
;  }
;  else if(c=getfun()) ;
;  else (c=getvar()) ;
;  if(errcode) return;
;  if(type==NULL) type=c;
;  if(c!=type) { errcode=DTMISERR; return; }

XEXPRS1             jsr       NUMERIC
                    bcs       XEXPRS4
                    cmpa      #'$'
                    beq       XEXPRS4
                    cmpa      #'"'
                    bne       XEXPRS5
XEXPRS4             jsr       GETCON
                    bra       XEXPRS7

XEXPRS5             jsr       GETFUN
                    tsta
                    bne       XEXPRS7
                    jsr       GETVAR
XEXPRS7             ldb       ,y
                    cmpb      #NULL
                    bne       XEXPRS8
                    sta       ,y
XEXPRS8             cmpa      ,y
                    beq       CHKOPRTR
                    lda       #DTMISERR
                    jmp       RPTERR

; *now look for operator or end of expression
;
;  chkoprtr:
;  c=*ibufptr;
;  if(c==EOL | c==MIDEOL | c==SPC | c==COMMA | c==SEMI | c==')')
;  {
;   return(c);
;  }

CHKOPRTR            jsr       GETCHR
                    cmpa      #EOL
                    beq       Done@@
                    cmpa      #MIDEOL
                    beq       Done@@
                    cmpa      #SPC
                    beq       Done@@
                    cmpa      #COMMA
                    beq       Done@@
                    cmpa      #SEMI
                    beq       Done@@
                    cmpa      #')'
                    beq       Done@@

;  if(type==NUM)
;  {
;   if(c=cknumop()) ;
;   else if(c=ckbolop()) ;
;   else if(ifwhflag) c=cklogop();
;   else c=NULL;
;  }
                    lda       ,y
                    cmpa      #NUM
                    bne       Fail@@
                    jsr       CKNUMOP
                    bcs       Operator@@
                    jsr       CKBOLOP
                    bcs       Operator@@
                    tst       IFWHFLAG
                    beq       Null@@
                    jsr       CKLOGOP
                    bra       Operator@@

Null@@              lda       #NULL
                    bra       Operator@@

;  else { errcode=IDTYERR; return; }

Fail@@              lda       #IDTYERR
                    jmp       RPTERR

;  if(c==NULL) { errcode=OPRTRERR; return; }
;  *tbufptr++=c;
; }
; return;
; *}

Operator@@          tsta
                    bne       Tokenize@@
                    lda       #OPRTRERR
                    jmp       RPTERR

Done@@              ins
                    puly
                    rts

Tokenize@@          jsr       PUTTOK
                    jmp       Loop@@

;*******************************************************************************
;***** cknumop() *****/
;
; *cknumop()
; *{
; if(match("+")) return(PLUSTOK);
; else if(match("-")) return(MINUSTOK);
; else if(match("*")) return(MULTTOK);
; else if(match("/")) return(DIVTOK);
; else if(match("\\")) return(MODTOK);
; else if(match("^")) return(PWRTOK);
; else return(NULL);
; *}

CKNUMOP             proc
                    ldx       #NUMOPTBL
CKOP                jsr       TBLSRCH
                    bcs       Done@@
                    lda       #NULL
Done@@              rts

;*******************************************************************************
;***** ckbolop() *****/
;
; *ckbolop()
; *{
; if(match("AND")) return(ANDTOK);
; else if(match("OR")) return(ORTOK);
; else if(match("EOR")) return(EORTOK);
; else return(NULL);
; *}

CKBOLOP             proc
                    ldx       #BOLOPTBL
                    bra       CKOP

;***** cklogop() *****/
;
; *cklogop()
; *{
; if(match("<=")) return(LTEQTOK);
; else if(match(">=")) return(GTEQTOK);
; else if(match("<>")) return(NOTEQTOK);
; else if(match("<")) return(LTTOK);
; else if(match(">")) return(GTTOK);
; else if(match("=")) return(EQTOK);
; else return(NULL);
; *}

CKLOGOP             proc
                    ldx       #LOGOPTBL
                    bra       CKOP

;*******************************************************************************
; *<><><><><> NOTE: THIS ROUTINE HAS NO 'C' COUNTER PART <><><><><><>

TBLSRCH             proc
                    jsr       STREQ               ; SEARCH FOR STRING.
                    bcs       Found@@             ; IF FOUND GO GET TOKEN & RETURN.
Loop@@              inx                           ; BUMP POINTER TO NEXT CHAR.
                    lda       ,x                  ; GET IT.
                    bne       Loop@@              ; KEEP LOOKING FOR END OF ENTRY.
                    inx:2                         ; FOUND IT. BUMP POINTER TO NEXT ENTRY.
                    lda       ,x                  ; AT THE END OF THE TABLE?
                    bne       TBLSRCH             ; NO. GO CHECK THE NEXT ENTRY.
                    clc                           ; YES. FLAG AS NOT FOUND.
                    rts                           ; RETURN.

Found@@             lda       1,x                 ; GET TOKEN.
                    sec                           ; FLAG AS FOUND.
                    rts                           ; RETURN.

;*******************************************************************************

NUMOPTBL            equ       *
PLUS                fcs       '+'
                    fcb       PLUSTOK
MINUS               fcs       '-'
                    fcb       MINUSTOK
MULT                fcs       '*'
                    fcb       MULTTOK
DIV                 fcs       '/'
                    fcb       DIVTOK
MODS                fcs       '\'
                    fcb       MODTOK
                    fcb       0                   ; END OF TABLE FLAG.

BOLOPTBL            equ       *
ANDS                fcs       '.AND.'
                    fcb       ANDTOK
ORS                 fcs       '.OR.'
                    fcb       ORTOK
EORS                fcs       '.EOR.'
                    fcb       EORTOK
                    fcb       0                   ; END OF TABLE FLAG.

LOGOPTBL            equ       *
LTEQ                fcs       '<='
                    fcb       LTEQTOK
GTEQ                fcs       '>='
                    fcb       GTEQTOK
NOTEQ               fcs       '<>'
                    fcb       NOTEQTOK
LT                  fcs       '<'
                    fcb       LTTOK
GT                  fcs       '>'
                    fcb       GTTOK
EQ                  fcs       '='
                    fcb       EQTOK
                    fcb       0                   ; END OF TABLE FLAG.

UINARYOP            equ       *
NEGS                fcs       '-'
                    fcb       NEGTOK
NOTS                fcs       'NOT'
                    fcb       NOTTOK
                    fcb       0                   ; END OF TABLE MARKER.
;                   include   'basiclb3.mod'
                    title     BASICLB3

;*******************************************************************************
;***** getvar *****/
;
; *tries to make a variable out of what is currently being pointed to by
; *'ibufptr' and places it into the variable symbol table if it is not
; *already there
;
; *getvar()
; *{
; short vartype,cnt;
; char varname[3];
; int offset;
; for(cnt=0; cnt<=2; cnt++) { varname[cnt]=0; } clr out var name
; if(alpha(*ibufptr)) { varname[0]=*ibufptr++; } is 1st char an alpha?
;  else { errcode=ILVARERR; return(0); } no. error

GETVAR              proc
                    pshy
                    clra
                    psha:4
                    tsy
                    jsr       GETCHR
                    jsr       ALPHA
                    bcs       Go@@
                    lda       #ILVARERR
                    jmp       RPTERR

Go@@                jsr       ToUpper
                    sta       ,y
                    jsr       INCIBP

; if(alphanum(*ibufptr)) { varname[1]=*ibufptr++; }
; if((vartype=chcktyp())==0) { vartype=FVARTOK; }
;  else { ++ibufptr; }

                    jsr       GETCHR
                    jsr       ALPHANUM
                    bcc       CheckType@@
                    jsr       ToUpper
                    sta       1,y
                    jsr       INCIBP
CheckType@@         jsr       CHCKTYP
                    sta       3,y

; if((offset=findvar(vartype,varname))==-1) is var already in table?
; {
;  if(errcode) return;
;  if((offset=putvar(vartype,varname))==-1) return;  no. put it there
; }
; if(errcode) return;

                    jsr       FINDVAR
                    cpd       #-1
                    bne       SaveVar@@
                    lda       3,y
                    jsr       PUTVAR

; *tbufptr++=vartype;    put variable type byte in token buffer
; putint(offset);        put offset after it
; if((vartype==IVARTOK) | (vartype==FVARTOK)) return(NUM);
; return(STRING);
; *}

SaveVar@@           pshd
                    lda       3,y
                    jsr       PUTTOK
                    puld
                    jsr       PUTDTOK
                    lda       3,y                 ; GET VARIABLE TYPE AGAIN.
                    bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       Cont@@              ; NO. CONTINUE.
                    jsr       INCIBP              ; MOVE THE INPUT BUFFER POINTER PAST THE OPEN (.
                    lda       #OPARNTOK
                    jsr       PUTTOK
                    lda       #NUM                ; YES. SUBSCRIPT EXPRESSION MUST BE NUMERIC.
                    jsr       XEXPRES             ; GO GET THE SUBSCRIPT.
                    jsr       GETNXCHR            ; GET THE TERMINATING CHARACTER.
                    cmpa      #')'                ; IS IT A CLOSING PAREN?
                    beq       Finish@@            ; YES. GO FINISH UP.
                    lda       #MPARNERR           ; NO. ERROR.
                    jmp       RPTERR

Finish@@            lda       #CPARNTOK           ; GET CLOSING PAREN TOKEN.
                    jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
Cont@@              lda       #NUM                ; NO. RETURN PROPER TYPE.
                    ldb       3,y
                    bitb      #2
                    beq       Done@@
                    lda       #STRING
Done@@              ins:4
                    puly
                    rts

;*******************************************************************************
;***** chcktype *****/
;
; *chcktyp()
; *{
; if(*ibufptr=='%') return(IVARTOK);
;  else if(*ibufptr=='$') return(SVARTOK);
;  else return(0);
; *}

CHCKTYP             proc
                    lda       #IVARTOK            ; IN V1.0 ONLY INTEGER VARIABLES ARE SUPPORTED.
                    psha                          ; IN V2.0 FLOATING POINT VARIABLES WILL BE
                    jsr       GETCHR              ; SUPPORTED.
                    cmpa      #'('                ; IS A SUBSCRIPT FOLLOWING THE NAME?
                    pula                          ; RESTORE THE TOKEN TYPE.
                    bne       Done@@              ; NO. RETURN.
                    adda      #$10                ; YES. MAKE IT AN ARRAY VARIABLE.
Done@@              rts                           ; RETURN.

;*******************************************************************************
;***** findvar *****/
;
; *findvar(vartype,varname)
; *short vartype;
; *char *varname;
; *{
; char *varptr;
; varptr=varbegin;               point to the start of the var table
; while(*varptr)                 we're not to the end of the table

FINDVAR             proc
                    ldx       VARBEGIN
Loop@@              tst       ,x
                    beq       Done@@

; {
;  if(*varptr==vartype)          is the current var the same type?
;  {                             yes.
;   if(streq(varptr+1,varname))  is the name the same?
;   {                            yes.
;    return(varptr-varbegin);    return the offset from the table start
;   }
;  }

                    cmpa      ,x
                    bne       NextVar@@
                    ldb       1,x
                    cmpb      ,y
                    bne       NextVar@@
                    ldb       2,x
                    cmpb      1,y
                    bne       NextVar@@
                    xgdx
                    subd      VARBEGIN
                    rts

;  if not, advance to the next variable in the table
;  if(*varptr==IVARTOK) varptr=varptr+ISIZ+3;
;  else if(*varptr==SVARTOK) varptr=varptr+SSIZ+3;
;  else if(*varptr==FVARTOK) varptr=varptr+FSIZ+3;
;  else { errcode=ILTOKERR; return(-1); }
; }

NextVar@@           ldb       ,x
                    bitb      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       Cont@@              ; NO CONTINUE.
                    ldb       #ASIZ+3             ; YES. GET ARRAY SIZE +3.
                    bra       Skip@@

Cont@@              cmpb      #IVARTOK
                    bne       Fail@@
                    ldb       #ISIZ+3
Skip@@              abx
                    bra       Loop@@

Fail@@              lda       #ILTOKERR
                    jmp       RPTERR

Done@@              ldd       #-1
                    rts

; return(-1);
; *}
;
;*******************************************************************************
;***** putvar *****/
;
; *putvar(vartype,varname)
; *short vartype;
; *char *varname;
; *{
; *short count,n;
; *char *varadd;
; varadd=varend;         save begining addr of var we are storing
; *varend++=vartype;     put token/type in variable symbol table
; *varend++=*varname++;  put variable name in
; *varend++=*varname++;

PUTVAR              proc
                    ldx       VAREND
                    pshx
                    sta       ,x
                    inx
                    ldb       ,y
                    stb       ,x
                    inx
                    ldb       1,y
                    stb       ,x
                    inx

; if(vartype==IVARTOK) count=ISIZ+1;     determine # of bytes for this
; else if(vartype==SVARTOK) count=SSIZ+1;variable
; else if(vartype==FVARTOK) count=FSIZ+1;
; else { errcode=ILTOKERR; return(-1); }
; for(n=1;n<=count;n++) *varend++=0;      zero the storage
; --varend;
; if(varend > varmend) { errcode=OMEMERR; return(-1); } memory overflow?
; vmemavil-=count;       decrement the amt of avail memory
; return(varadd-varbegin);       return offset
; *}

                    bsr       CLRVAR
                    clr       ,x                  ; CLEAR 1 BYTE BEYOND THE END OF THE VAR AREA.
                    stx       VAREND
                    cpx       VARMEND
                    bls       Go@@
                    lda       #OMEMERR
                    bra       ?ErrHook

Go@@                puld
                    subd      VARBEGIN
                    pshd                          ; SAVE THE OFFSET TO THIS VARIABLE.
                    jsr       CCLEAR3             ; CLEAR ALL VARIABLES SINCE WE MAY HAVE TRASHED
                                                  ; ANY ARRAYS THAT HAD BEEN ALLOCATED.
                    puld                          ; RESTORE THE 'NEW' VARIABLE OFFSET.
                    rts

;*******************************************************************************

CLRVAR              proc
                    bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       Go@@                ; NO. CONTINUE.
                    ldb       #ASIZ               ; YES. GET THE DICTIONARY SIZE+1.
                    bra       Loop@@              ; PUT THE VARIABLE IN THE DICTIONARY.

Go@@                cmpa      #IVARTOK
                    bne       Fail@@
                    ldb       #ISIZ
Loop@@              clr       ,x
                    inx
                    decb
                    bne       Loop@@
                    rts

Fail@@              lda       #ILTOKERR
?ErrHook            jmp       RPTERR

;*******************************************************************************
;***** getcon() *****/
;
; *getcon()
; *{
; int const;
; char *litp;
; short count;
; litp=ibufptr;          save a pointer to start of constant
; if(*ibufptr=='"') { getscon(); return(STRING); } if " get strng

GETCON              proc
                    jsr       GETCHR

;  else if(*ibufptr=='$') { ++ibufptr; const=gethex(); } if '$' get hex
;  else const=getdeci();         else assume its a decimal constant
; if(errcode) return(0);         if error abort

                    ldx       IBUFPTR
                    pshx
                    cmpa      #'$'
                    bne       Decimal@@
                    jsr       INCIBP
                    jsr       GETHEX
                    bra       Go@@

Decimal@@           jsr       GETDECI

; *tbufptr++=ICONTOK;            put integer constant token in buffer
; putint(const);                 follow it with the constant
; count=ibufptr-litp;    get number of bytes in source form of const.
; *tbufptr++=count;       put it in the token buffer
; while(litp < ibufptr) *tbufptr++=*litp++; copy source form into buffer
; return(NUM);           return the constant type
; }

Go@@                psha
                    lda       #ICONTOK
                    jsr       PUTTOK
                    pula
                    jsr       PUTDTOK
                    ldd       IBUFPTR
                    tsx
                    subd      ,x
                    tba
                    jsr       PUTTOK
                    pulx
Loop@@              lda       ,x
                    jsr       PUTTOK
                    inx
                    decb
                    bne       Loop@@
                    lda       #NUM
                    rts

;*******************************************************************************
;***** getdeci() *****/
;
; *getdeci()
; *{
; char c;
; int num;
; num=0;
; if(numeric(*ibufptr)==0)       is 1st char numeric?
;  { errcode=SYTXERR; return; }  no. flag error & return
; while(numeric(c=*ibufptr))     yes. while *ibufptr is numeric
; {
;  num=num*10+(c-'0');           build number
;  if(num < 0) { errcode=INTOVERR; return; }     if <0 flag error & ret
;  ++ibufptr;
; }
; return(num);           return the value
; *}

GETDECI             proc
                    pshy
                    clra
                    psha:2
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    jsr       NUMERIC
                    bcs       Loop@@
                    lda       #SYTXERR
                    bra       CHCKERR

Loop@@              lda       ,x
                    jsr       NUMERIC
                    bcc       ?GETDECI
                    jsr       ADDDIG
                    bpl       Loop@@
                    lda       #INTOVERR
                    bra       CHCKERR

?GETDECI            stx       IBUFPTR
                    ldd       ,y
                    ins:2
                    puly
                    rts

;*******************************************************************************
;***** gethex() *****/
;
; *gethex()
; *{
; char c;
; short count;
; int num;
; num=count=0;
; if(hexdig(*ibufptr)==0)        is the char a hex digit?
;  { errcode=IVHEXERR; return; } no. flag error & return

GETHEX              proc
                    pshy
                    clra
                    psha:2
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    jsr       HEXDIG
                    bcs       Loop@@
                    lda       #IVHEXERR
CHCKERR             tst       RUNFLAG
                    jne       RPTRERR
                    jmp       RPTERR

; while(hexdig(c=*ibufptr))      while a hex digit is in the buffer
; {
;  if(numeric(c)) num=num*16+(c-'0');  build the number
;   else num=num*16+(c-55);
;  if(count++ > 4)
;   { errcode=HEXOVERR; return; }  if over 4 digits flag overflow & ret
;  ++ibufptr;
; }
; return(num);   return constant value
; *}

Loop@@              lda       ,x
                    jsr       HEXDIG
                    bcc       ?GETDECI
                    ldd       ,y
                    lsld
                    bcs       Overflow@@
                    lsld
                    bcs       Overflow@@
                    lsld
                    bcs       Overflow@@
                    lsld
                    bcs       Overflow@@
                    std       ,y
                    lda       ,x
                    jsr       ToUpper
                    tab
                    inx
                    subb      #'0'
                    cmpb      #9
                    bls       Cont@@
                    subb      #7
Cont@@              clra
                    addd      ,y
                    std       ,y
                    bra       Loop@@

Overflow@@          lda       #HEXOVERR
                    bra       CHCKERR

;*******************************************************************************
;***** hexdig() *****/
;
; *hexdig(c)
; *char c;
; *{
; return(numeric(c) | (c>='A' & c<='F')); return true if c is hex
; *}

HEXDIG              proc
                    jsr       NUMERIC
                    bcc       Letter@@
                    rts

Letter@@            jsr       ToUpper
                    cmpa      #'A'
                    blo       Done@@
                    cmpa      #'F'
                    bhi       Done@@
                    sec
                    rts

Done@@              clc
                    rts

;*******************************************************************************
;***** getscon *****/
;
; *getscon()
; *{
; short count;
; char *bufptr,c;
; count=2;       initialize byte count to 2
; *tbufptr++=SCONTOK;   put string constant token in buffer
; bufptr=tbufptr++;   save value of tbufptr, advance to next byte,
;                     and reserve a byte for string length
; *tbufptr++=*ibufptr++;   put 1st quote in token buffer

GETSCON             proc
                    ldb       #2
                    lda       #SCONTOK
                    jsr       PUTTOK
                    ldx       TBUFPTR
                    pshx
                    clra
                    jsr       PUTTOK
                    jsr       GETNXCHR            ; PUT FIRST QUOTE IN TOKEN BUFFER.
                    jsr       PUTTOK

; while(((c=*ibufptr) != '"'))
; {
;  if(c==EOL)             if we hit EOL
;   { errcode=MISQUERR; return; } flag error & return
;  *tbufptr++=c;         if not, put next char in buffer
;  ++ibufptr;            advance input buffer pointer
;  ++count;              up byte count
; }

Loop@@              jsr       GETNXCHR
                    cmpa      #'"'
                    beq       Quote@@
                    cmpa      #EOL
                    bne       Cont@@
                    lda       #MISQUERR
                    jmp       RPTERR

Cont@@              jsr       PUTTOK
                    incb
                    bra       Loop@@

; *tbufptr++=c;          put closing quote in token buffer
; ++ibufptr;             advance input buffer pointer
; *bufptr=count;         put string byte count in token buffer
; return;
; *}

Quote@@             jsr       PUTTOK
                    pulx
                    stb       ,x
                    rts
;                   include   'basiclb5.mod'
                    title     BASICLB5

;*******************************************************************************
;***** getfun() *****/
;
; *getfun()
; *{
; short type;
; if(match("FDIV")) type=xfdiv();
; else if(match("CHR$")) type=xchrs();
; else if(match("ABS")) type=xabs();
; else if(match("RND")) type=xrnd();
; else if(match("SGN")) type=xsgn();
; else if(match("TAB")) type=xtab();
; else if(match("ADC")) type=xadc();
; else if(match("CALL")) type=xcall();
; else return(0);
; return(type);
; *}

GETFUN              proc
                    ldx       #FuncTable@@
Loop@@              jsr       STREQ
                    bcs       Func@@
FindNull@@          inx
                    lda       ,x
                    bne       FindNull@@
                    ldb       #4
                    abx
                    tst       ,x
                    bne       Loop@@
                    clra
                    rts

Func@@              lda       #FUNCTFLG
                    jsr       PUTTOK
                    lda       1,x
                    ldx       2,x
                    jmp       ,x

;-------------------------------------------------------------------------------

FuncTable@@         equ       *
FDIVS               @token    'FDIV',FDIVTOK,BNUMFUN
CHRS                @token    'CHR$',CHRTOK,UNUMFUN
ABS                 @token    'ABS',ABSTOK,UNUMFUN
RND                 @token    'RND',RNDTOK,UNUMFUN
SGN                 @token    'SGN',SGNTOK,UNUMFUN
TABS                @token    'TAB',TABTOK,UNUMFUN
ADCS                @token    'ADC',ADCTOK,UNUMFUN
CALL                @token    'CALL',CALLTOK,UNUMFUN
PEEK                @token    'PEEK',PEEKTOK,UNUMFUN
                    @token    'EEP',FEEPTOK,UNUMFUN
HEX2                @token    'HEX2',HEX2TOK,UNUMFUN
HEX                 @token    'HEX',HEXTOK,UNUMFUN
                    @token    'PORT',FPRTATOK,FINDPORT
                    @token    'TIME',FTIMETOK,XTIMEF
                    @token    'PACC',FPACCTOK,XPACCF
                    fcb       0                   ; END OF TABLE MARKER.

;*******************************************************************************

XPOKE               proc
                    ldx       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    dex                           ; DEC. TO COMPENSATE FOR PUTTOK DONE IN XLATE.
                    stx       TBUFPTR             ; SAVE NEW POINTER VALUE. FALL THROUGH TO BNUMFUN.
                    lda       ,x                  ; GET TOKEN BACK INTO THE A-REG.
;                   bra       BNUMFUN

;*******************************************************************************
;***** xfdiv() *****/
;
; *xfdiv()
; *{
; short type[2];
; type[0]=type[1]=NUM;            both arguments must be type NUM
; dofunct(FDIVTOK,2,type);
; return(NUM);
; *}

BNUMFUN             proc
                    pshy
                    ldb       #NUM
                    pshb:2
                    tsy
                    ldb       #2
                    jsr       DOFUNCT
;                   lda       #NUM
                    pula:2
                    puly
                    rts

;***** xchrs *****/
;
; *xchrs()
; *{
; return(unumfun(CHRTOK));
; *}
;
;***** xabs() *****/
;
; *xabs()
; *{
; return(unumfun(ABSTOK));
; *}
;
;***** xrnd() *****/
;
; *xrnd()
; *{
; return(unumfun(RNDTOK));
; *}
;
;***** xsgn() *****/
;
; *xsgn()
; *{
; return(unumfun(SGNTOK));
; *}
;
;***** xtab() *****/
;
; *xtab()
; *{
; return(unumfun(TABTOK));
; *}
;
;***** xadc() *****/
;
; *xadc()
; *{
; return(unumfun(ADCTOK));
; *}
;***** xcall() *****/
;
; *xcall()
; *{
; return(unumfun(CALLTOK));
; *}
;
;***** unumfun() *****/
;
; *unumfun(token)  common code for a uinary numerical function
; *short token;
; *{
; short type[1];         setup argument 'type' array
; type[0]=NUM;           set the 1st (only) argument type to NUM
; dofunct(token,1,type); go do the function
; return(NUM);           return the function type
; *}

XEEP                proc                          ; PROGRAM A WORD OF EEPROM.
                    ldx       TBUFPTR             ; COMPENSATE FOR TOKEN PLACEMENT BU UNUMFUN
                    dex                           ; ROUTINE.
                    stx       TBUFPTR             ; SAVE POINTER.
                    lda       ,x                  ; GET TOKEN FROM BUFFER.
                    bsr       UNUMFUN             ; GO TREAT AS A UNIARY NUMERIC FUNCTION.
                    jmp       ASIGNMT1            ; GO USE ASSIGNMENT CODE FOR REST OF FUNCTION.

;*******************************************************************************

UNUMFUN             proc
                    pshy
                    ldb       #NUM
                    pshb
                    ldb       #1
                    tsy
                    bsr       DOFUNCT
;                   lda       #NUM
                    pula
                    puly
                    rts

;*******************************************************************************
;***** dofunct() *****/
;
; *dofunct(functok,nargs,type)
; *short functok,nargs,*type;
; *{
; *tbufptr++=functok;            put function token in buffer
; if(*ibufptr!='(') { errcode=ILFSYERR; return; }
; *tbufptr++=OPARNTOK;           put open paren in token buffer
; ++ibufptr;

DOFUNCT             proc
                    jsr       PUTTOK
                    jsr       GETCHR
                    cmpa      #'('
                    beq       LParen@@
Fail@@              lda       #MPARNERR
                    jmp       RPTERR

LParen@@            jsr       INCIBP
                    lda       #OPARNTOK
                    jsr       PUTTOK

; while(1)
; {
;  xexpres(*type++);             get the argument/expression
;  if(errcode) return;           return if error
;  if(--nargs==0) break;         if we have all the arguments, quit
;  if(*ibufptr!=',')             if delimiter not present, return
;   { errcode=ILFSYERR; return; }
;  *tbufptr++=COMMATOK;          if it is, put it in the token buffer
;  ++ibufptr;                    point to the next character
; }

Loop@@              lda       ,y
                    iny
                    pshb
                    jsr       XEXPRES
                    pulb
                    decb
                    beq       RParen@@
                    jsr       CHKCOMA
                    bcc       Fail@@
                    bra       Loop@@

; if(*ibufptr!=')')              must see closing paren
; {
;  errcode=ILFSYERR;             if not, error
;  return;
; }
; else                           saw closing paren
; {
;  *tbufptr++=CPARNTOK;          put it in the token buffer
;  ++ibufptr;                    advance input buffer pointer
; }
; return;
; *}

RParen@@            jsr       GETCHR
                    cmpa      #')'
                    bne       Fail@@
                    jsr       INCIBP
                    lda       #CPARNTOK
                    jmp       PUTTOK              ; PUT TOKEN IN BUFFER & RETURN.

;*******************************************************************************

FINDPORT            proc
                    jsr       GETNXCHR            ; GO GET PORT "NUMBER".
                    jsr       ToUpper             ; Translate the character to upper case.
                    cmpa      #'A'                ; IS IT AN A OR HIGHER?
                    bhs       CheckUpperLimit@@   ; YES. GO CHECK UPPER LIMIT.
Fail@@              lda       #ILPRTERR           ; NO. ILLEGAL PORT "NUMBER".
                    jmp       RPTERR              ; REPORT ERROR.

CheckUpperLimit@@   cmpa      #'E'                ; IS IT HIGHER THAN AN "E"?
                    bhi       Fail@@              ; YES. ILLEGAL PORT.
                    suba      #'A'                ; SUBTRACT "BASE" PORT OF A
                    adda      #FPRTATOK           ; ADD IN "BASE" TOKEN.
;                   bra       XPACCF              ; STEAL SOME CODE.

;*******************************************************************************

XPACCF              proc
XTIMEF              proc
                    jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    lda       #NUM                ; RETURN TYPE "NUM".
                    rts                           ; RETURN.
;                   include   'basiclb4.mod'
                    title     BASICLB4

;*******************************************************************************
;***** xon *****/
;
; *xon()
; *{
; int num;
; *tbufptr++=ONTOK;      put ON token in buffer
; blanks();              skip any blanks
; xexpres(NUM);          get the expression
; if(errcode) return;    if error, return
; blanks();              skip any blanks
; if(match("GOTO")) xgoto();     check for GOTO
; else if(match("GOSUB")) xgosub();      check for GOSUB
; else errcode=IONSYERR;         if neither, flag an error
; if(errcode) return;            if error, return
; blanks();              skip blanks

XON                 proc
;                   jsr       BLANKS
                    lda       #NUM
                    jsr       XEXPRES
                    jsr       BLANKS
                    ldx       #GOTO
                    jsr       STREQ
                    bcc       Gosub?@@
                    lda       #GOTOTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       XGOTO
                    bra       Loop@@

Gosub?@@            ldx       #GOSUB
                    jsr       STREQ
                    bcs       Gosub@@
                    lda       #IONSYERR
                    jmp       RPTERR

Gosub@@             lda       #GOSUBTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       XGOSUB
Loop@@              jsr       BLANKS

; while(*ibufptr==',')   do until EOL
; {
;  *tbufptr++=COMMATOK;  put COMMA token in buffer
;  ++ibufptr;            advance the input buffer pointer
;  blanks();             skip blanks
;  *tbufptr++=LCONTOK;   put line number constant token in buffer
;  num=getlinum();       get line number
;  if(num==0) errcode=LINENERR;   if 0, line number error
;  if(errcode) return;    if error, return
;  putint(num);           put line number in buffer
;  blanks();              skip blanks
; }
; return;
; *}
                    jsr       CHKCOMA
                    bcs       Comma@@
                    rts

Comma@@             jsr       BLANKS
                    lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
                    jsr       PUTDTOK
                    bra       Loop@@

;*******************************************************************************

XONIRQ              proc                          ; "ONIRQ" HAS A FUNCTION CODE & LINE NUMBER.
XONTIME             proc
                    lda       #NUM                ; GO GET THE VALUE OF THE TIMER WE SHOULD GO ON.
                    jsr       XEXPRES
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       CHKCOMA             ; GO CHECK FOR A COMMA.
                    bcs       Go@@                ; IF PRESENT, IT'S OK.
?XONTIME            lda       #MCOMAERR           ; IF NOT, REPORT AN ERROR.
                    jmp       RPTERR

Go@@                jsr       BLANKS
                    lda       #LCONTOK            ; GET THE LINE CONSTANT TOKEN.
                    jsr       PUTTOK              ; PUT IT IN THE TOKEN BUFFER.
                    jsr       GETLINUM            ; GO GET A LINE NUMBER.
                    jmp       PUTDTOK             ; PUT THE LINE NUMBER IN THE TOKEN BUFFER.

;*******************************************************************************

XONPACC             proc
                    bsr       GETARG              ; GET AN ARGUMENT AND A COMMA.
                    bra       XONTIME             ; GO USE SOME OTHER CODE.

;*******************************************************************************

GETARG              proc
                    lda       #NUM                ; GO GET THE "OPERATING MODE" EXPRESSION.
                    jsr       XEXPRES
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       CHKCOMA             ; GO CHECK FOR COMMA.
                    bcc       ?XONTIME            ; NO COMMA. REPORT ERROR.
                    jmp       BLANKS              ; SKIP BLANKS AFTER COMMA AND RETURN.

;*******************************************************************************
;***** xif() *****/
;
; *xif()
; *{
; *int num;
; *tbufptr++=IFTOK;              put if token in the buffer
; blanks();                      skip any blanks
; ifwhflag=1;                    let xexpres() know we are doing an IF
; xexpres(NULL);                 get relational expression
; if(errcode) return;            if error, return
; blanks();                      if not, skip blanks

XIF                 proc
;                   jsr       BLANKS
                    inc       IFWHFLAG
                    lda       #NUM
                    jsr       XEXPRES
                    jsr       BLANKS

; if(match("THEN"))              check for "THEN" clause
; {
;  *tbufptr++=THENTOK;           put THEN token in the buffer
;  blanks();                     skip any blanks after "THEN"

                    ldx       #THENS
                    jsr       STREQ
                    bcs       Then@@
                    lda       #MTHENERR
                    jmp       RPTERR

;  if(numeric(*ibufptr))         is a line number present after THEN?
;  {                             yes
;   *tbufptr++=LCONTOK;          put line # const. token in buffer
;   num=getlinum();              get the line #
;   if(num==0) errcode=LINENERR;
;   if(errcode) return;          if error, return
;   putint(num);                 put number in buffer
;  }
;  else                          not a line #, check for statement
;  {
;   xlate();             try to make a statement out of what follows
;   if(errcode) return;          if error, return
;  }
; }

Then@@              lda       #THENTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcc       Fail@@

                    lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
                    jsr       PUTDTOK

; else                   if "THEN" not present
; {
;  errcode=MTHENERR;             flag a missing THEN error
;  return;
; }
;
;
; blanks();                      skip any blanks after object of THEN
; if(match("ELSE"))              is "ELSE" clause present?
; {                              yes
;  *tbufptr++=ELSETOK;           put ELSE token in the buffer
;  blanks();                     skip any blanks after ELSE

                    jsr       BLANKS
                    ldx       #ELSES
                    jsr       STREQ
                    bcs       ElseTok@@
                    rts

ElseTok@@           lda       #ELSETOK
                    jsr       PUTTOK
                    jsr       BLANKS

;  if(numeric(*ibufptr))         is a line # present after ELSE
;  {                     yes
;   *tbufptr++=LCONTOK;          put line # const. token in buffer
;   num=getlinum();              get the line #
;   if(num==0) errcode=LINENERR;
;   if(errcode) return;          if error, return
;   putint(num);                 put number in buffer
;  }
;  else          line # not present, try to xlate a statement
;  {
;   xlate();
;  }
; }
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcs       Cont@@
Fail@@              lda       #LINENERR
                    jmp       RPTERR

Cont@@              lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
                    jmp       PUTDTOK

; return;                in any case, return
; *}

THENS               fcs       'THEN'
ELSES               fcs       'ELSE'

;*******************************************************************************
;***** xfor() *****/
;
; *xfor()
; *{
; short type;
; *tbufptr++=FORTOK;             put for token in buffer
; blanks();              skip blanks between FOR & assignment statement
; type=getvar();         get variable
; if((type!=NUM)|(*ibufptr++!='='))      is it a numerical variable?
; { errcode=IVEXPERR; return; }  no. flag error & return

XFOR                proc
;                   jsr       BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       Go@@
Fail@@              lda       #IVEXPERR
                    jmp       RPTERR

Go@@                jsr       GETNXCHR
                    cmpa      #'='
                    bne       Fail@@

; *tbufptr++=EQUALTOK;   put equals token in buffer
; xexpres(NUM);          go get a numerical expression
; if(errcode) return;    if error, return
; blanks();              skip blanks

                    lda       #EQUALTOK
                    jsr       PUTTOK
                    lda       #NUM
                    jsr       XEXPRES
                    jsr       BLANKS

; if(match("TO"))        is TO present?
; {                      yes
;  *tbufptr++=TOTOK;     put TO token in buffer
;  blanks();             skip blanks
;  xexpres(NUM);         get the "TO" expression
;  if(errcode) return;   return if error
; }
; else                   "TO" not present
; {
;  errcode=MTOERR;       set error flag & return
;  return;
; }

                    ldx       #TO
                    jsr       STREQ
                    bcs       To@@
                    lda       #MTOERR
                    jmp       RPTERR

To@@                lda       #TOTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    lda       #NUM
                    jsr       XEXPRES

; blanks();              skip blanks
; if(match("STEP"))      is optional "STEP" clause present?
; {                      yes
;  *tbufptr++=STEPTOK;   put STEP token in buffer
;  blanks();             skip blanks
;  xexpres(NUM);         get expression
; }
; return;
; *}

                    jsr       BLANKS
                    ldx       #STEP
                    jsr       STREQ
                    bcs       Step@@
                    rts

Step@@              lda       #STEPTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    lda       #NUM
                    jmp       XEXPRES

TO                  fcs       'TO'
STEP                fcs       'STEP'

;*******************************************************************************
;***** xnext() *****/
;
; *xnext()
; *{
; *tbufptr++=NEXTTOK;    put NEXT token in buffer
; blanks();              skip blanks
; if(getvar()!=NUM) errcode=SYTXERR;     get variable, must be numeric
; return;
; *}

XNEXT               proc
;                   jsr       BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       Done@@
                    lda       #SYTXERR
                    jmp       RPTERR
Done@@              rts

;*******************************************************************************
;***** xprint() *****/
;
; *xprint()
; *{
; *tbufptr++=PRINTTOK;   put PRINT token in buffer
; blanks();             skip blanks

XPRINT              proc
;                   jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #'#'                ; HAS AN ALTERNATE PORT BEEN SPECIFIED?
                    bne       Loop@@              ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.
                    lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    bra       Expr@@              ; GO GET EXPRESSION & CONTINUE.

; while((*ibufptr!=EOL)|(*ibufptr!=MIDEOL))    do until end of line
; {
;  xexpres(NULL);        get expression
;  if(errcode) return;   if error, return
;  blanks();             skip blanks
;  if(*ibufptr==COMMA) *tbufptr=COMMATOK;        check for comma
;  else if(*ibufptr==SEMI) *tbufptr=SEMITOK;     check for semicolon
;  else return;          if neither, return
;  ++ibufptr;            advance input buffer pointer
;  ++tbufptr;            advance token buffer pointer
;  blanks();             skip blanks after delimeter
; }
; return;
; *}

Loop@@              jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #EOL
                    beq       Done@@
                    cmpa      #MIDEOL
                    bne       GetChar@@
Done@@              rts

GetChar@@           jsr       GETCHR              ; GET THE NEXT CHARACTER IN THE BUFFER.
                    cmpa      #'"'                ; IS IT A STRING CONSTANT?
                    bne       Expr@@
                    jsr       GETSCON             ; YES. GO GET A STRING CONSTANT.
                    bra       Cont@@              ; CONTINUE.

Expr@@              lda       #NUM
                    jsr       XEXPRES
Cont@@              jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #EOL
                    beq       Done@@
                    cmpa      #MIDEOL
                    beq       Done@@
                    bsr       CHKCOMA
                    bcs       Loop@@
                    cmpa      #SEMI
                    beq       Semicolon@@
                    lda       #MCMSMERR
                    jmp       RPTERR

Semicolon@@         lda       #SEMITOK
                    bsr       ?CHKCOMA
                    bra       Loop@@

;*******************************************************************************

CHKCOMA             proc
                    jsr       GETCHR              ; GET CHARACTER FROM INPUT BUFFER.
                    cmpa      #COMMA              ; IS IT A COMMA?
                    beq       Go@@                ; YES. PUT IT IN THE TOKEN BUFFER.
                    clc                           ; NO. FLAG NO COMMA FOUND.
                    rts                           ; RETURN.

Go@@                lda       #COMMATOK           ; GET THE COMMA TOKEN.
?CHKCOMA            jsr       PUTTOK              ; PUT THE TOKEN IN THE BUFFER.
                    jsr       INCIBP              ; BUMP THE INPUT BUFFER POINTER.
                    sec
                    rts                           ; RETURN.

;*******************************************************************************
;***** xinput() *****/
;
; *xinput()
; *{
; *tbufptr++=INPUTTOK;           put INPUT token in buffer
; blanks();                      skip blanks

XINPUT              proc
;                   jsr       BLANKS
                    bsr       XCHKDEV             ; GO CHECK FOR AN ALTERNATE DEVICE NUMBER.

; if(*ibufptr=='"')              is a prompt included?
; {                      yes
;  getscon();            get the string constant
;  if(errcode) return;   if error, return
;  if(*ibufptr!=COMMA) { errcode=SYTXERR; return; }
;  *tbufptr++=COMMATOK;
;  ++ibufptr;
; }
; inreadcm();            get the input variable list
; return;
; *}

                    jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #'"'
                    bne       INREADCM
                    jsr       GETSCON
                    bsr       CHKCOMA             ; IF COMMA PRESENT, PUT IN TOKEN BUFFER.
                    bcs       INREADCM
?XINPUT             lda       #MCOMAERR
                    jmp       RPTERR

;*******************************************************************************
;***** inreadcm() *****/
;
; *inreadcm()
; *{
; while(1)               do forever
; {
;  blanks();             skip blanks
;  getvar();             get a variable
;  if(errcode) return;   if error, return
;  blanks();             skip blanks
;  if(*ibufptr==COMMA)
;  {
;   *tbufptr++=COMMATOK;      put delimiter in buffer
;   ++ibufptr;                and point to the next char in the buffer
;  }
;  else return;          if no delimiter return
; }
; *}

XDIM                proc
INREADCM            proc
XREAD               proc
;                   jsr       BLANKS
                    jsr       GETVAR
                    jsr       BLANKS
                    bsr       CHKCOMA
                    bcs       XREAD
                    rts

;*******************************************************************************

XCHKDEV             proc
                    jsr       GETCHR
                    cmpa      #'#'                ; HAS AN ALTERNATE PORT BEEN SPECIFIED?
                    beq       Go@@                ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.
                    rts                           ; RETURN.

Go@@                lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    lda       #NUM                ; EXPRESSION MUST BE NUMERIC.
                    jsr       XEXPRES             ; GO GET THE EXPRESSION.
                    jsr       BLANKS              ; SKIP SPACES.
                    bsr       CHKCOMA             ; GO GET COMMA THAT MUST FOLLOW THE EXPRESSION.
                    bcc       ?XINPUT             ; MISSING COMMA. GO REPORT THE ERROR.
                    rts                           ; IT WAS THERE. GO PROCESS THE REST OF THE STATEMENT.

;*******************************************************************************

XINBYTE             proc
                    bsr       XCHKDEV             ; GO CHECK FOR ALTERNATE DEVICE.
                    jsr       BLANKS              ; SKIP BLANKS AFTER COMMA.
                    jmp       GETVAR              ; GO TRY TO GET A VARIABLE.

;***** xread *****/
;
; *xread()
; *{
; *tbufptr++=READTOK;    put read token in buffer
; inreadcm();            get the variable list
; return;
; *}
;
;***** xrestore() *****/
;
; *xrestore()
; *{
; *tbufptr++=RESTRTOK;   put RESTORE token in buffer
; return;
; *}
;
;***** xwhile() *****/
;
; *xwhile()
; *{
; *tbufptr++=WHILETOK;   put WHILE token in buffer
; blanks();              skip blanks
; ifwhflag=1;            indicate we are going to get a WHILE expression
; xexpres(NULL);         get expression
; return;
; *}

XWHILE              proc
;                   jsr       BLANKS
                    inc       IFWHFLAG
                    lda       #NULL
                    jmp       XEXPRES

;*******************************************************************************
;***** xendwh() *****/
;
; *xendwh()
; *{
; *tbufptr++=ENDWHTOK;   put ENDWH token in buffer
; return;
; *}

XPACC               proc
XTIME               proc
                    ldb       #NUM                ; SETUP TO USE CODE IN "ASIGNMT".
                    jmp       ASIGNMT1            ; GO DO ASSIGNMENT STATEMENT.

;*******************************************************************************
;***** rpterr() *****/
;
; *rpterr()
; *{
; *char *ptr,c;
; ptr=inbuff;    point to start of input buffer
; nl();
; nl();
; while((c=*ptr++)!=EOL) outbyte(c);     print the input buffer

RPTERR              proc
                    sta       ERRCODE
                    jsr       NL2
                    ldx       INBUFFS
Loop@@              lda       ,x
                    cmpa      #EOL
                    beq       RPTERR2
                    jsr       OUTBYTE
                    inx
                    bra       Loop@@

; nl();          go to next line
; ptr=inbuff;    point to begining of input buffer
; while(ptr++ < ibufptr-2) outbyte('*');    output '*' to point of error
; pl("^^^");  point to error
; nl();

RPTERR2             jsr       NL
                    ldx       IBUFPTR
                    dex:2
                    cpx       INBUFFS
                    bls       RPTERR4
                    stx       IBUFPTR
                    ldx       INBUFFS
                    lda       #'*'
RPTERR3             jsr       OUTBYTE
                    inx
                    cpx       IBUFPTR
                    bne       RPTERR3
RPTERR4             ldx       #ARROWS
                    jsr       PL
                    jsr       NL
                    bsr       RPTERR5
                    lda       #1
                    sta       CONTFLAG
                    jmp       MAIN3

; pl("ERROR # ");
; outdeci(errcode);
; return;
; *}

RPTERR5             ldx       #ERRORS
                    jsr       PL
                    ldb       ERRCODE
                    clra
                    jmp       OUTDECI

ARROWS              fcs       '^^^'
ERRORS              fcs       'ERROR # '

;*******************************************************************************

RPTRERR             proc                          ; REPORT A RUN TIME ERROR.
                    sta       ERRCODE
                    jsr       RPTERR5
                    ldx       #INLINE
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
                    jmp       MainLoop

BREAKS              fcc       'BREAK'
INLINE              fcs       ' IN LINE # '
          #ifz HC11
;*******************************************************************************
;***** outdeci() *****/
;
; *outdeci(num)
; *short num;
; *{
; int k,zs;
; char c;
; zs=0;
; k=10000;
; if(num<0)
; {
;  num=(-num);
;  outbyte('-');
; }

OUTDECI             proc
                    cpd       #0
                    bne       Go@@
                    lda       #'0'
                    jmp       OUTBYTE

Go@@                pshy
                    pshd
                    clrb
                    pshb:2
                    tsy
                    ldd       2,y
                    bpl       OUTDECI1
                    negd
                    std       2,y
                    lda       #'-'
                    jsr       OUTBYTE

; while(k>=1)
; {
;  c=num/k+'0';
;  if((c!='0') | (k==1) | (zs)) { zs=1; outbyte(c); }
;  num=num%k;
;  k=k/10;
; }
; return;
; *}

OUTDECI1            ldx       #PWRTEN
OUTDECI2            ldd       2,y
                    clr       1,y
OUTDECI3            subd      ,x
                    bmi       OUTDECI5
                    inc       1,y
                    bra       OUTDECI3

OUTDECI5            addd      ,x
                    std       2,y
                    lda       1,y
                    bne       OUTDECI6
                    tst       ,y
                    beq       OUTDECI4
OUTDECI6            adda      #$30
                    ldb       #1
                    stb       ,y
                    jsr       OUTBYTE
OUTDECI4            inx:2
                    tst       1,x
                    bne       OUTDECI2
                    ins:4
                    puly
                    rts

PWRTEN              fdb       10000
                    fdb       1000
                    fdb       100
                    fdb       10
                    fdb       1
                    fdb       0
          #else
OUTDECI             proc
                    cpd       #0
                    bne       Go@@
                    lda       #'0'
                    jmp       OUTBYTE

Go@@                pshy
                    pshd                          ; SAVE THE NUMBER TO PRINT.
                    ldd       #10000              ; NUMBER TO START DIVIDING BY.
                    pshd
                    clrb                          ; SET INITIAL VALUE OF LEADING ZERO SUPRESS FLAG.
                    pshb
                    tsy
                    ldd       3,y                 ; IS THE NUMBER NEGATIVE?
                    bpl       OUTDECI2            ; NO. GO PRINT THE NUMBER.
                    negd                          ; YES. MAKE THE NUMBER POSITIVE.
                    std       3,y                 ; SAVE THE RESULT.
                    lda       #'-'                ; PRINT A MINUS SIGN TO SHOW IT'S NEGATIVE.
                    jsr       OUTBYTE
OUTDECI2            ldd       3,y                 ; GET THE DIVIDEND.
                    ldx       1,y                 ; GET THE DIVISOR.
                    idiv                          ; DO THE DIVIDE.
                    std       3,y                 ; SAVE THE REMAINDER.
                    xgdx                          ; PUT QUOTIENT IN D.
                    cpd       #0                  ; IS THE QUOTIENT 0?
                    bne       OUTDECI3            ; NO. GO OUTPUT THE NUMBER.
                    tst       ,y                  ; YES. ARE WE STILL SUPRESSING LEADING ZEROS?
                    beq       OUTDECI4            ; YES. DON'T PRINT THE NUMBER.
OUTDECI3            tba                           ; PUT THE NUMBER IN THE A-REG.
                    adda      #$30                ; MAKE THE NUMBER ASCII.
                    ldb       #1                  ; MAKE THE ZERO SUPRESS FLAG NON-ZERO.
                    stb       ,y
                    jsr       OUTBYTE             ; OUTPUT THE NUMBER.
OUTDECI4            ldd       1,y                 ; GET CURRENT DIVISOR.
                    ldx       #10                 ; DIVIDE IT BY 10.
                    idiv
                    stx       1,y                 ; SAVE RESULT. ARE WE DONE?
                    bne       OUTDECI2            ; NO KEEP GOING.
                    ldb       #5                  ; DEALLOCATE LOCALS.
                    aby
                    tys
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.
          #endif
;                   include   'leditor.mod'
                    title     LEDITOR

;***** storlin() *****/
;
; *storlin()
; *{
; int *numptr,*linum;
; numptr=tknbuf;                 get int pointer into token buffer
; if(*numptr>hiline)             line # larger than current hi line*/
; {
;  apendlin();                   append it to the end of the buffer
;  hiline=*numptr;               make it the current hi line number
;  return;
; }

STORLIN             proc
                    lda       #1                  ; set the continue flag.
                    sta       CONTFLAG            ; we don't allow continues if the program has been altered.
                    ldx       TKNBUFS             ; point to the start of the token buffer
                    ldd       ,x                  ; get the first 2 bytes of the token buffer (the line number).
                    cpd       HILINE              ; was the entered lines number higher than the highest so far?
                    bls       Go@@                ; no. go do an insert or replace operation.
                    jsr       APENDLIN            ; yes. just append the line to the end of the program buffer.
                    ldx       TKNBUFS             ; point to the start of the token buffer
                    ldd       ,x                  ; get the first 2 bytes of the token buffer (the line number).
                    std       HILINE
                    rts                           ; return.

; linum=findline(*numptr);       look for line # in the program buffer
; if(*linum==*numptr)            is it the same line #?
; {
;  repline(linum);               yes. replace it with the new line
;  return;
; }
; insrtlin(linum);               no. insert the new line in the buffer
; return;
; *}

Go@@                bsr       FINDLINE
                    ldd       ,x
                    pshx
                    ldx       TKNBUFS
                    cpd       ,x
                    pulx
                    bne       INSRTLIN
                    jmp       REPLINE

;*******************************************************************************
; ***** delline() *****
;
; *delline(num)            delete line from basic buffer
; *int num;
; *{
; int *linum;
; char *ptr;
; if(num > hiline) return;       line number can't be there, return
; linum=findline(num);           look for the requested line #

DELLINE             proc
                    pshd                          ; SAVE THE LINE NUMBER TO DELETE.
                    tsy                           ; POINT TO THE LINE NUMBER WE SAVED.
                    cpd       HILINE              ; IS IT HIGHER THAN THE HIGHEST LINE ENTERED SO FAR?
                    bls       Go@@                ; NO. GO SEE IF THE LINE EXISTS.
Done@@              lda       #1                  ; YES. THE LINE CANNOT EXIST.
                    sta       CONTFLAG
                    puld                          ; PULL THE LINE NUMBER OFF THE STACK.
                    rts                           ; RETURN.

Go@@                bsr       FINDLINE            ; GO SEE IF THE LINE EXISTS.
; RETURN A POINTER TO A LINE NUMBER IN THE BASIC PROGRAM BUFFER.

; if(*linum!=num) return;        if the line # doesn't exist, return
; ptr=linum;                     make the int pointer a char pointer
; closespc(ptr[2],ptr);          go delete the line
; if(num==hiline) hiline=findhiln();
; return;
; *}
                    ldd       ,x                  ; GET THE LINE NUMBER THAT WAS FOUND.
                    cpd       ,y                  ; WAS THE LINE NUMBER FOUND THE ONE THAT WAS REQUESTED TO BE DELETED.
                    bne       Done@@              ; NO. THE LINE DOESN'T EXIST. JUST RETURN.
                    ldb       2,x                 ; YES. GET THE LENGTH OF THE LINE.
                    bsr       CLOSESPC            ; GO CLOSE THE SPACE IN THE PROGRAM BUFFER.
                    ldd       HILINE              ; GET THE HIGHEST LINE NUMBER ENTERED.
                    cpd       ,y                  ; DID WE DELETE THE HIGHEST LINE NUMBER?
                    bne       Done@@              ; NO. JUST RETURN.
                    bsr       FINDHILN            ; YES. GO FIND THE HIGHEST LINE NUMBER.
                    std       HILINE              ; SAVE IT.
                    bra       Done@@              ; RETURN.

;*******************************************************************************
;***** closespc() *****/        close up space in the BASIC buffer
;
; *closespc(bytes,ptr)
; *char bytes,*ptr;
; *{
; char *to,*from;                define the from/to pointers
; to=ptr;                        set up destination pointer
; from=ptr+bytes;                setup source pointer
; while(from<basend)             while we're not at the end of the buff
; { *to++=*from++; }             move source to destination
; basend=to;                     set new basend pointer
; return;
; *}

CLOSESPC            proc                          ; ENTERED WITH
                    pshy                          ; SAVE THE CURRENT VALUE OF Y.
                    txy                           ; TRANSFER X TO Y
                    aby                           ; ADD THE LENGTH TO Y.

Loop@@              cpy       BASEND              ; HAVE WE MOVED ALL THE BYTES?
                    bhs       Done@@              ; YES. RETURN.
                    lda       ,y                  ; NO. GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    inx                           ; ADVANCE THE DESTINATION POINTER.
                    iny                           ; ADVANCE THE SOURCE POINTER.
                    bra       Loop@@              ; GO CHECK TO SEE IF WE'RE DONE.

Done@@              stx       BASEND              ; SAVE THE NEW 'END OF BASIC PROGRAM' POINTER.
                    jsr       MoveVarsDn          ; MOVE ALL THE VARIABLES DOWN.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

;*******************************************************************************
;***** findline() *****/        return pointer to line number or next
;                                   highest line number
; *findline(linenum)
; *int linenum;
; *{
; char *linelen;
; int *basbufp;
; basbufp=basbeg;                set pointer to start of basic buffer
; while(*basbufp<linenum)        do until we find line # or one higher
; {
;  linelen=basbufp;              convert int pointer to char pointer
;  basbufp=linelen+linelen[2];   convert char ptr back to int pointer
; }
; return(basbufp);               return the pointer
; *}

FINDLINE            proc
                    ldx       BASBEG
Loop@@              cpd       ,x
                    bls       Done@@
                    pshb
                    ldb       2,x
                    abx
                    pulb
                    bra       Loop@@
Done@@              rts

;*******************************************************************************
;***** findhiln() *****
;
; *findhiln()                      find highest line number in basic buffer **{
; int *iptr,lnum;
; char *cptr;
; lnum=0;                        set line # to 0
; iptr=basbeg;                   set int pointer to basbeg
; while(iptr!=basend)            we're not to the end of the basic buff
; {
;  lnum=*iptr;                   get current line #
;  cptr=iptr;                    convert int pointer to char pointer
;  iptr=cptr+cptr[2];            add line length, point to next line
; }
; return(lnum);                  return highest line number
; *}

FINDHILN            proc
                    ldx       BASBEG
Loop@@              cpx       BASEND
                    beq       Done@@
                    ldd       ,x
                    pshb
                    ldb       2,x
                    abx
                    pulb
                    bra       Loop@@
Done@@              rts

;*******************************************************************************
;***** insrtlin() *****/
;
; *insrtlin(ptr)
; *char *ptr;
; *{
; openspc(tknbuf[2],ptr);        go open space in the program bufer
; if(errcode) return;            return if out of memory
; putline(ptr);                  put line into buffer
; return;
; *}

INSRTLIN            proc
                    pshx
                    ldx       TKNBUFS
                    ldb       2,x
                    pulx
                    pshx
                    bsr       OPENSPC
                    pulx
                    bra       PUTLINE

;*******************************************************************************
;***** openspc() *****/         open space in program buffer
;
; *openspc(bytes,ptr)
; *char bytes,*ptr;
; *{
; char *to,*from;                declare to/from pointers
; from=basend;                   set source at end of buffer
; to=basend+bytes;               set destination "bytes" beyond source
; if(to>basmend)                 if out of memory, return an error
; { errcode=OMEMERR; return; }
; basend=to;                     set new end of buffer
; while(from>=ptr)
; { *to--=*from--; }             open up area in buffer
; return;
; *}

OPENSPC             proc
                    pshy
                    pshx
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       Fail@@
                    jsr       MoveVarsUp
                    ldx       BASEND
                    pshx
                    abx
                    pshx
                    tsy
                    ldd       ,y
                    std       BASEND
Loop@@              ldd       2,y
                    cpd       4,y
                    blo       Done@@
                    ldx       2,y
                    lda       ,x
                    dex
                    stx       2,y
                    ldx       ,y
                    sta       ,x
                    dex
                    stx       ,y
                    bra       Loop@@

Fail@@              lda       #OMEMERR
                    jmp       RPTERR

Done@@              pulx:3
                    puly
                    rts

;*******************************************************************************
;***** putline() *****/         move line from token buffer to program
;                                   buffer
; *putline(cptr)
; *char *cptr;
; *{
; short count;
; count=tknbuf[2];               get length of line in token buffer
; tbufptr=tknbuf;                point to start of token buffer
; while(count)
; {
;  *cptr++=*tbufptr++;           move a byte
;  --count;                      decrement the byte count
; }
; return;
; *}

PUTLINE             proc
                    pshx
                    ldx       TKNBUFS
                    ldb       2,x
                    pulx
                    ldy       TKNBUFS
Loop@@              lda       ,y
                    iny
                    sta       ,x
                    inx
                    decb
                    bne       Loop@@
                    rts

;*******************************************************************************
;***** apendlin() *****/        append line to end of program buffer
;
; *apendlin()
; *{
; if((basend+tknbuf[2])<=basmend)  do we have enough memory left?
; {
;  putline(basend);              move the line
;  basend+=tknbuf[2];            set the new end of basic pointer
; }
; else errcode=OMEMERR;          not enough memory, error
; return;
; *}

APENDLIN            proc
                    ldx       TKNBUFS
                    ldb       2,x
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       Fail@@
;                   ldb       TKNBUF+2
                    jsr       MoveVarsUp
                    ldx       BASEND
                    abx
                    xgdx
                    ldx       BASEND
                    std       BASEND
                    bra       PUTLINE

Fail@@              lda       #OMEMERR
                    jmp       RPTERR

;*******************************************************************************
;***** repline() *****/         replace line in buffer
;
; *repline(ptr)
; *char *ptr;
; *{
; short lendif,temp1,temp2;
; temp1=*(ptr+2);                convert type from char to int
; temp2=(tknbuf[2]);
; lendif=temp1-temp2;            get the difference in line length
;      if(lendif==0)             if the same, just write over the old
;      {
;       putline(ptr);
;      }

REPLINE             proc
                    ldb       2,x
                    pshx
                    ldx       TKNBUFS
                    subb      2,x
                    pulx
                    bne       Go@@                ; /change to BEQ PUTLINE
                    bra       PUTLINE             ;/

; else if(lendif<0)              if line in tknbuf is larger
;      {
;       lendif=-lendif;          make it a positive number
;       openspc(lendif,ptr);     tru to open up a space
;       if(errcode) return;      if not enough memory, return
;       putline(ptr);            if ok, copy line to program buffer
;      }

Go@@                bpl       Shrink@@
                    negb
                    pshx
                    jsr       OPENSPC
                    pulx
                    bra       PUTLINE

; else                           if line in tknbuf is smaller
;      {
;       closespc(lendif,ptr);    close up excess space
;       putline(ptr);            put new line in program buffer
;      }
; return;
; *}

Shrink@@            pshx
                    jsr       CLOSESPC
                    pulx
                    bra       PUTLINE

;*******************************************************************************

MoveVarsUp          proc
                    pshy                          ; SAVE THE Y REGISTER.
                    pshb                          ; SAVE THE BYTE COUNT.
                    ldx       VAREND              ; POINT TO THE END OF THE VARIABLE MEMORY SPACE.
                    ldy       VAREND              ; POINT TO THE END OF VARIABLE MEMORY SPACE.
                    abx                           ; ADD THE NUMBER OF BYTES TO MOVE TO THE POINTER.
                    ldd       VAREND              ; GET THE CURRENT VARIABLE TABLE ENDING ADDRESS.
                    stx       VAREND              ; SAVE THE NEW END OF VARIABLE TABLE POINTER.
                    subd      VARBEGIN            ; CALCULATE THE NUMBER OF BYTES TO MOVE.
                    beq       Done@@              ; JUST RETURN IF THERE IS NOTHING TO MOVE.
                    std       VarSize             ; save the size of the variable table (9/12/89).
Loop@@              lda       ,y                  ; GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    dex
                    dey
                    bsr       DecCount            ; DECREMENT THE BYTE COUNT. ARE WE DONE? (9/12/89).
                    bpl       Loop@@              ; GO TILL WE'RE DONE.
                    inx                           ; ADJUST THE POINTER
Done@@              stx       VARBEGIN            ; SAVE THE NEW START OF VARIABLE TABLE POINTER.
                    pulb                          ; RESTORE THE BYTE COUNT.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

;*******************************************************************************

MoveVarsDn          proc
                    pshy                          ; SAVE Y.
                    pshb                          ; SAVE THE BYTE COUNT.
                    ldy       VARBEGIN            ; POINT TO THE CURRENT START OF THE VARIABLE TABLE.
                    lda       #$FF                ; MAKE THE BYTE COUNT NEGATIVE SO WE CAN JUST ADD.
                    negb
                    addd      VARBEGIN            ; CALCULATE THE NEW START OF THE VARIABLE TABLE.
                    xgdx                          ; PUT THE NEW STARTING ADDRESS OF THE VARIABLE TABLE INTO X.
                    ldd       VAREND              ; GET THE OLD TABLE ENDING ADDRESS.
                    subd      VARBEGIN            ; SUBTRACT THE OLD TABLE STARTING ADDRESS TO GET THE SIZE OF THE TABLE.
                    stx       VARBEGIN            ; SAVE THE POINTER TO THE NEW START OF THE VARIABLE TABLE.
                    std       VarSize             ; save the size of the variable table (9/12/89).
                    beq       Done@@              ; IF THE SIZE IS 0 (NO VARIABLES ALLOCATED) EXIT.
Loop@@              lda       ,y                  ; GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    inx                           ; MOVE THE DESTINATION POINTER.
                    iny                           ; MOVE THE SOURCE POINTER.
                    bsr       DecCount            ; DECREMENT THE BYTE COUNT. ARE WE DONE? (9/12/89).
                    bpl       Loop@@              ; NO. KEEP MOVIN' THEM BYTES.
                    dex
Done@@              stx       VAREND              ; SAVE THE NEW POINTER TO THE END OF THE VARIABLE TABLE.
                    pulb                          ; RESTORE THE BYTE COUNT.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

;*******************************************************************************

DecCount            proc
                    ldd       VarSize             ; get the size of the variable table.
                    decd                          ; decrement it.
                    std       VarSize             ; save the new value.
                    rts                           ; return.
;                   include   'inits.mod'
                    title     Inits

;*******************************************************************************
;***** initvars() *****/
;
; *initvars()
; *{
; char *x;
; varbegin=varend=0x7000;
; varmend=0x7FFF;
; for(x=varbegin; x<=varmend; x++) *x=0;
; basbeg=basend=0x4000;
; basmend=0x6FFF;
; for(x=basbeg; x<=basmend; x++) *x=0;
; hiline=0;
; return;
; *}

INITVARS            proc
                    ldx       RAMStart            ; YES. VARIABLES START AT RAMBEG.
                    stx       VARBEGIN            ; SET POINTERS TO BOTH START AND END OF VARIABLE
                    stx       VAREND              ; STORAGE.
                    stx       BASBEG              ; SET POINTERS TO BOTH THE START AND END OF THE
                    stx       BASEND              ; BASIC PROGRAM.
                    xgdx                          ; GET THE START OF RAM INTO D
                    addd      RAMSize             ; add the size of the RAM to it.
                    subd      #SWSTKSize+1        ; subtract the size of the software stack, token & input buffers.
                    std       VARMEND             ; SAVE THE POINTER.
                    std       BASMEND             ; MAKE IT THE END OF BASIC PROGRAM MEMORY.
                    incd                          ; Set up a pointer to the input buffer.
                    std       INBUFFS             ; Save the pointer.
                    addd      #IBUFLEN            ; add the length of the input buffer to create a pointer to the token buffer.
                    std       TKNBUFS             ; save the pointer.
                    addd      #TBUFLEN            ; add the length of the token buffer to create a pointer to the end of the operand stack.
                    std       EOPSTK              ; save the pointer to the end of the operator stack.
                    addd      #OPSLEN             ; add the length of the operand stack.
                    std       STOPS               ; save the pointer to the start of the operator stack.
                    std       ENUMSTK             ; also make it the end of the operand stack.
                    addd      #NUMSLEN            ; add the length of the operand stack.
                    std       STNUMS              ; save the result as the start of the operand stack.
                    std       EFORSTK             ; also make it the end of the FOR stack.
                    addd      #FORSLEN            ; Add the length of the FOR stack.
                    std       STFORSTK            ; save the result as the start of the FOR stack.
                    std       EWHSTK              ; also make it the end of the while stack.
                    addd      #WHSLEN             ; add the length of the while stack.
                    std       STWHSTK             ; save the pointer as the start of the while stack.
                    std       EGOSTK              ; also make it the end of the GOSUB stack.
                    addd      #GOSLEN             ; add the length of the GOSUB stack.
                    std       STGOSTK             ; save it as the start of the GOSUB stack.
                    ldx       BASBEG              ; point to the start of the basic program buffer.
Loop@@              clr       ,x                  ; CLEAR THE STORAGE TO ZERO.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       BASMEND             ; ARE WE DONE?
                    bls       Loop@@              ; NO. KEEP CLEARING.
                                                  ; YES. POINT TO THE PHYSICAL END OF MEMORY.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG AGAIN.
                    cmpa      #$55                ; IS THE AUTO START MODE SET?
                    bne       NormalInit@@        ; NO. DO A NORMAL INIT.

                    jsr       AUTOLOAD            ; GO LOAD the program and VARIABLES INTO RAM.
NormalInit@@        ldd       #0                  ; MAKE THE HIGHEST LINE IN THE PROGRAM 0.
                    std       HILINE
                    std       CURLINE             ; MAKE THE CURRENT LINE #0.
                    jsr       RUNINIT             ; GO INITIALIZE ALL THE SOFTWARE STACKS.
                    clr       TRFLAG              ; TURN THE TRACE MODE OFF.
                    lda       #1                  ; "CONT" COMMAND NOT ALLOWED.
                    sta       CONTFLAG
                    ldx       DFLOPADR            ; point to the D-Flip flop address.
                    sta       ,x                  ; CONNECT SCI RX PIN TO THE HOST CONNECTOR.
                    clr       DEVNUM              ; MAKE THE DEFAULT DEVICE NUMBER 0 (CONSOLE).
                    clr       IMMED               ; clear the immediate mode flag (added 9/17/90).
                    ldx       VAREND              ; GET THE POINTER TO THE END OF VARIABLE STORAGE.
                    inx                           ; BUMP IT BY 1.
                    stx       STRASTG             ; POINT TO THE DYNAMIC ARRAY STORAGE.
                    rts                           ; RETURN.

;*******************************************************************************

POWERUP             proc
                    ldd       IOBase              ; GET THE BASE ADDRESS OF THE I/O REGISTERS.
                    std       IOBaseV
                    lsra:4
                    sta       $103D               ; remap the I/0 regs to where the user wants them.
                    ldx       IOBaseV             ; point to the I/O Register Base.
                    lda       #$93                ; TURN ON A/D, USE E CLOCK, SET IRQ LEVEL SENSITIVE
                    sta       OPTION,x            ; DELAY AFTER STOP, DISABLE CLOCK MONITOR, SET COP
                                                  ; TIMOUT PERIOD TO MAX.
                    lda       #$03                ; SET THE TIMER PRESCALER TO /16.
                    sta       TMSK2,x

                    ldd       RAMStart            ; Get start of RAM.
                    addd      RAMSize             ; Add the size of the RAM to it.
                    xgdx                          ; Put the calculated address into X.
                    txs                           ; Transfer the address to the stack pointer.

                    ldx       #RAMVECTS           ; POINT TO THE RAM INTERRUPT VECTOR TABLE.
                    ldy       #RETII              ; GET ADDRESS OF RTI INSTRUCTION.
                    ldb       #20                 ; PUT THE "JMP" OPCODE INTO ALL VECTOR LOCATIONS.
                    lda       #JMPOP              ; GET THE JMP OPCODE.
Loop@@              sta       ,x                  ; STORE IT.
                    inx                           ; POINT TO THE NEXT VECTOR.
                    sty       ,x                  ; INITIALIZE VECTOR TO "RTI".
                    inx:2
                    decb                          ; DONE?
                    bne       Loop@@              ; NO. INITIALIZE ALL VECTORS.
                    ldx       #ILLOP              ; POINT TO THE ILLEGAL OP-CODE VECTOR.
                    ldd       #POWERUP            ; GET THE ADDRESS OF THE POWER UP VECTOR.
                    std       1,x                 ; INITIALIZE ILLEGAL OP-CODE VECTOR.
                    std       4,x                 ; INITIALIZE WATCH DOG FAIL VECTOR.
                    std       7,x                 ; INITIALIZE CLOCK MONITOR FAIL VECTOR.

                    ldx       #INTABLE            ; POINT TO THE START OF THE I/O VECTOR TABLE.
                    ldy       #IOVects            ; point to the default table in ROM.
                    ldb       #32                 ; GET NUMBER OF BYTES IN THE TABLE.
ToRam@@             lda       ,y                  ; Move a byte of the table from ROM into RAM.
                    sta       ,x
                    inx                           ; POINT TO THE NEXT BYTE.
                    iny
                    decb                          ; DECREMENT THE COUNT.
                    bne       ToRam@@             ; GO TILL WE'RE DONE.

                    ldx       #TIMEINT            ; GET THE ADDRESS OF THE OUTPUT COMPARE 1 ROUTINE.
                    stx       TOC1+1              ; PUT IT IN THE INTERRUPT VECTOR.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    clrd
                    std       TIMEREG             ; ZERO THE TIME REGISTER.
                    std       TIMECMP             ; zero the time compare register.
                    ldx       IOBaseV
                    bsr       TIMINTS             ; GO SETUP THE TIMER FOR THE FIRST INTERRUPT.
                    lda       #$80                ; ENABLE INTERRUPTS FROM OC1.
                    sta       TMSK1,x

                    ldx       #IRQINT             ; GET THE ADDRESS OF THE IRQ SERVICE ROUTINE.
                    stx       IRQI+1              ; PUT IT IN THE IRQ VECTOR.
                    ldx       #PACCINT            ; GET THE ADDRESS OF THE PACC INT ROUTINE.
                    stx       PACCIE+1            ; SET ADDRESS IN INPUT EDGE INTERRUPT VECTOR.
                    stx       PACCOVF+1           ; SET ADDRESS IN PACC OVERFLOW INTERRUPT VECTOR.
                    clrd
                    std       ONTIMLIN            ; INITIALIZE THE LINE POINTERS.
                    std       ONIRQLIN
                    std       ONPACLIN

                    ldx       UserInit
                    jsr       ,x                  ; INITIALIZE THE ACIA & SCI.
                    jmp       MAIN                ; GO TO BASIC.

;*******************************************************************************

TIMEINT             proc
                    bsr       TIMINTS
RETII               rti                           ; RETURN FROM ALL INTERRUPT SOURCES.

;*******************************************************************************

TIMINTS             proc
                    ldx       IOBaseV             ; Point to the I/O Base Address.
                    ldd       TOC1REG,x           ; GET THE VALUE OF THE TIMER/COUNTER.
;                   bra       TIMINTS3

;*******************************************************************************

TIMINTS3            proc
                    addd      TimeVal             ; ADD IN 62500 FOR NEXT COMPARE ( 2 HZ INT.).
                    std       TOC1REG,x           ; PUT IT IN THE OUTPUT COMPARE REGISTER.
                    lda       #$80                ; SETUP TO CLEAR THE OC1 FLAG.
                    sta       TFLAG1,x
                    dec       TIMEPRE             ; HAVE TWO OUTPUT COMPARES OCCURED?
                    bne       Done@@              ; NO. JUST RETURN.
                    lda       #SWPRE              ; YES. RELOAD THE REGISTER.
                    sta       TIMEPRE
                    ldd       TIMEREG             ; GET THE CURRENT VALUE OF "TIME".
                    incd                          ; ADD 1 SECOND TO THE COUNT.
                    std       TIMEREG             ; UPDATE THE TIME REGISTER.
                    ldd       TIMECMP             ; GET THE VALUE TO COMPARE TO FOR "ONTIME".
                    beq       Done@@              ; IF IT'S 0, THE "ONTIME" FUNCTION IS OFF.
                    cpd       TIMEREG             ; DOES THE COMPARE VALUE MATCH THE TIME REGISTER?
                    bne       Done@@              ; NO. JUST RETURN.
                    ldy       ONTIMLIN            ; MAKE THE POINTER TO THE LINE NUMBER THE NEW IP.
                    ins:2                         ; GET RID OF THE RETURN ADDRESS.
?TIMINTS            inc       IMMED               ; FAKE THE GOTO ROUTINE OUT.
                    ldd       CURLINE             ; SAVE THE CURRENT LINE NUMBER IN MAIN PROGRAM.
                    std       SCURLINE
                    ldd       ADRNXLIN            ; SAVE THE ADDRESS OF THE NEXT LINE IN MAIN PROG.
                    std       SADRNXLN
                    jmp       RGOTO3              ; GOTO THE SERVICE ROUTINE.
Done@@              rts                           ; RETURN.

;*******************************************************************************

IRQINT              proc
                    ldy       ONIRQLIN            ; GET POINTER TO LINE NUMBER OF THE IRQ SERVICE.
                    bne       ?TIMINTS            ; GO DO IT.
                    rti                           ; IF IT'S 0, "ONIRQ" HAS NOT BEEN EXECUTED.

;*******************************************************************************

PACCINT             proc
                    ldx       IOBaseV
                    lda       #$30                ; RESET BOTH THE TIMER OVERFLOW & INPUT FLAG.
                    sta       TFLG2,x
                    ldy       ONPACLIN            ; GET POINTER TO LINE NUMBER OF THE SERVICE ROUT.
                    bne       ?TIMINTS
                    rti

;                   include   'command1.mod'
                    title     COMMAND1

;*******************************************************************************
;***** chckcmds() *****/
;
; *chckcmds()
; *{
; if(match("LIST")) clist();
; else if(match("RUN")) crun();
; else if(match("NEW")) cnew();
; else if(match("CONT")) ccont();
; else if(match("CLEAR")) cclear();
; else return(0);
; return(1);
; *}

CHCKCMDS            proc
                    jsr       GETCHR              ; GET FIRST CHAR FROM THE INPUT BUFFER.
                    cmpa      #EOL                ; IS IT AN EOL?
                    bne       Go@@                ; NO. GO CHECK FOR COMMANDS.
NotFound@@          ldd       #0                  ; YES. JUST RETURN.
                    rts

Go@@                ldx       #Table@@            ; POINT TO COMMAND TABLE.
Loop@@              jsr       STREQ               ; GO CHECK FOR A COMMAND.
                    bcs       Found@@             ; IF WE FOUND ONE GO EXECUTE IT.
ToEnd@@             inx                           ; ADVANCE POINTER TO NEXT CHAR IN TABLE ENTRY.
                    lda       ,x                  ; GET THE CHAR. ARE WE AT THE END OF THIS ENTRY?
                    bne       ToEnd@@             ; NO. KEEP GOING TILL WE ARE PAST IT.
                    inx:3                         ; BYPASS END OF COMMAND MARKER & EXECUTION ADDR.
                    tst       ,x                  ; ARE WE AT THE END OF THE TABLE?
                    bne       Loop@@              ; NO. GO CHECK THE NEXT TABLE ENTRY.
                    bra       NotFound@@          ; YES. RETURN W/ ENTRY NOT FOUND INDICATION.

Found@@             ldx       1,x                 ; GET ADDRESS OF COMMAND.
                    jsr       ,x                  ; GO DO IT.
                    ldd       #1                  ; SHOW WE EXECUTED A COMMAND.
                    rts                           ; RETURN.

Table@@             @token    'LIST',CLIST
                    @token    'RUN',CRUN
                    @token    'NEW',CNEW
                    @token    'CONT',CCONT
                    @token    'CLEAR',CCLEAR
                    @token    'ESAVE',CESAVE
                    @token    'ELOAD',CELOAD
                    @token    'LLIST',CLLIST
                    @token    'AUTOST',CAUTOST
                    @token    'NOAUTO',CNOAUTO
                    @token    'FREE',CFREE
                    fcb       0                   ; END OF TABLE MARKER.

;*******************************************************************************
;***** clist() *****/
;
; *clist()
; *{
; int *intptr;
; char token;
; if(basbeg==basend) return;             if basic buffer empty, return
; skipspcs();                    skip any spaces after "LIST"

CLIST               proc
                    jsr       NL2
                    ldd       BASBEG
                    cpd       BASEND
                    bne       Go@@
                    rts

Go@@                jsr       SKIPSPCS

; if(numeric(getchr()))
; {
;  firstlin=getlinum();
;  if(getchr()=='-')
;  {
;   incibp();
;   lastlin=getlinum;
;  }
; }
; else
; {
;  intptr=basbeg;
;  lastlin=hiline;
;  firstlin=*intptr;
; }
; if(firstlin<lastlin) return;
; tokptr=intptr=findline(firstlin);

                    jsr       GETCHR
                    jsr       NUMERIC
                    bcc       CLIST2
                    jsr       GETLINUM
                    std       FIRSTLIN
                    jsr       GETCHR
                    cmpa      #'-'
                    beq       Range@@
                    ldd       FIRSTLIN
                    std       LASTLIN
                    cpd       HILINE
                    bls       CLIST4
                    rts

Range@@             jsr       INCIBP
                    jsr       GETLINUM
                    cpd       HILINE
                    bls       SaveLastLine@@
                    ldd       HILINE
SaveLastLine@@      std       LASTLIN
                    bra       CLIST4

CLIST2              cmpa      #EOL
                    beq       Process@@
                    rts

Process@@           ldx       BASBEG
                    ldd       ,x
                    std       FIRSTLIN
                    ldd       HILINE
                    std       LASTLIN
CLIST4              ldd       FIRSTLIN
                    cpd       LASTLIN
                    bls       CLIST5
                    rts

CLIST5              ldd       FIRSTLIN
                    jsr       FINDLINE
                    stx       TOKPTR
                    ldd       LASTLIN
                    jsr       FINDLINE
                    ldd       ,x
                    cpd       LASTLIN
                    bne       CLIST12
                    ldb       2,x
                    abx
CLIST12             stx       LASTLIN

; while(*intptr<lastlin)
; {
;  intptr=tokptr;
;  outdeci(*intptr);
;  tokptr+=3;
;  while(*tokptr!=EOLTOK)
;  {
;   token=gettok;
;   if(token>=0x80)
;   {
;    lvarcon();
;   }
;   else
;   {
;    lkeyword();
;   }
;  }
;  nl();
;  ++tokptr;
; }
; return;
; *}

Loop@@              ldd       TOKPTR
                    cpd       LASTLIN
                    bne       CLIST7
                    rts

CLIST7              ldx       TOKPTR
                    ldd       ,x
                    inx:3
                    stx       TOKPTR
                    jsr       OUTDECI
CLIST8              ldx       TOKPTR
                    lda       ,x
                    cmpa      #EOLTOK
                    beq       CLIST9
                    tsta
                    bmi       CLIST10
                    jsr       LKEYWORD
                    bra       CLIST8

CLIST10             jsr       LVARCON
                    bra       CLIST8

CLIST9              jsr       NL
                    ldx       TOKPTR
                    inx
                    stx       TOKPTR
                    bra       Loop@@

;*******************************************************************************
;***** lvarcon() *****/
;
; *lvarcon()
; *{
; char tok;
; tok=gettok;
; if(tok<=0x88)
; {
;  if(tok==FVARTOK) lfvar();
;  else if(tok==SVARTOK) lsvar();
;  else if(tok==IVARTOK) livar();
;  else { errcode=ILTOKERR; return; }
; }
; else if(tok<=0xA8)
; {
;  if(tok==FCONTOK) lfcon();
;  else if(tok==SCONTOK) lscon();
;  else if(tok==LCONTOK) llcon();
;  else if(tok==ICONTOK) licon();
;  else { errcode=ILTOKERR; return; }
; }
; else { errcode=ILTOKERR; return; }
; *}

LVARCON             proc
                    ldx       TOKPTR
                    lda       ,x
                    anda      #$EF                ; MASK OFF ARRAY INDICATOR IF PRESENT.
                    ldx       #Table@@
Loop@@              cmpa      ,x
                    beq       Found@@
                    inx:3
                    tst       ,x
                    bne       Loop@@
                    lda       #ILTOKERR
                    jmp       RPTERR

Found@@             ldx       1,x
                    jsr       ,x
                    rts

Table@@             @token    IVARTOK,LIVAR
                    @token    SCONTOK,LSCON
                    @token    LCONTOK,LLCON
                    @token    ICONTOK,LICON
                    fcb       0                   ; END OF TABLE MARKER.

;*******************************************************************************
;***** livar() *****/
;
; *livar()
; *{
; lfvar();
; outbyte('%');
; return;
; *}

LIVAR               proc
                    ldx       TOKPTR
                    inx
                    ldd       ,x
                    addd      VARBEGIN
                    inx:2
                    stx       TOKPTR
                    xgdx
                    lda       1,x
                    jsr       OUTBYTE
                    lda       2,x
                    beq       Done@@
                    jsr       OUTBYTE
Done@@              rts

;*******************************************************************************

LFCON               proc
                    ldd       TOKPTR
                    addd      #FSIZ+1
?LFCON              xgdx
                    ldb       ,x
                    inx
Loop@@              lda       ,x
                    jsr       OUTBYTE
                    inx
                    decb
                    bne       Loop@@
                    stx       TOKPTR
                    rts

;*******************************************************************************
;***** licon() *****/
;
; *licon()
; *{
; int count;
; tokptr=tokptr+ISIZ+1;
; count=*tokptr++;
; while(count--)
; {
;  outbyte(*tokptr++);
; }
; return;
; *}

LICON               proc
                    ldd       TOKPTR
                    addd      #ISIZ+1
                    bra       ?LFCON

;*******************************************************************************
;***** lscon() *****/
;
; *lscon()
; *{
; int count;
; ++tokptr;
; count=*tokptr++;
; while(count--)
; {
;  outbyte(*tokptr++);
; }
; return;
; *}

LSCON               proc
                    ldd       TOKPTR
                    incd
                    bra       ?LFCON

;*******************************************************************************
;***** llcon *****/
;
; *llcon()
; *{
; int *intptr;
; intptr=++tokptr;
; tokptr+=2;
; outdeci(*intptr);
; return;
; *}

LLCON               proc
                    ldx       TOKPTR
                    inx
                    ldd       ,x
                    inx:2
                    stx       TOKPTR
                    jsr       OUTDECI
                    rts

;*******************************************************************************
;***** lkeyword *****/
;
; *lkeyword()
; *{
; char *charptr,token;
; token=*tokptr++

LKEYWORD            proc
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    cmpa      #MSCNTOK
                    bne       _1@@
                    jmp       LMSPCS

_1@@                cmpa      #REMTOK
                    bne       _2@@
                    jmp       LREMLINE

_2@@                cmpa      #DATATOK
                    bne       _3@@
                    jmp       LDATALIN

_3@@                cmpa      #FUNCTFLG
                    bne       _4@@
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    ldx       #LFUNCTBL
                    bra       Loop@@

_4@@                ldx       #TOKTBL
Loop@@              cmpa      ,x
                    beq       Done@@
                    inx:3
                    tst       ,x
                    bne       Loop@@
                    lda       #ILTOKERR
                    jmp       RPTERR

Done@@              ldx       1,x
                    jmp       PL

;*******************************************************************************

LMSPCS              proc
                    ldx       TOKPTR
                    ldb       ,x
                    inx
                    stx       TOKPTR
                    lda       #$20
Loop@@              jsr       OUTBYTE
                    decb
                    bne       Loop@@
                    rts

;*******************************************************************************

LDATALIN            proc
                    ldx       #DATA
                    jsr       PL
                    bra       ?LREM

;*******************************************************************************

LREMLINE            proc
                    ldx       #REM
                    jsr       PL
?LREM               ldx       TOKPTR
                    inx                           ; PUT POINTER PAST LENGTH BYTE.
Loop@@              lda       ,x
                    cmpa      #EOL
                    bne       Cont@@
                    inx
                    stx       TOKPTR
                    rts

Cont@@              jsr       OUTBYTE
                    inx
                    bra       Loop@@

;*******************************************************************************

TOKTBL              @token    LETTOK,LET
                    @token    READTOK,READ
                    @token    RESTRTOK,RESTORE
                    @token    GOSUBTOK,GOSUB
                    @token    GOTOTOK,GOTO
                    @token    ONTOK,ON
                    @token    RETNTOK,RETURN
                    @token    IFTOK,IIF
                    @token    THENTOK,THENS
                    @token    ELSETOK,ELSES
                    @token    INPUTTOK,INPUT
                    @token    PRINTTOK,PRINT
                    @token    FORTOK,FOR
                    @token    NEXTTOK,NEXT
                    @token    STOPTOK,STOPSS
                    @token    ENDTOK,ENDS
                    @token    TRONTOK,TRON
                    @token    TROFFTOK,TROFF
                    @token    WHILETOK,WHILE
                    @token    ENDWHTOK,ENDWH
                    @token    STEPTOK,STEP
                    @token    TOTOK,TO
                    @token    COMMATOK,COMMAC
                    @token    SEMITOK,SEMIC
                    @token    MEOLTOK,COLLINC
                    @token    IMLETTOK,IMLET
                    @token    POKETOK,POKE
                    @token    EQUALTOK,EQ
                    @token    OPARNTOK,OPARN
                    @token    CPARNTOK,CPARN
                    @token    ANDTOK,ANDS
                    @token    ORTOK,ORS
                    @token    EORTOK,EORS
                    @token    LTTOK,LT
                    @token    GTTOK,GT
                    @token    LTEQTOK,LTEQ
                    @token    GTEQTOK,GTEQ
                    @token    EQTOK,EQ
                    @token    NOTEQTOK,NOTEQ
                    @token    PLUSTOK,PLUS
                    @token    MINUSTOK,MINUS
                    @token    MULTTOK,MULT
                    @token    DIVTOK,DIV
                    @token    MODTOK,MODS
                    @token    NOTTOK,NOTS
                    @token    RTIMETOK,RTIMES
                    @token    NEGTOK,NEGS
                    @token    SSCNTOK,SPACE
                    @token    DIMTOK,DIM
                    @token    EEPTOK,EEP
                    @token    PORTATOK,PORTA
                    @token    PORTBTOK,PORTB
                    @token    PORTCTOK,PORTC
                    @token    PORTDTOK,PORTD
                    @token    PNUMTOK,POUNDSGN
                    @token    INBYTTOK,INBYTES
                    @token    TIMETOK,TIME
                    @token    ONTIMTOK,ONTIME
                    @token    ONIRQTOK,ONIRQ
                    @token    RETITOK,RETI
                    @token    PACCTOK,PACC
                    @token    ONPACTOK,ONPACC
                    @token    SLEEPTOK,SLEEP
                    fcb       0                   ; END OF TABLE MARKER.

LFUNCTBL            @token    FDIVTOK,FDIVS
                    @token    CHRTOK,CHRS
                    @token    ADCTOK,ADCS
                    @token    ABSTOK,ABS
                    @token    RNDTOK,RND
                    @token    SGNTOK,SGN
                    @token    TABTOK,TABS
                    @token    CALLTOK,CALL
                    @token    PEEKTOK,PEEK
                    @token    FEEPTOK,EEP
                    @token    HEXTOK,HEX
                    @token    FPRTATOK,PORTA
                    @token    FPRTBTOK,PORTB
                    @token    FPRTCTOK,PORTC
                    @token    FPRTDTOK,PORTD
                    @token    FPRTETOK,PORTE
                    @token    FTIMETOK,TIME
                    @token    HEX2TOK,HEX2
                    @token    FPACCTOK,PACC
IMLET               fcb       0                   ; NO KETWORD TO PRINT FOR AN IMPLIED LET.
COLLINC             fcs       ':'
SEMIC               fcs       ';'
COMMAC              fcs       ','
OPARN               fcs       '('
CPARN               fcs       ')'
SPACE               fcs       ' '
PORTE               fcs       'PORTE'
POUNDSGN            fcs       '#'

;*******************************************************************************

CRUN                proc
                    jsr       NL2                 ; DO 2 CR/LF SEQUENCES.
                    jsr       RUNINIT             ; INITIALIZE RUNTIME VARIABLES.
                    lda       #1                  ; SET THE RUN MODE FLAG.
                    sta       RUNFLAG

;        END OF POINTER INITIALIZATIONS

                    ldy       BASBEG              ; POINT TO THE START OF THE PROGRAM.
                    cpy       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       Go@@                ; YES. GO RUN IT.
                    rts                           ; NO. RETURN.

Go@@                ldd       ,y                  ; GET NUMBER OF FIRST/NEXT LINE OF BASIC PROGRAM.
                    std       CURLINE             ; MAKE IT THE CURRENT LINE.
                    tst       TRFLAG              ; IS THE TRACE MODE TURNED ON?
                    beq       Skip@@              ; NO. CONTINUE.
                    lda       #'['                ; YES. PRINT THE CURRENT LINE.
                    jsr       OUTBYTE
                    ldd       CURLINE
                    jsr       OUTDECI
                    lda       #']'
                    jsr       OUTBYTE
                    jsr       NL
Skip@@              pshy                          ; SAVE POINTER TO START OF NEW LINE.
                    ldb       2,y                 ; GET LENGTH OF LINE.
                    aby                           ; POINT TO START OF NEXT LINE.
                    sty       ADRNXLIN            ; SAVE THE ADDRESS OF THE NEXT LINE.
                    puly
                    ldb       #3                  ; BYTE COUNT OF LINE NUMBER & LENGTH.
                    aby                           ; POINT TO THE FIRST TOKEN.
Loop@@              bsr       RSKIPSPC            ; SKIP SPACES IF PRESENT.
                    ldb       ,y                  ; GET KEYWORD TOKEN.
                    iny                           ; POINT PAST THE KEYWORD.
                    bsr       RSKIPSPC            ; SKIP SPACES AFTER KEYWORD.
                    decb                          ; SUBTRACT ONE FOR INDEXING.
                    lslb                          ; MULTIPLY BY THE # OF BYTES PER ADDRESS.
                    ldx       #RKEYWORD           ; POINT TO RUN TIME ADDRESS TABLE.
                    abx                           ; POINT TO ADDRESS
                    ldx       ,x                  ; POINT TO RUNTIME ROUTINE.
                    jsr       ,x                  ; GO DO IT.

                    dec       BREAKCNT            ; SHOULD WE CHECK FOR A BREAK YET?
                    bne       CRUN7               ; NO. CONTINUE.
                    jsr       CHCKBRK             ; CHECK FOR BREAK FROM CONSOLE.

CRUN7               bsr       RSKIPSPC            ; SKIP ANY SPACES.
                    lda       ,y                  ; GET THE NEXT TOKEN IN THE LINE.
                    cmpa      #EOLTOK             ; ARE WE AT THE END OF THE LINE?
                    bne       Cont@@
                    iny                           ; YES. POINT TO START OF THE NEXT LINE.
CRUN1               cpy       BASEND              ; HAVE WE REACHED THE END OF THE BASIC PROGRAM?
                    bne       Go@@                ; NO. GO EXECUTE THE NEXT LINE.
                    jmp       REND                ; GO DO AN "END".

Cont@@              iny                           ; MUST BE A MID EOL.
                    bra       Loop@@              ; GO DO NEXT KEYWORD.

;*******************************************************************************

RSKIPSPC            proc
                    lda       ,y                  ; GET A CHARACTER.
                    bmi       Done@@
                    cmpa      #SSCNTOK            ; IS IT A SINGLE SPACE?
                    beq       Skip@@              ; YES. BUMP IP BY 1.
                    blo       Done@@
                    iny                           ; BUMP IP BY 2 FOR MULTIPLE SPACES.
Skip@@              iny                           ; BUMP IP.
Done@@              rts                           ; RETURN.

;*******************************************************************************

RKEYWORD            fdb       RLET
                    fdb       RLET
                    fdb       RPRINT
                    fdb       RFOR
                    fdb       RNEXT
                    fdb       RTRON
                    fdb       RTROFF
                    fdb       RPOKE
                    fdb       RDIM
                    fdb       RREM
                    fdb       RPACC
                    fdb       RDATA
                    fdb       RREAD
                    fdb       RRESTOR
                    fdb       RGOSUB
                    fdb       0
                    fdb       0
                    fdb       RGOTO
                    fdb       RON
                    fdb       RRETURN
                    fdb       RIF
                    fdb       RINPUT
                    fdb       RSTOP
                    fdb       REND
                    fdb       RWHILE
                    fdb       RENDWH
                    fdb       REEP
                    fdb       RPORTA
                    fdb       RPORTB
                    fdb       RPORTC
                    fdb       RPORTD
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       RINBYTE
                    fdb       RTIME
                    fdb       RONTIME
                    fdb       RONIRQ
                    fdb       RRETI
                    fdb       RONPACC
                    fdb       RSLEEP
                    fdb       RRTIME

;*******************************************************************************

RUNLINE             proc
                    jsr       NL2
                    ldy       TKNBUFS             ; POINT TO THE TOKEN BUFFER.
                    ldd       ,y                  ; GET CURRENT LINE NUMBER.
                    std       CURLINE             ; MAKE "0" THE CURRENT LINE #.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER & LENGTH.
                    aby
Loop@@              bsr       RSKIPSPC            ; SKIP SPACES.
                    ldb       ,y                  ; GET KEYWORD.
                    iny                           ; POINT PAST KEYWORD.
                    bsr       RSKIPSPC            ; SKIP SPACES.
                    decb                          ; SUBTRACT ONE FOR INDEXING.
                    lslb                          ; MULT BY THE # OF BYTES/ADDRESS.
                    ldx       #RKEYWORD           ; POINT TO ADDRESS TABLE.
                    abx                           ; POINT TO ADDRESS OF RUN TIME ROUTINE.
                    ldx       ,x                  ; GET ADDRESS.
                    jsr       ,x                  ; GO DO IT.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y
                    cmpa      #EOLTOK             ; ARE WE AT THE END OF THE LINE?
                    bne       Cont@@
                    rts

Cont@@              iny                           ; MUST BE A MID EOL.
                    bra       Loop@@

;*******************************************************************************

CHCKBRK             proc
                    lda       #10                 ; RELOAD THE BREAK CHECK COUNT.
                    sta       BREAKCNT
                    jsr       CONSTAT             ; GET CONSOLE STATUS. CHARACTER TYPED?
                    bne       Go@@                ; YES. GO CHECK IT OUT.
                    rts                           ; NO. RETURN.

Go@@                jsr       INCONNE             ; GET BYTE FROM CONSOLE BUT DON'T ECHO.
                    cmpa      #$03                ; WAS IT A CONTROL-C?
                    beq       ?DoBreak            ; YES. GO DO A BREAK.
                    rts                           ; NO. RETURN.

?DoBreak            sty       IPSAVE              ; SAVE THE IP POINTER IN CASE OF A CONTINUE.
                    jsr       NL
                    ldx       #BREAKS             ; POINT TO BREAK STRING.
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    jsr       NL
                    jmp       MainLoop

;*******************************************************************************

RUNINIT             proc
                    jsr       CCLEAR              ; GO CLEAR ALL VARIABLE STORAGE.
?RUNINIT            ldx       STNUMS              ; GET START OF NUMERIC OPERAND STACK.
                    stx       NUMSTACK            ; INITIALIZE THE OPERAND STACK POINTER.
                    ldx       STOPS               ; GET THE START OF THE OPERATOR STACK.
                    stx       OPSTACK             ; INITIALIZE THE OPREATOR STACK POINTER.
                    ldx       STFORSTK            ; GET THE START OF THE FOR-NEXT STACK.
                    stx       FORSTACK            ; INITIALIZE THE FOR NEXT STACK POINTER.
                    ldx       STWHSTK             ; GET THE START OF THE WHILE STACK.
                    stx       WHSTACK             ; INITIALIZE THE WHILE STACK POINTER.
                    ldx       STGOSTK             ; GET THE START OF THE GOSUB STACK.
                    stx       GOSTACK             ; SET THE START OF THE GOSUB STACK.
                    ldx       VAREND              ; GET THE VARIABLE END POINTER.
                    inx                           ; POINT TO THE NEXT AVAILABLE BYTE.
                    stx       STRASTG             ; INITIALIZE THE STRING/ARRAY STORAGE POINTER.
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    lda       #10                 ; SET COUNT FOR BREAK CHECK.
                    sta       BREAKCNT
                    clr       CONTFLAG            ; CLEAR THE CONTINUE FLAG.
                    clrx                          ; CLEAR THE DATA POINTER.
                    stx       DATAPTR
                    rts

;*******************************************************************************

CCONT               proc
                    jsr       NL2
                    tst       CONTFLAG
                    bne       Fail@@
                    ldy       IPSAVE
                    jmp       CRUN7

Fail@@              lda       #CNTCNERR
                    sta       ERRCODE
                    jmp       RPTERR5

;*******************************************************************************

CNEW                proc
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG.
                    cmpa      #$55                ; IS IT SET?
                    bne       Init@@              ; NO. GO INITIALIZE EVERYTHING.
                    lda       #$FF                ; YES. RESET (ERASE) IT.
                    sta       AUTOSTF,x
                    jsr       DLY10MS
Init@@              jsr       INITVARS            ; INITIALIZE EVERYTHING.
                    rts                           ; RETURN.

;*******************************************************************************

CCLEAR              proc
                    jsr       ?RUNINIT            ; GO INITIALIZE ALL STACKS ETC.
;                   bra       CCLEAR3

;*******************************************************************************

CCLEAR3             proc
                    ldx       VARBEGIN
Loop@@              lda       ,x
                    beq       Done@@
                    inx:3
                    jsr       CLRVAR
                    bra       Loop@@

Done@@              ldx       VAREND
                    inx
                    stx       STRASTG
                    rts

;                   include   'command2.mod'
                    title     COMMAND2

;*******************************************************************************

CESAVE              proc
                    ldd       BASBEG              ; GET POINTER TO THE START OF THE BASIC PROGRAM.
                    cpd       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       Go@@                ; YES. GO SAVE IT.
                    rts                           ; NO. RETURN.

Go@@                ldd       VAREND
                    subd      BASBEG
                    cpd       EESize
                    bls       Load@@
                    lda       #EETOSMAL
                    jmp       RPTERR

Load@@              ldx       EEStart             ; point to the start of the EEPROM.
                    ldy       #BASBEG
                    ldb       #4
                    stb       COUNT
Loop@@              ldd       ,y
                    subd      RAMStart
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    tba
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    iny:2
                    dec       COUNT
                    bne       Loop@@

                    ldd       ,y
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    tba
                    sta       ,x
                    jsr       DLY10MS

                    ldx       EEStart
                    ldy       BASBEG
Save@@              lda       ,y
                    sta       SSTART,x
                    jsr       DLY10MS
                    inx
                    iny
                    cpy       VAREND
                    bls       Save@@
                    rts                           ; RETURN.

;*******************************************************************************

CELOAD              proc
                    ldx       EEStart             ; point to the start of the program storage EEPROM.
                    ldy       #BASBEG             ; point to the start of the program pointer storage area.
                    ldb       #4                  ; number of words to move.
                    stb       COUNT               ; save the count.
Loop@@              ldd       ,x                  ; get the offset that was saved.
                    addd      RAMStart            ; add the starting address of the RAM to it.
                    std       ,y                  ; save the resulting pointer
                    inx:2                         ; point to the next offset.
                    iny:2                         ; point to the next pointer in RAM
                    dec       COUNT               ; have we gotten all the pointers yet?
                    bne       Loop@@              ; no. keep going.

                    ldd       ,x                  ; yes. get the high line number.
                    std       ,y                  ; save it in RAM.

          ; now load the actual program from EEPROM

                    ldx       EEStart             ; point to the start of the EEPROM
                    ldy       BASBEG              ; point to the start of the BASIC program buffer.
?CELOAD             lda       SSTART,x            ; get a byte of the program.
                    sta       ,y                  ; put it in the program buffer.
                    inx                           ; point to the next program byte
                    iny                           ; point to the next buffer location.
                    cpy       VAREND              ; have we finished loading the program.
                    bls       ?CELOAD             ; no. keep loading.
                    sty       STRASTG             ; yes. initialize the array storage area.
                    rts                           ; RETURN.

;*******************************************************************************

CLLIST              proc
                    lda       #$01                ; USE DEVICE #1 FOR HARD COPY LISTING.
                    sta       DEVNUM
                    jsr       CLIST               ; GO DO A STANDARD LIST COMMAND.
                    clr       DEVNUM
                    rts                           ; RETURN.

;*******************************************************************************

CAUTOST             proc                          ; SET AUTO START MODE FOR BASIC PROGRAM.
                    lda       #$55                ; GET FLAG.
?SaveAutoFlag       ldx       EEStart
                    sta       AUTOSTF,x           ; PROGRAM IT INTO THE EEPROM
                    jsr       DLY10MS             ; WAIT WHILE IT PROGRAMS.
                    rts                           ; RETURN.

;*******************************************************************************

CNOAUTO             proc
                    lda       #$FF
                    bra       ?SaveAutoFlag

;*******************************************************************************

AUTOLOAD            proc
                    ldx       EEStart
                    ldd       EEStart
                    addd      #SSTART
                    std       BASBEG
                    ldd       EEStart
                    addd      SBASEND,x
                    addd      #SSTART
                    std       BASEND

                    ldd       SVAREND,x
                    subd      SVARBEG,x
                    addd      RAMStart
                    std       VAREND
                    ldd       RAMStart
                    std       VARBEGIN
                    xgdy
                    ldd       EEStart
                    addd      SVARBEG,x
                    xgdx
                    bra       ?CELOAD

;*******************************************************************************

CFREE               proc
                    jsr       NL2
                    ldd       VARMEND
                    subd      STRASTG
                    jsr       OUTDECI
                    jsr       NL
                    rts

;*******************************************************************************

CDUMP               proc
                    rts

#ifdef ;------------------------------------------------------------------------
                    jsr       NL2                 ; PRINT TWO BLANK LINES.
                    clr       DNAME+2             ; ZERO THE LAST BYTE OF THE VARIABLE NAME 'ARRAY'
                    ldx       VARBEGIN            ; POINT TO THE START OF THE VARIABLE TABLE.
Loop@@              lda       ,x                  ; GET AN ENTRY. IS IT THE END OF THE TABLE?
                    bne       Go@@                ; YES. WE'RE DONE.
                    rts

Go@@                lda       1,x                 ; NO. GET THE FIRST CHARACTER OF THE NAME.
                    sta       DNAME
                    lda       2,x
                    sta       DNAME+1
                    ldx       #DNAME
                    jsr       PL
                    lda       ,x                  ; GET THE VARIABLE TOKEN.
                    cmpa      #IVARTOK            ; IS IT AN INTEGER?
                    beq       CDUMP9              ; YES. DUMP ITS VALUE.
                    cmpa      #IAVARTOK           ; NO. IS IT AN INTEGER ARRAY?
                    bne       CDUMP99             ; NO.
                    ldd       3,x                 ; YES. GET THE POINTER TO THE ARRAY STORAGE. HAS IT BEEN DIMENSIONED?
                    bne       CDUMP5              ; YES. GO PRINT ALL THE VALUES.
                    ldx       #UNDIM
                    jsr       PL
CDUMP6              ldb       #5
                    abx
                    bra       Loop@@

CDUMP5              pshx                          ; SAVE THE POINTER TO THE VARIABLE TABLE.
                    xgdx                          ; POINT TO THE ARRAY STORAGE AREA.
                    ldd       ,x                  ; GET THE MAXIMUM SUBSCRIPT.
                    std       SUBMAX
                    clrd
                    std       SUBCNT
CDUMP77             lda       #'('
                    jsr       OUTBYTE
                    ldd       SUBCNT
                    jsr       OUTDECI
                    ldx       #CPEQ
                    jsr       PL
                    inx:2
                    ldd       ,x
                    jsr       OUTDECI
                    jsr       NL
                    ldd       SUBCNT
                    incd
                    cmpd      SUBMAX
                    bhi       CDUMP88
                    std       SUBCNT
                    ldx       #DNAME
                    jsr       PL
                    bra       CDUMP77

CDUMP88             pulx
                    bra       CDUMP6

CDUMP9              lda       #'='
                    jsr       OUTBYTE
                    ldd       3,x
                    jsr       OUTDECI
                    jsr       NL
                    bra       CDUMP6

UNDIM               fcs       '=[?]'
CPEQ                fcs       ')='

#endif ;------------------------------------------------------------------------
;                   include   'runtime1.mod'
                    title     RUNTIME1

;*******************************************************************************

RREM                proc                          ; NON-EXECUTABLE STATEMENT JUST SKIP IT.
RDATA               proc
                    ldb       ,y                  ; GET LENGTH OF REMARK OR DATA LINE.
                    aby                           ; POINT TO THE EOLTOK.
                    rts                           ; RETURN.

;*******************************************************************************

RLET                proc
                    lda       ,y                  ; GET VARIABLE FLAG.
                    bsr       RVARPTR             ; GET POINTER TO ASIGNMENT VARIABLE.
                    pshd                          ; SAVE POINTER TO VARIABLE.
                    iny                           ; PUT IP PAST THE "=" TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET VALUE INTO D.
                    pulx                          ; POINT TO THE DICTIONARY ENTRY.
                    std       ,x                  ; STORE VALUE.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

;*******************************************************************************

RVARPTR             proc
                    lda       ,y                  ; GET VARIABLE FLAG.
                    bita      #$02                ; IS IT A STRING VARIABLE?
                    bne       StringPtr@@         ; YES. GO GET POINTER FOR A STRING DESCRIPTOR.
                    bita      #$10                ; IS IT A NUMERIC ARRAY VARIABLE?
                    bne       Subscript@@         ; YES. GO CALCULATE THE SUBSCRIPT.
DictOffset@@        ldd       1,y                 ; GET THE OFFSET TO THE DICTIONARY ENTRY.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE DCTIONARY.
                    addd      #3                  ; MAKE POINTER POINT TO THE ACTUAL STORAGE LOCATION
                    pshb                          ; SAVE B.
                    ldb       #3                  ; POINT TO THE FIRST ELEMENT PAST THE VARIABLE.
                    aby
                    pulb                          ; RESTORE B.
                    rts

Subscript@@         jsr       CALCSUB             ; GO GET BASE ADDR & EVALUATE SUBSCRIPT EXPRESSION.
                    pshx                          ; PUSH BASE ADDRESS ONTO STACK.
                    tsx                           ; POINT TO IT.
                    lsld                          ; MULT THE SUBSCRIPT BY THE # OF BYTES/ELEMENT.
GetAddr@@           addd      ,x                  ; GET ADDRESS OF ELEMENT.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

StringPtr@@         bita      #$10                ; IS IT A STRING ARRAY?
                    beq       DictOffset@@        ; NO. JUST GO GET POINTER TO DESCRIPTOR.
                    jsr       CALCSUB             ; GET BASE ADDR. & CALC SUBSCRIPT.
                    pshx                          ; SAVE THE BASE ADDRESS.
                    pshd                          ; SAVE THE SUBSCRIPT VALUE.
                    tsx                           ; POINT TO THE VALUES.
                    lsld                          ; MULT BY 2.
                    addd      ,x                  ; MULT BY 3.
                    ins:2                         ; GET RID OF SUBSCRIPT VALUE.
                    tsx                           ; POINT TO BASE ADDRESS.
                    bra       GetAddr@@

;*******************************************************************************

RGOTO               proc
                    tst       IMMED               ; DID WE ENTER HERE FROM THE IMMEDIATE MODE?
                    beq       Go@@                ; NO. JUST GO DO A GOTO.
                    ldd       BASEND              ; YES. SET ADRNXLIN TO END OF PROGRAM SO THE
                    std       ADRNXLIN            ; SEARCH STARTS AT THE FIRST LINE.
Go@@                ldx       ADRNXLIN            ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; IS THIS THE LAST LINE OF THE PROGRAM?
                    bne       SearchNextLine@@    ; NO. SEARCH STARTING AT THE NEXT LINE.
RGOTO3              ldx       BASBEG              ; YES. POINT TO THE START OF THE BASIC PROGRAM.
                    bra       Loop@@

SearchNextLine@@    ldd       ,x                  ; GET THE NEXT LINE NUMBER IN THE PGM.
                    cpd       1,y                 ; IS IT > THE LINE NUMBER WE ARE TO "GOTO"?
                    bhi       RGOTO3              ; YES. START THE SEARCH AT THE BEGINING.
Loop@@              ldd       ,x                  ; GET THE NEXT LINE NUMBER INTO D.
                    cpd       1,y                 ; IS THIS THE CORRECT LINE?
                    beq       NewLine@@           ; YES. "GOTO" THE NEW LINE.
                    blo       GetLen@@            ; NO. IS IT LESS THAN THE "TARGET LINE"?
Fail@@              lda       #LNFERR             ; NO. THE LINE MUST NOT EXIST.
                    jmp       RPTRERR             ; REPORT THE ERROR & RETURN TO MAIN LOOP.

GetLen@@            ldb       2,x                 ; GET THE LENGTH OF THIS LINE.
                    abx                           ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; DID WE HIT THE END OF THE PROGRAM?
                    beq       Fail@@              ; YES. THE LINE DOESN'T EXIST.
                    bra       Loop@@              ; NO. GO SEE IF THIS IS THE CORRECT LINE.

NewLine@@           xgdx                          ; MAKE IT THE NEW IP.
                    xgdy
                    tst       IMMED
                    beq       RGOTO8
                    clr       IMMED
Run@@               jmp       CRUN1

RGOTO8              ins:2
                    bra       Run@@

;*******************************************************************************

RGOSUB              proc
                    pshy                          ; SAVE THE I.P. TO THE LINE NUMBER.
                    tst       IMMED               ; DID WE GET HERE FROM THE IMMEDIATE MODE?
                    beq       Go@@                ; NO. GO DO A NORMAL GOSUB.
                    ldy       BASEND              ; YES. MAKE RETURN POINT TO THE LAST EOL TOKEN
                    dey                           ; IN THE PROGRAM.
                    bra       ?RGOSUB             ; GO PUT IT ON THE ARGUMENT STACK.

Go@@                ldb       #3                  ; BYPASS THE LINE NUMBER.
                    aby
?RGOSUB             jsr       RSKIPSPC            ; SKIP SPACES AFTER THE LINE NUMBER.
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    dex:2                         ; POINT TO THE NEXT ENTRY ON THE STACK.
                    cpx       EGOSTK              ; OUT OF STACK SPACE?
                    bhs       Done@@              ; NO. GO PUSH THE "RETURN ADDRESS" ON THE STACK.
                    lda       #GOSOVERR           ; YES. GET THE ERRCODE.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Done@@              stx       GOSTACK             ; SAVE THE "GOSUB" STACK POINTER.
                    sty       ,x                  ; PUT THE RETURN ADDRESS ON THE STACK.
                    puly                          ; GET THE POINTER TO THE LINE NUMBER.
                    jmp       RGOTO               ; GO DO A "GOTO".

;*******************************************************************************

RRETURN             proc
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    cpx       STGOSTK             ; IS THERE A RETURN ADDRESS ON THE GOSUB STACK?
                    bne       Go@@                ; YES. GO RETURN.
                    lda       #RWOGERR            ; NO. RETURN W/O GOSUB ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

Go@@                ldy       ,x                  ; GET THE RETURN ADDRESS IN THE IP.
                    inx:2                         ; REMOVE THE ADDRESS FROM THE STACK.
                    stx       GOSTACK             ; SAVE THE STACK POINTER.
                    rts                           ; BACK TO THE MAIN INTERPRET LOOP.

;*******************************************************************************

RSTOP               proc
                    ldx       #STOPSTR
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    sty       IPSAVE
                    bra       ?REND

;*******************************************************************************

REND                proc
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
?REND               ldd       #0
                    std       CURLINE
                    jmp       MainLoop

STOPSTR             fcs       LF,CR,'STOPPED AT LINE # '

;*******************************************************************************

RWHILE              proc
                    ldx       WHSTACK             ; GET THE WHILE STACK POINTER.
                    dex:2                         ; POINT TO THE NEXT STACK LOCATION.
                    cpx       EWHSTK              ; ARE WE AT THE END OF THE STACK?
                    bhs       Go@@                ; NO. GO STACK IT.
                    lda       #WHSOVERR           ; YES. WHILE STACK OVER FLOW.
                    jmp       RPTRERR             ; REPORT THE ERROR.

Go@@                stx       WHSTACK             ; SAVE THE WHILE STACK POINTER.
                    sty       ,x                  ; PUT IT ON THE STACK.
                    ldb       #$01                ; GET THE WHILE COUNT INTO B. (FOR NESTED WHILE'S)
Loop@@
          #ifndef BUGFIX
                    pshb                          ; (tonyp: Bug in case of BNE fall-through)
          #endif
                    ldy       ADRNXLIN            ; GET THE ADDRESS OF THE NEXT LINE.
                    bne       Save@@
                    rts
Save@@
          #ifdef BUGFIX
                    pshb
          #endif
                    pshy                          ; SAVE THE IP.
                    cpy       BASEND              ; ARE WE AT THE END OF THE PROGRAM?
                    beq       REND                ; YES. DO AN END.
                    ldx       ADRNXLIN            ; NO. GET THE ADDRESS OF THE NEXT LINE IN X.
                    ldb       2,x                 ; GET THE LENGTH OF THIS LINE.
                    abx                           ; POINT TO THE START OF THE NEXT LINE.
                    stx       ADRNXLIN            ; SAVE IT.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER & LINE LENGTH.
                    aby
                    jsr       RSKIPSPC            ; SKIP ANY SPACES.
                    lda       ,y                  ; GET THE KEYWORD TOKEN.
                    puly                          ; RESTORE THE IP.
                    pulb                          ; GET THE NESTED WHILE COUNT.
                    cmpa      #WHILETOK           ; IS IT ANOTHER WHILE?
                    bne       Cont@@              ; NO. GO CHECK FOR ENDWH.
                    incb                          ; YES. UP THE NESTED WHILE COUNT.
Cont@@              cmpa      #ENDWHTOK           ; IS IT THE END WHILE STATEMENT?
                    bne       Loop@@              ; NO. GO LOOK AT THE NEXT LINE.
                    decb                          ; YES. IS IT THE CORRECT 'ENDWH'?
                    bne       Loop@@              ; NO. LOOK FOR ANOTHER ONE.
                    jmp       RGOTO8              ; BACK TO INTERPRET LOOP.

;*******************************************************************************

RENDWH              proc
                    ldx       WHSTACK             ; GET THE WHILE STACK POINTER.
                    cpx       STWHSTK             ; HAS A WHILE STATEMENT BEEN EXECUTED?
                    bne       Go@@                ; YES. GO GET THE ADDRESS OF THE WHILE STATEMENT.
                    lda       #ENDWHERR           ; NO. GET ENDWHILE ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.
Go@@                pshy                          ; SAVE THE IP IN CASE THE WHILE TEST FAILS.
                    ldy       ,x                  ; GET THE IP POINTER TO THE WHILE EXPRESSION.
                    jsr       DONEXP              ; YES. GO EVALUATE A NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET RESULT OFF NUMERIC STACK. IS IT TRUE?
                    bne       Exec@@              ; YES. GO EXECUTE CODE BETWEEN WHILE & ENDWH.
                    puly                          ; NO. GET THE ADDRESS OF THE NEXT LINE/STATEMENT.
                    ldx       WHSTACK             ; GET WHILE STACK POINTER.
                    inx:2                         ; TAKE ADDRESS OFF OF WHILE STACK.
                    stx       WHSTACK             ; SAVE STACK POINTER.
                    bra       Done@@              ; GO TO INTERPRET LOOP.
Exec@@              ins:2                         ; REMOVE POINTER TO STATEMENT AFTER "ENDWH" FROM STACK.
Done@@              rts                           ; GO EXECUTE LINES TILL "ENDWH".

;*******************************************************************************

RON                 proc
                    jsr       DONEXP              ; GO EVALUATE THE EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER EXPRESSION.
                    lda       ,y                  ; GET EITHER "GOTO" OR "GOSUB" TOKEN.
                    psha                          ; SAVE IT.
                    iny                           ; POINT TO NEXT TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    ldx       NUMSTACK            ; POINT TO THE OPERAND STACK.
                    ldd       ,x                  ; GET EXPRESSION VALUE.
                    bpl       Go@@                ; IS IT NEGATIVE?
                    bne       Go@@                ; OR ZERO?
Fail@@              lda       #ONARGERR           ; YES. REPORT ERROR.
                    jmp       RPTRERR

Go@@                ldd       ,x                  ; GET THE EXPRESSION VALUE.
                    decd                          ; SUBTRACT 1. HAVE WE FOUND THE LINE NUMBER?
                    beq       Exec@@              ; YES. GO DO "GOTO" OR "GOSUB".
                    std       ,x                  ; NO. SAVE REMAINDER.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER VALUE.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES PAST THE LINE NUMBER.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #EOLTOK             ; HAVE WE HIT THE END OF THE LINE?
                    beq       Fail@@              ; YES. ERROR.
                    iny                           ; NO. MUST BE A COMMA. BYPASS IT.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    bra       Go@@                ; GO SEE IF THE NEXT LINE NUMBER IS THE ONE.

Exec@@              jsr       PULNUM              ; GET RID OF ARGUMENT.
                    pula                          ; GET "GO" TOKEN.
                    cmpa      #GOTOTOK            ; IS IT A "GOTO" TOKEN?
                    bne       DoGosub@@           ; NO. MUST BE A "GOSUB"
                    jmp       RGOTO               ; GO DO A "GOTO".

DoGosub@@           pshy                          ; SAVE THE POINTER TO THE LINE NUMBER.
ToEol@@             ldb       #3                  ; POINT PAST THE LINE NUMBER.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER LINE NUMBER.
                    lda       ,y                  ; GET NEXT TERMINATOR CHARACTER.
                    cmpa      #EOLTOK             ; HIT THE END OF THE LINE YET?
                    beq       Gosub@@             ; YES. GO DO THE GOSUB.
                    cmpa      #MEOLTOK            ; NO. HIT THE LOGICAL END OF THE LINE YET?
                    beq       Gosub@@             ; YES. GO DO THE GOSUB.
                    iny                           ; NO. MUST BE A COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    bra       ToEol@@             ; GO FIND THE END OF THE LINE.

Gosub@@             jmp       ?RGOSUB             ; GO DO A "GOSUB".

;*******************************************************************************

RPOKE               proc
                    iny                           ; PASS UP THE OPEN PAREN.
                    jsr       RSKIPSPC            ; PASS UP ANY SPACES.
                    jsr       DONEXP              ; GO EVALUATE THE ADDRESS EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP ANY SPACES.
                    iny                           ; SKIP THE COMMA.
                    jsr       RSKIPSPC            ; SKIP ANY SPACES.
                    jsr       DONEXP              ; GET THE VALUE TO PUT INTO MEMORY.
                    iny                           ; PASS UP THE CLOSING PAREN.
                    jsr       PULNUM              ; GET THE MEMORY VALUE.
                    xgdx                          ; SAVE IT.
                    jsr       PULNUM              ; GET THE ADDRESS.
                    xgdx                          ; PUT ADDRESS INTO X & MEM VALUE INTO D.
                    stb       ,x                  ; PUT VALUE INTO MEMORY.
                    rts                           ; BACK TO THE INTERPRET LOOP.

;*******************************************************************************

RPORTA              proc
                    ldb       #PORTAIO
?RPORT              ldx       IOBaseV             ; GET ADDRESS OF PORTA I/O REGISTER.
                    abx
                    pshx                          ; SAVE POINTER TO VARIABLE.
                    iny                           ; PUT IP PAST THE "=" TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET VALUE INTO D.
                    tsta                          ; IS THE VALUE <0 AND >255?
                    beq       Done@@              ; NO. GO PUT THE VALUE IN THE PORT.
                    lda       #PRTASERR           ; YES. ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.
Done@@              pulx                          ; POINT TO THE DICTIONARY ENTRY.
                    stb       ,x                  ; STORE VALUE.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

;*******************************************************************************

RPORTB              proc
                    ldb       #PORTBIO            ; GET ADDRESS OF PORTB I/O REGISTER.
                    bra       ?RPORT              ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTC              proc
                    ldb       #PORTCIO            ; GET ADDRESS OF PORTC I/O REGISTER.
                    bra       ?RPORT              ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTD              proc
                    ldb       #PORTDIO            ; GET ADDRESS OF PORTD I/O REGISTER.
                    bra       ?RPORT              ; GO DO AN ASIGNMENT.
;                   include   'runtime2.mod'
                    title     RUNTIME2

;*******************************************************************************

RTRON               proc
                    lda       #$FF                ; SET FLAG TO TURN TRACE MODE ON.
                    sta       TRFLAG              ; PUT IT IN THE FLAG BYTE.
                    rts                           ; BACK TO THE INTERPRET LOOP.

;*******************************************************************************

RTROFF              proc
                    clr       TRFLAG              ; TURN THE TRACE MODE OFF.
                    rts                           ; BACK TO THE INTERPRET LOOP.

;*******************************************************************************

RSLEEP              proc
                    sei                           ; DON'T ALLOW AN INTERRUPT TO BRING US OUT OF THE SLEEP MODE.
                    tpa                           ; GET THE CONDITION CODE REGISTER.
                    anda      #$7F                ; CLEAR THE STOP BIT
                    tap                           ; TRANSFER THE RESULT BACK TO THE CCR.
                    stop                          ; HALT THE CPU.
                    tpa                           ; ON EXIT FROM THE STOP MODE, GET THE CCR.
                    ora       #$80                ; DISABLE THE STOP INSTRUCTION.
                    tap                           ; TRANSFER THE RESULT BACK TO THE CCR.
                    cli                           ; ALLOW INTERRUPTS.
                    rts                           ; RETURN TO WHAT WE WERE DOING.

;*******************************************************************************

RPRINT              proc
                    jsr       CHCKDEV             ; GO CHECK FOR ALTERNATE OUTPUT DEVICE.
                    lda       ,y                  ; GET FIRST TOKEN.
                    cmpa      #EOLTOK             ; IS IT AN EOL TOKEN?
                    beq       NewLine@@           ; YES. JUST PRINT A CR/LF.
                    cmpa      #MEOLTOK            ; IS IT A MID EOL TOKEN?
                    bne       Loop@@              ; NO. GO PRINT A STRING OR NUMBER.
NewLine@@           jsr       NL                  ; YES. JUST PRINT A CR/LF.
                    clr       DEVNUM              ; GO BACK TO DEVICE #0.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

Loop@@              cmpa      #SCONTOK            ; IS IT A STRING CONSTANT?
                    bne       Func?@@             ; NO. GO CHECK FOR A "PRINT FUNCTION".
                    pshy
                    ldb       #2                  ; COMPENSATE FOR CONSTANT & LENGTH BYTE.
                    addb      1,y                 ; ADD IN LENGTH BYTE.
                    aby                           ; POINT BEYOND PROMPT.
                    pulx                          ; GET POINTER INTO X.
                    inx                           ; POINT TO LENGTH BYTE.
                    ldb       ,x                  ; GET IT.
                    subb      #2                  ; SUBTRACT OUT THE DELIMETER COUNT.
                    inx:2                         ; POINT TO STRING.
                    jsr       OUTSTR              ; GO PRINT THE STRING.
                    bra       NextExpr@@          ; GO DO NEXT EXPRESSION.

Func?@@             cmpa      #FUNCTFLG           ; IS IT A FUNCTION?
                    bne       NumExpr@@           ; NO. GO EVALUATE A NUMERIC EXPRESSION.
                    lda       1,y                 ; GET THE FUNCTION TYPE.
                    cmpa      #TABTOK             ; IS IT A TAB?
                    bne       Chr?@@              ; NO GO CHECK FOR "CHR$".
                    jsr       RTAB                ; GO DO TAB.
                    bra       NextExpr@@          ; GO SEE IF THERE'S MORE TO PRINT.

Chr?@@              cmpa      #CHRTOK             ; IS IT THE CHR$ FUNCTION.
                    bne       Hex?@@              ; NO. GO CHECK FOR HEX().
                    jsr       RCHRS               ; YES. GO DO CHR$.
                    bra       NextExpr@@          ; GO SEE IF THERE'S MORE TO PRINT.

Hex?@@              cmpa      #HEXTOK             ; IS IT THE HEX() FUNCTION?
                    bne       NumExpr@@           ; NO. GO DO A NUMERIC EXPRESSION.
                    jsr       RHEX                ; YES. GO PRINT THE NUMBER AS HEX.
                    bra       NextExpr@@          ; GO SEE IF THERE'S MORE TO PRINT.

NumExpr@@           cmpa      #HEX2TOK            ; IS IT THE HEX2() FUNCTION?
                    bne       DoNumExpr@@         ; NO. GO DO A NUMERIC EXPRESSION.
                    jsr       RHEX2               ; YES GO PRINT A NUMBER >=255 AS 2 HEX BYTES.
                    bra       NextExpr@@          ; GO SEE IF THERE'S MORE TO PRINT.

DoNumExpr@@         jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE NUMERIC STACK.
                    jsr       OUTDECI             ; PRINT IT.
                    lda       #SPC                ; PUT A TRAILING SPACE AFTER ALL NUMBERS.
                    jsr       OUTBYTE             ; PRINT IT.
NextExpr@@          jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET SEPERATOR CHARACTER.
                    cmpa      #COMMATOK           ; IS IT A COMMA?
                    beq       PastComma@@         ; NO.
                    cmpa      #SEMITOK            ; IS IT A SEMICOLON?
                    bne       Eol@@               ; NO. MUST BE AN EOLTOK.
                    iny                           ; DO NOTHING BUT BUMP THE IP.
                    bra       Eol?@@              ; GO CHECK FOR EOL AFTER COMMA OR SEMICOLON.

PastComma@@         iny                           ; BUMP IP PAST THE COMMATOK.
                    ldb       PRINTPOS            ; YES. "TAB" TO NEXT PRINT FIELD.
                    andb      #$07                ; MASK OFF ALL BUT THE FIELD WIDTH.
                    negb                          ; MAKE IT NEGATIVE.
                    addb      #8                  ; ADD IN THE FIELD WIDTH. ARE WE ON A FIELD BOUND?
                    beq       Eol?@@              ; YES. GO CHECK FOR AN EOL.
                    lda       #SPC                ; NO. GET A SPACE & PRINT TILL WE GET THERE.
SpaceLoop@@         jsr       OUTBYTE             ; PRINT A SPACE.
                    decb                          ; DECREMENT THE COUNT. ARE WE DONE?
                    bne       SpaceLoop@@         ; NO. KEEP GOING.
Eol?@@              jsr       RSKIPSPC            ; SKIP ANY SPACES.
                    lda       ,y                  ; GET THE NEXT TOKEN IN THE LINE.
                    cmpa      #EOLTOK             ; IS IT AN EOL TOKEN?
                    beq       Done@@              ; YES. DONT DO A CR/LF AFTER A COMMA OR SEMI.
                    cmpa      #MEOLTOK            ; NO. IS IT A MID EOL?
                    beq       Done@@              ; SAME AS BEFORE.
                    jmp       Loop@@              ; IF NEITHER, GO PRINT THE NEXT EXPRESSION.

Eol@@               jsr       NL                  ; DO A CR/LF IF EOL OR MIDEOL FOLLOWS EXPRESSION.
Done@@              clr       DEVNUM              ; GO BACK TO DEVICE #0.
                    rts                           ; GO DO NEXT LINE.

;*******************************************************************************

RTAB                proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       Loop@@              ; YES. GO DO TAB.
                    lda       #TABARGER           ; NO. ERROR.
?RTAB               jmp       RPTRERR             ; REPORT ERROR.

Loop@@              cmpb      PRINTPOS            ; ARE WE ALREADY PAST THE "TAB" POSITION?
                    bls       Done@@              ; YES. DONE.
                    lda       #SPC                ; GET A SPACE.
                    jsr       OUTBYTE             ; PRINT IT.
                    bra       Loop@@
Done@@              rts                           ; RETURN.

;*******************************************************************************

RCHRS               proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       Go@@                ; YES. GO DO TAB.
                    lda       #CHRARGER           ; NO. ERROR.
                    bra       ?RTAB               ; REPORT ERROR.

Go@@                tba                           ; PUT BYTE INTO A
                    jmp       OUTBYTE             ; PRINT THE BYTE & RETURN.

;*******************************************************************************

RHEX2               proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       ?RHEX               ; YES. GO PRINT 2 HEX CHARACTERS & RETURN.
                    lda       #HEX2AERR           ; NO. ARG. MUST BE >=0 & <=255.
                    bra       ?RTAB               ; GO REPORT ERROR.

;*******************************************************************************

RHEX                proc
                    bsr       PFUNCOM             ; GO DO COMMON CODE FOR PRINT FUNCTIONS
                    bsr       PRNT2HEX            ; GO PRINT 2 HEX CHARACTERS.
?RHEX               tba                           ; PUT LOWER BYTE IN A.
;                   bra       PRNT2HEX

;*******************************************************************************

PRNT2HEX            proc
                    psha                          ; SAVE THE CHARACTER.
                    bsr       Msb@@               ; PRINT THE LEFT HEX NIBBLE.
                    pula                          ; GET BYTE BACK.
                    bra       Lsb@@               ; PRINT RIGHT NIBBLE & RETURN.

Msb@@               lsra:4                        ; GET UPPER NIBBLE INTO LOWER ONE.
Lsb@@               anda      #$0F                ; MASK OFF UPPER NIBBLE.
                    adda      #'0'                ; MAKE IT A HEX NUMBER.
                    cmpa      #'9'                ; IS IT?
                    bls       Print@@             ; YES. PRINT IT.
                    adda      #$07                ; NO. MAKE IT A HEX LETTER.
Print@@             jmp       OUTBYTE             ; PRINT IT & RETURN.

;*******************************************************************************

PFUNCOM             proc
                    ldb       #3                  ; POINT PAST FUNCTION FLAG, FUNCTION TOKEN, &
                    aby                           ; OPEN PAREN.
                    jsr       DONEXP              ; GO GET POSITION TO TAB TO.
                    iny                           ; BUMP IP PAST CLOSING PAREN.
                    jsr       PULNUM              ; GET OPERAND OFF STACK.
                    tsta                          ; CHECK THAT OPERAND IS >0 & <=255 FOR FUNCTIONS
                                                  ; THAT REQUIRE IT.
                    rts                           ; RETURN.

;*******************************************************************************

RDIM                proc
                    lda       ,y                  ; GET VARIABLE FLAG/TYPE.
                    bita      #$10                ; IS IT A SUBSCRIPTED VARIABLE?
                    bne       Go@@                ; YES. GO DIMENSION IT.
                    lda       #NOSUBERR           ; NO. GET ERROR.
Fail@@              jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                ldd       1,y                 ; GET THE OFFSET INTO THE DICTIONARY.
                    addd      VARBEGIN            ; ADD IN THE START OF THE DICTIONARY.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    ldd       3,x                 ; GET THE POINTER TO THE STORAGE. BEEN DIMENSIONED?
                    beq       Dim@@               ; NO. GO DIMENSION IT.
                    lda       #REDIMERR           ; YES. ERROR.
                    bra       Fail@@

Dim@@               pshx                          ; SAVE THE POINTER TO THE DICTIONARY.
                    ldb       #4                  ; POINT TO 1ST TOKEN IN EXPRESSION.
                    aby
                    jsr       DONEXP              ; EVALUATE THE SUBSCRIPT.
                    iny                           ; PASS UP THE CLOSING PAREN.
                    pulx                          ; RESTORE POINTER TO DICTIONARY.
                    ldd       STRASTG             ; GET THE DYNAMIC MEMORY POOL POINTER.
                    std       3,x                 ; PUT THE POINTER IN THE DICTIONARY ENTRY.
                    addd      #2                  ; UP THE POINTER.
                    std       STRASTG             ; SAVE NEW POINTER FOR NOW.
                    jsr       PULNUM              ; GET SUBSCRIPT OFF OF NUMERIC STACK.
                    bpl       GoodSub@@           ; ONLY POSITIVE SUBSCRIPTS ALLOWED.
                    lda       #NEGSUBER           ; NEGATIVE NUMBER.
                    bra       Fail2@@             ; REPORT ERROR.

GoodSub@@           pshx
                    ldx       3,x                 ; GET POINTER TO STORAGE.
                    std       ,x                  ; PUT MAX SUBSCRIPT IN POOL STORAGE.
                    incd                          ; COMPENSATE FOR "0" SUBSCRIPT.
                    pulx                          ; RESTORE POINTER TO DICTIONARY ENTRY.
                    lsld                          ; MULT. BY 2 (2 BYTES/INTEGER).
                    addd      STRASTG             ; ADD IN CURRENT POINTER TO POOL.
                    cpd       STRASTG             ; WAS THE SUBSCRIPT SO BIG WE WRAPPED AROUND?
                    bls       OutOfMem@@          ; YES. ERROR.
                    cpd       VARMEND             ; DO WE HAVE ENOUGH MEMORY?
                    bls       SavePtr@@           ; YES.
OutOfMem@@          lda       #OMEMERR            ; NO. ERROR.
Fail2@@             jmp       RPTRERR             ; GO REPORT THE ERROR.

SavePtr@@           std       STRASTG             ; SAVE POINTER.
                    ldx       3,x                 ; POINT TO START OF STORAGE.
                    inx:2                         ; POINT PAST THE SUBSCRIPT LIMIT.
Clear@@             clr       ,x                  ; CLEAR THE STORAGE.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       STRASTG             ; ARE WE DONE?
                    bne       Clear@@             ; NO. KEEP GOING.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET THE NEXT CHARACTER.
                    cmpa      #EOLTOK             ; ARE WE AT THE END OF THE LINE.
                    beq       Done@@              ; YES.
                    iny                           ; BUMP IP PAST THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    bra       RDIM                ; DO DIMENSION THE NEXT VARIABLE.

Done@@              rts                           ; BACK TO MAIN INTERPRET LOOP.
;                   include   'runtime3.mod'
                    title     RUNTIME3

;*******************************************************************************

RFOR                proc
                    ldd       FORSTACK            ; GET FOR STACK POINTER.
                    subd      #10                 ; ALLOCATE NEW FOR-NEXT DESCRIPTOR BLOCK.
                    cpd       EFORSTK             ; HAVE WE RUN OUT OF FOR-NEXT STACK SPACE?
                    bhs       Go@@                ; NO. CONTINUE.
                    lda       #FORNXERR           ; YES. ERROR.
                    jmp       RPTRERR             ; REPORT ERROR.

Go@@                std       FORSTACK            ; SAVE NEW STACK POINTER.
                    pshy                          ; SAVE IP ON STACK.
                    jsr       RVARPTR             ; GET POINTER TO ASIGNMENT VARIABLE.
                    puly                          ; RESTORE IP.
                    ldx       FORSTACK            ; GET FOR STACK POINTER.
                    std       ,x                  ; PUT POINTER TO CONTROL VARIABLE IN STACK.
                    ldd       CURLINE             ; GET CURRENT LINE NUMBER.
                    std       8,x                 ; SAVE CURRENT LINE NUMBER IN STACK.
                    jsr       RLET                ; GO DO ASIGNMENT PART OF FOR.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; SKIP PAST "TO" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; CALCULATE THE TERMINATING LOOP VALUE.
                    jsr       PULNUM              ; GET NUMBER OFF OF THE STACK.
                    ldx       FORSTACK            ; GET STACK POINTER.
                    std       4,x                 ; PUT VALUE IN STACK BLOCK.
                    ldd       #1                  ; ASSUME A "STEP" VALUE OF 1.
Save@@              std       2,x                 ; PUT IT IN THE STACK.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #STEPTOK            ; IS THE STEP CLAUSE PRESENT?
                    beq       Step@@              ; YES. GO GET THE "STEP" VALUE.
                    sty       6,x                 ; PUT TERMINATING CHARACTER OF "FOR" STATEMENT ON.
                    rts                           ; EXECUTE NEXT STATEMENT.

Step@@              iny                           ; SKIP PAST THE "STEP" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; GO CALCULATE THE "STEP" VALUE.
                    jsr       PULNUM              ; GET VALUE OFF OPERAND STACK.
                    ldx       FORSTACK            ; GET POINTER TO FOR STACK.
                    bra       Save@@              ; GO PUT VALUE IN STACK.

;*******************************************************************************

RNEXT               proc
                    jsr       RVARPTR             ; GET POINTER TO LOOP INDEX VARIABLE.
                    ldx       FORSTACK            ; GET "FOR" STACK POINTER.
                    cpd       ,x                  ; IS THE LOOP VARIABLE THE SAME?
                    beq       Go@@                ; YES. CONTINUE.
                    lda       #MFRNXERR           ; NO. ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

Go@@                pshy                          ; SAVE IP.
                    ldy       ,x                  ; GET POINTER TO CONTROL VARIABLE.
                    ldd       ,y                  ; GET CONTROL VARIABLE VALUE.
                    addd      2,x                 ; ADD THE STEP VALUE TO IT.
          #ifdef BUGFIX
                    bvs       Cont@@              ; BugFix (2000.03.28 <tonyp@acm.org>). On overflow, exit
          #endif
                    std       ,y                  ; SAVE THE RESULT.
                    tst       2,x                 ; IS THE STEP VALUE NEGATIVE?
                    bmi       Done?@@             ; YES. GO DO TEST.
                    cpd       4,x                 ; NO. ARE WE DONE?
                    ble       Done@@              ; NO. GO DO THE LOOP AGAIN.
Cont@@              puly                          ; RESTORE THE CURRENT IP.
                    xgdx                          ; PUT "FOR - NEXT" STACK POINTER IN D.
                    addd      #10                 ; REMOVE DESCRIPTOR FROM STACK.
                    std       FORSTACK            ; SAVE NEW STACK VALUE.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER CONTROL VARIABLE.
                    rts                           ; DO THE STATEMENT AFTER THE NEXT.

Done?@@             cpd       4,x                 ; ARE WE DONE?
                    blt       Cont@@              ; YES. CONTINUE.
Done@@              puly                          ; CLEAN Y OFF OF STACK.
                    ldy       6,x                 ; GET NEW IP.
                    ldd       8,x                 ; GET LINE NUMBER OF FOR STATEMENT.
                    std       CURLINE             ; MAKE IT THE CURRENT LINE.
                    rts

;*******************************************************************************

RINPUT              proc
                    bsr       CHCKDEV             ; CHECK FOR ALTERNATE INPUT DEVICE.
                    lda       ,y                  ; GET A TOKEN.
                    cmpa      #SCONTOK            ; IS THERE A PROMPT TO PRINT?
                    bne       Loop@@              ; NO JUST GO GET THE DATA.
                    pshy                          ; YES. SAVE POINTER.
                    ldb       #2                  ; COMPENSATE FOR CONSTANT & LENGTH BYTE.
                    addb      1,y                 ; ADD IN LENGTH BYTE.
                    aby                           ; POINT BEYOND PROMPT.
                    pulx                          ; GET POINTER INTO X.
                    inx                           ; POINT TO LENGTH BYTE.
                    ldb       ,x                  ; GET IT.
                    subb      #2                  ; SUBTRACT OUT THE DELIMETER COUNT.
                    inx:2                         ; POINT TO STRING.
                    jsr       OUTSTR              ; GO PRINT THE STRING.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER COMMA.
                    bra       Prompt@@

Loop@@              jsr       NL
Prompt@@            ldx       #Msg@@              ; POINT TO PROMPT.
                    jsr       PL                  ; PRINT IT.
                    jsr       GETLINE             ; GET THE DATA IN THE INPUT BUFFER.
                    bsr       RINRDC
                    bcs       Loop@@
                    jsr       NL
                    clr       DEVNUM              ; SET DEVICE NUMBER BACK TO 0.
                    rts

Msg@@               fcs       '? '

;*******************************************************************************

CHCKDEV             proc
                    lda       ,y                  ; GET A TOKEN.
                    cmpa      #PNUMTOK            ; IS AN ALTERNATE DEVICE SPECIFYED?
                    beq       Go@@                ; YES. CONTINUE.
                    rts                           ; NO. RETURN.

Go@@                iny                           ; YES. PASS THE '#' TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; GO EVALUATE THE NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    bpl       Limit@@             ; NEGATIVE NUMBERS NOT ALLOWED.
Fail@@              lda       #ILLIOERR           ; REPORT THE ERROR.
                    jmp       RPTRERR

Limit@@             cpd       #7                  ; IS IT LARGER THAN 7?
                    bhi       Fail@@
                    stb       DEVNUM              ; MAKE IT THE NEW DEVICE NUMBER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    cmpa      #EOLTOK             ; IF THIS IS A PRINT STATEMENT, IS IT EOL?
                    beq       Done@@              ; YES. DON'T BUMP THE IP.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
Done@@              rts                           ; RETURN.

;*******************************************************************************

RINRDC              proc
                    jsr       SKIPSPCS
                    cmpa      #EOL
                    bne       Go@@
                    sec
                    rts

Go@@                bsr       INNUMD
                    jsr       RSKIPSPC
                    lda       ,y
                    cmpa      #EOLTOK
                    beq       Done@@
                    cmpa      #MEOLTOK
                    beq       Done@@
                    iny                           ; BUMP PAST THE COMMA.
                    jsr       RSKIPSPC
                    bra       RINRDC

Done@@              clc
                    rts

;*******************************************************************************

INNUMD              proc
                    cmpa      #'$'
                    bne       Decimal@@
                    jsr       INCIBP
                    jsr       GETHEX
                    bra       Go@@

Decimal@@           jsr       INDECI
Go@@                pshd
                    jsr       SKIPSPCS
                    cmpa      #COMMA
                    beq       Comma@@
                    cmpa      #EOL
                    beq       Eol@@
                    lda       #MCOMAERR
                    jmp       RPTRERR

Comma@@             jsr       INCIBP
Eol@@               jsr       RVARPTR
                    xgdx
                    puld
                    std       ,x
                    rts

;*******************************************************************************

OUTSTR              proc
                    tstb
                    beq       Done@@
Loop@@              lda       ,x
                    inx
                    jsr       OUTBYTE
                    decb
                    bne       Loop@@
Done@@              rts

;*******************************************************************************

INDECI              proc
                    jsr       GETCHR              ; GET A CHARACTER.
                    cmpa      #'-'                ; IS IT A NEGATIVE NUMBER?
                    bne       Plus@@              ; NO. GO GET POSITIVE NUMBER.
                    jsr       INCIBP              ; YES. BUMP INPUT BUFFER PAST IT.
                    jsr       GETDECI             ; GET THE NUMBER.
                    negd                          ; NEGATE IT.
                    rts                           ; RETURN.

Plus@@              jsr       GETDECI
                    rts

;*******************************************************************************

RREAD               proc
                    ldx       DATAPTR             ; GET POINTER TO DATA. IS IT POINTING TO DATA?
                    bne       Go@@                ; YES. CONTINUE TO READ DATA.
                    bsr       RRESTOR             ; NO. GO GET POINTER TO FIRST DATA STATEMENT.
                    ldx       DATAPTR             ; GET POINTER TO DATA.
Go@@                stx       IBUFPTR             ; PUT IT IN THE INPUT BUFFER POINTER.
                    jsr       RINRDC              ; GO USE INPUT/READ COMMON CODE.
                    bcs       ReadMore@@          ; IF CARRY SET, MORE DATA TO READ.
                    ldx       IBUFPTR             ; GET POINTER TO DATA LINE.
                    stx       DATAPTR             ; SAVE DATA POINTER FOR NEXT READ.
                    rts                           ; RETURN.

ReadMore@@          pshy                          ; SAVE Y.
                    ldy       IBUFPTR
                    iny:2
                    bsr       ?RRESTOR            ; GO FIND NEXT "DATA" STATEMENT.
                    puly                          ; RESTORE Y.
                    bra       RREAD               ; KEEP READING DATA.

;*******************************************************************************

RRESTOR             proc
                    pshy                          ; SAVE Y.
                    ldy       BASBEG              ; START SEARCH FOR "DATA" STATEMENTS AT THE BEGIN.
Loop@@              pshy                          ; SAVE POINTER TO THIS LINE.
                    ldb       2,y                 ; GET LINE LENGTH.
                    aby                           ; GET START OF NEXT LINE.
                    sty       DATAPTR             ; SAVE IN "DATAPTR".
                    puly                          ; RESTORE POINTER.
                    ldb       #3
                    aby                           ; POINT TO FIRST TOKEN IN LINE.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET THE KEYWORD.
                    cmpa      #DATATOK            ; IS IT A DATA LINE?
                    beq       Done@@              ; YES. GO SET UP POINTER.
                    ldy       DATAPTR             ; GET ADDRESS OF NEXT LINE.
Cont@@              cpy       BASEND              ; ARE WE AT THE END OF THE PROGRAM?
                    bne       Loop@@              ; NO. KEEP LOOKING.
                    lda       #ODRDERR            ; OUT OF DATA ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

Done@@              iny:2                         ; POINT PAST DATA TOKEN & THE DATA LENGTH.
                    sty       DATAPTR             ; SAVE POINTER TO DATA.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

?RRESTOR            pshy                          ; CALL TO COMPENSATE FOR PULL OF Y ON RETURN.
                    bra       Cont@@

;*******************************************************************************

RIF                 proc
                    jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; SKIP PAST "THEN" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THEN.
                    jsr       PULNUM              ; GET RESULT OF EXPRESSION FROM OPERAND STACK.
                    beq       DoElse@@            ; NOT TRUE. SEE IF ELSE CLAUSE PRESENT.
Goto@@              jmp       RGOTO               ; RESULT WAS TRUE. GOTO PROPER LINE NUMBER.

DoElse@@            ldb       #3                  ; BUMP IP PAST LINE NUMBER.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES IF PRESENT.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #ELSETOK            ; IS IT THE "ELSE" CLAUSE.
                    bne       Done@@              ; NO RETURN.
                    iny                           ; PASS ELSE TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    bra       Goto@@              ; DO A GOTO.
Done@@              rts                           ; RETURN.

;*******************************************************************************

REEP                proc                          ; PROGRAM A WORD OF EEPROM.
                    iny                           ; PASS UP THE OPEN PAREN.
                    jsr       RSKIPSPC            ; PASS UP ANY SPACES.
                    jsr       DONEXP              ; GO GET THE "SUBSCRIPT" OF THE EEPROM LOCATION.
                    iny:2                         ; PASS UP THE CLOSING PAREN.
                                                  ; PASS UP THE EQUALS TOKEN.
                    jsr       DONEXP              ; GET VALUE TO FROGRAM INTO EEPROM.
                    pshy                          ; SAVE THE Y REG.
                    ldy       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    ldd       2,y                 ; GET THE SUBSCRIPT FOR THE EEPROM LOCATION.
                    bmi       Fail@@              ; NEGATIVE SUBSCRIPTS NOT ALLOWED.
                    cpd       #MAXEESUB           ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       Go@@                ; YES. CONTINUE.
Fail@@              lda       #EESUBERR           ; EEPROM SUBSCRIPT ERROR.
                    jmp       RPTRERR             ; REPORT IT.

Go@@                lsld                          ; MULT THE SUBSCRIPT BY 2.
                    addd      #EEPBASAD           ; ADD IN THE EEPROM BASE ADDRESS.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    lda       ,x                  ; GET THE MOST SIGNIFIGANT BYTE OF THE CURRENT NUM.
                    cmpa      #$FF                ; DOES IT NEED ERASING?
                    beq       Skip@@              ; NO. SEE IF NEXT BYTE NEEDS ERASING.
                    bsr       ERASEBYT            ; YES. GO ERASE IT.
Skip@@              inx                           ; POINT TO NEXT BYTE.
                    lda       ,x                  ; GET NEXT BYTE.
                    cmpa      #$FF                ; DOES THIS BYTE NEED TO BE ERASED?
                    beq       Write@@             ; NO. GO WRITE DATA TO EEPROM.
                    bsr       ERASEBYT            ; YES. GO ERASE THE BYTE.
Write@@             lda       1,y                 ; GET LS BYTE OF WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    dex                           ; POINT TO THE MOST SIGNIFIGANT EEPROM LOCATION.
                    lda       ,y                  ; GET THE MS BYTE OF THE WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    puly                          ; RESTORE Y.
                    jsr       PULNUM              ; FIX UP NUM STACK.
                    jsr       PULNUM
                    rts                           ; RETURN.

;*******************************************************************************

ERASEBYT            proc
                    pshy
                    ldy       IOBaseV             ; Point to the base address of the I/O Registers.
                    ldb       #$16                ; SET UP BYTE ERASE MODE, ADDR LATCH, ERASE
                    stb       PPROG,y             ; VOLTAGE OFF.
                    sta       ,x                  ; LATCH ADDRESS.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE ERASING.
                    ldb       #$17                ; TURN ON ERASE VOLTAGE
                    stb       PPROG,y
                    bsr       DLY10MS             ; DELAY ABOUT 10 MS.
                    ldb       #$16                ; TURN PROGRAMING VOLTAGE OFF.
                    stb       PPROG,y
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG,y
                    puly
                    rts                           ; RETURN.

;*******************************************************************************

PROGBYTE            proc
                    pshy
                    ldy       IOBaseV             ; Point to the base address of the I/O Registers.
Loop@@              ldb       #$02                ; SET UP NORMAL PROGRAMING MODE, ADDRESS/DATA
                    stb       PPROG,y             ; LATCHED, PROGRAMING VOLTAGE OFF.
                    sta       ,x                  ; LATCH DATA & ADDRESS.
                    psha                          ; SAVE THE DATA FOR COMPARE AFTER PROGRAMING.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE PROGRAMING.
                    ldb       #$03                ; TURN ON PROGRAMING VOLTAGE.
                    stb       PPROG,y
                    bsr       DLY10MS             ; LEAVE IT ON FOR 10 MS.
                    ldb       #$02                ; NOW, TURN THE PROGRAMMING VOLTAGE OFF.
                    stb       PPROG,y
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG,y             ; PUT THE EEPROM BACK IN THE READ MODE.
                    pula                          ; RESTORE THE DATA TO SEE IF IT WAS PROGRAMMED.
                    cmpa      ,x                  ; WAS THE DATA WRITTEN PROPERLY?
                    bne       Loop@@              ; NO. TRY AGAIN.
                    puly                          ; Restore Y.
                    rts                           ; YES. RETURN.

;*******************************************************************************
                              #Cycles
DLY10MS             proc
                    pshx                          ; SAVE X.
                    ldx       #DELAY@@            ; GET DELAY CONSTANT.
                              #Cycles
Loop@@              dex                           ; DECREMENT THE COUNT. DONE?
                    bne       Loop@@              ; NO. DELAY SOME MORE.
                              #temp :cycles
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

DELAY@@             equ       10*BUS_KHZ-:cycles-:ocycles/:temp

;*******************************************************************************

RINBYTE             proc
                    jsr       CHCKDEV             ; GO CHECK FOR AN ALTERNATE DEVICE DESIGNATION.
                    jsr       RVARPTR             ; GO GET POINTER TO THE BYTE INPUT VARIABLE.
                    xgdx                          ; PUT THE POINTER INTO X.
                    jsr       INBYTE              ; GO GET A BYTE FROM THE SPECIFIED INPUT DEVICE.
                    tab                           ; PUT THE BYTE IN THE L.S.BYTE.
                    clra                          ; ZERO THE UPPER BYTE.
                    std       ,x                  ; PUT IT IN THE VARIABLE.
                    clr       DEVNUM              ; RESET TO DEVICE #0.
                    rts                           ; RETURN.

;*******************************************************************************

RTIME               proc
                    iny                           ; POINT PAST THE EQUALS TOKEN.
                    jsr       DONEXP              ; GO EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    rts                           ; RETURN.

;*******************************************************************************

RRTIME              proc
                    sei                           ; disable interrupts.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    ldx       IOBaseV             ; Point to the I/O Base Address.
                    ldd       TCNT,x              ; get the current value of the timer counter.
                    jsr       TIMINTS3            ; go initialize the TOC using the timer interrupt code.
                    clrd
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    cli
                    rts                           ; RETURN.

;*******************************************************************************

RPACC               proc
                    iny                           ; POINT PAST EQUALS TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    tsta                          ; IS THE NUMBER WITHIN RANGE?
                    beq       Go@@                ; YES. GO SETUP THE PACC REGISTER.
                    lda       #PACCARGE           ; NO. REPORT AN ERROR.
                    jmp       RPTRERR

Go@@                ldx       IOBaseV
                    stb       PACNT,x             ; PUT NUMBER IN PULSE ACC.
                    rts                           ; RETURN.

;*******************************************************************************

RONTIME             proc
                    bsr       CHCKIMED            ; NOT ALLOWED IN IMMEDIATE.
                    jsr       DONEXP              ; GO EVALUATE THE TIME "MATCH" EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMECMP             ; PUT IN THE COMPARE REGISTER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; PASS UP COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    sty       ONTIMLIN            ; SAVE THE POINTER TO THE LINE NUMBER.
                    bra       ?RONIRQ             ; GO FINISH UP.

;*******************************************************************************

RONIRQ              proc
                    bsr       CHCKIMED
                    jsr       DONEXP              ; GO CHECK TO SEE IF WE ARE TO ENABLE OR DISABLE.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO COMMA.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO LINE NUMBER.
                    jsr       PULNUM              ; GET MODE. SHOULD WE ENABLE THE FUNCTION?
                    bne       Enable@@            ; YES.
                    std       ONIRQLIN            ; NO. MAKE THE LINE NUMBER 0.
                    bra       ?RONIRQ             ; GO FINISH UP.

Enable@@            sty       ONIRQLIN            ; SAVE THE POINTER TO THE LINE NUMBER,
?RONIRQ             ldb       #3                  ; MOVE IP PAST THE LINE NUMBER.
                    aby
                    rts                           ; RETURN.

;*******************************************************************************

RRETI               proc
                    bsr       CHCKIMED
                    tpa                           ; CHECK TO SEE IF THE INTERRUPT MASK IS SET.
                    bita      #$10                ; ARE WE IN AN INTERRUPT ROUTINE?
                    bne       Go@@                ; SINCE THE IRQ MASK IS SET WE MUST BE.
                    lda       #NOTINTER           ; NO. FLAG AN ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

Go@@                ldd       SCURLINE            ; RESTORE THE MAIN PROGRAM CURRENT LINE.
                    std       CURLINE
                    ldd       SADRNXLN            ; RESTORE MAIN PROGRAM "ADDRESS OF THE NEXT LINE".
                    std       ADRNXLIN
                    ins:2                         ; TAKE THE RETURN ADDRESS OFF THE STACK.
                    rti                           ; GO BACK TO WHERE WE LEFT OFF.

;*******************************************************************************

CHCKIMED            proc
                    tst       IMMED               ; ARE WE IN THE IMMEDIATE MODE?
                    beq       Done@@              ; NO. JUST RETURN.
                    lda       #NOTALERR           ; YES. THIS COMMAND NOT ALLOWED.
                    jmp       RPTRERR             ; REPORT THE ERROR.
Done@@              rts                           ; RETURN.

;*******************************************************************************

RONPACC             proc
                    bsr       CHCKIMED            ; THIS INSTRUCTION NOT ALLOWED IN IMMED MODE.
                    jsr       DONEXP              ; GO EVALUATE THE COUNT MODE EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER COMMA.
                    jsr       DONEXP              ; GO EVALUATE THE INTERRUPT MODE EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS.
                    sty       ONPACLIN            ; SAVE POINTER TO INTERRUPT ROUTINE.
                    jsr       PULNUM              ; GET INTERRUPT MODE OFF STACK.
                    cpd       #1                  ; IS THE ARGUMENT <=1?
                    bls       Go@@                ; YES. ARG. OK.
Fail@@              lda       #INTMODER           ; NO. GET ERROR CODE.
                    jmp       RPTRERR
Go@@                lda       #$10                ; GET BIT TO ENABLE INTERRUPT.
                    tstb                          ; WAS THE ARGUMENT 0?
                    beq       EnableInts@@        ; YES. GO ENABLE INTS. ON EACH COUNT.
                    lsla                          ; NO. ENABLE INTS. ON PACC OVERFLOW ONLY.
EnableInts@@        ldx       IOBaseV
                    sta       TMSK2,x
                    jsr       PULNUM              ; GET THE COUNT MODE OFF THE STACK.
                    bne       SetMode@@           ; GO SET THE MODE IF NOT 0.
                    ldx       IOBaseV
                    clr       PACTL,x             ; TURN OFF THE PULSE ACCUMULATOR.
                    std       ONPACLIN            ; CLEAR POINTER TO LINE NUMBER.
                    bra       Done@@              ; GO CLEAN UP & RETURN.
SetMode@@           cpd       #4                  ; IS THE ARGUMENT IN RANGE?
                    bhi       Fail@@              ; YES. REPORT AN ERROR.
                    addb      #3                  ; GET BIT TO ENABLE PACC.
                    lslb:4
                    ldx       IOBaseV
                    stb       PACTL,x             ; ENABLE THE PACC & SET MODE.
Done@@              pula                          ; GET OLD I-BIT STATUS OFF STACK.
                    tap                           ; RESTORE OLD STATUS.
                    ldb       #3
                    aby                           ; PASS UP LINE NUMBER.
                    rts                           ; RETURN.
;                   include   'rexpres.mod'
                    title     REXPRES

;*******************************************************************************
;                                                                              *
;               RUNTIME EXPRESSION EVALUATION SUBROUTINE                       *
;                                                                              *
;*******************************************************************************

DONEXP              proc
                    lda       #OPARNTOK           ; USE AN OPEN PAREN AS AN END OF EXPRESSION MARKER.
                    jsr       PSHOP               ; PUSH OPEN PAREN ON THE STACK.
Loop@@              lda       ,y                  ; GET THE NEXT CHARACTER IN THE EXPRESSION.
                    cmpa      #OPARNTOK           ; IS IT AN OPEN PAREN?
                    bne       Cont@@              ; NO. CONTINUE.
                    iny                           ; POINT TO NEXT TOKEN.
                    bsr       DONEXP              ; GO DO A SUBEXPRESSION.
                    iny                           ; MOVE THE IP PAST THE CLOSING PAREN.
                    bra       Loop@@              ; GO GET THE NEXT CHARACTER.

Cont@@              tsta                          ; CHECK FOR OPERATOR OR OPERAND.
                    bpl       Function@@          ; IF NOT VARIABLE OR CONSTANT, GO CHECK FOR FUNCT.
                    bsr       PSHNUM              ; GO PUSH OPERAND ONTO STACK.
                    bra       Loop@@              ; GO GET NEXT TOKEN.

Function@@          jsr       CHKNFUN             ; GO CHECK FOR FUNCTION THAT RETURNS A NUMBER.
                    jsr       CHCKEE              ; GO CHECK FOR END OF EXPRESSION.
                    bcc       Save@@              ; IF NOT END OF EXPRESSION, GO PUSH OPERATOR.
                    rts                           ; IF AT END, RETURN.

Save@@              iny                           ; POINT TO THE NEXT TOKEN.
                    jsr       PSHOP               ; PUSH OPERATOR ONTO STACK.
                    bra       Loop@@              ; GO GET NEXT TOKEN.

;*******************************************************************************
;        PSHNUM SUBROUTINE
;
;        PUSHES A NUMERIC OPERAND (CONSTANT OR VARIABLE) VALUE ONTO THE
;        OPERAND STACK.

PSHNUM              proc
                    cmpa      #IVARTOK            ; IS IT AN INTEGER SCALER VARIABLE?
                    bne       Const@@             ; NO. GO CHECK FOR CONSTANT.
                    ldd       1,y                 ; YES. GET THE "OFFSET" ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE VARIABLE TABLE.
                    xgdx                          ; GET THE ADDRESS INTO X.
                    ldb       #$03                ; BUMP INTERPRETER POINTER PAST "VARIABLE".
                    aby
                    ldd       3,x                 ; GET THE VARIABLE VALUE.
                    bra       PushOpStackReturn   ; GO PUT IT ON THE STACK.

Const@@             cmpa      #ICONTOK            ; IS IT AN INTEGER CONSTANT?
                    bne       IntArray@@          ; NO. GO CHECK FOR AN INTEGER ARRAY VARIABLE.
                    ldx       1,y                 ; GET THE CONSTANT VALUE INTO X.
                    ldb       #$04
                    addb      3,y
                    aby
                    xgdx                          ; PUT THE CONSTANT VALUE INTO D.
                    bra       PushOpStackReturn   ; GO PUT IT ON THE STACK.

IntArray@@          cmpa      #IAVARTOK           ; IS IT AN INTEGER ARRAY?
                    bne       Fail@@              ; NO. GO CHECK FOR A STRING VARIABLE.
                    bsr       CALCSUB             ; GO GET BASE ADDR. & SUBSCRIPT OF ARRAY.
                    pshy                          ; SAVE THE INTERPRETER POINTER.
                    pshx                          ; PUT THE BASE ADDRESS OF THE ARRAY ON THE STACK.
                    asld                          ; MULTIPLY THE SUBSCRIPT BY THE # OF BYTES/ELEMENT.
                    tsy                           ; POINT TO THE BASE ADDRESS.
                    addd      ,y                  ; GET ADDRESS OF THE ELEMENT.
                    pulx                          ; RESTORE X.
                    puly                          ; RESTORE Y
                    xgdx                          ; PUT ELEMENT ADDRESS INTO X.
                    ldd       ,x                  ; GET VALUE OF ELEMENT IN D.
                    bra       PushOpStackReturn

Fail@@              lda       #ILTOKERR
                    jmp       RPTRERR

;*******************************************************************************

PushOpStackReturn   proc
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    dex:2                         ; MAKE ROOM ON THE STACK FOR NEW OPERAND.
                    cpx       ENUMSTK             ; HAS THE STACK OVERFLOWED?
                    bhs       Save@@              ; NO. GO STACK THE VALUE.
                    lda       #MSTKOERR           ; YES.
                    sta       ERRCODE
                    jmp       RPTRERR             ; GO REPORT THE ERROR.
Save@@              stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    std       ,x                  ; PUT THE VALUE ON THE STACK.
                    rts                           ; RETURN.

;*******************************************************************************
;        THIS SUBROUTINE CALCULATES BOTH THE BASE ADDRESS AND THE
;        SUBSCRIPT OF THE ARRAY VARIABLE THAT IS CURRENTLY POINTED TO BY
;        THE Y-REG. IT CHECKS TO SEE IF THE VARIABLE HAS BEEN DIMENTIONED
;        AND IF THE SUBSCRIPT IS IN RANGE. THE ROUTINE RETURNS WITH THE
;        ADDRESS OF THE ARRAY IN THE X-REG. & THE SUBSCRIPT IN THE D-REG.

CALCSUB             proc
                    ldd       1,y                 ; GET THE VARIABLE OFFSET ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START OF THE VARIABLE AREA.
                    xgdx                          ; PUT ADDRESS INTO X.
                    ldx       3,x                 ; GET THE ACTUAL STORAGE ADDRESS.
                                                  ; HAS THE ARRAY BEEN DIMENTIONED?
                    bne       Go@@                ; YES. CONTINUE.
                    lda       #UNDIMERR           ; NO. UNDIMENTIONED ARRAY REFERENCE.
Fail@@              jmp       RPTRERR             ; GO REPORT THE ERROR.
Go@@                ldb       #4                  ; SET POINTER TO START OF SUBSCRIPT EXPRESSION.
                    aby
                    pshx                          ; SAVE THE POINTER TO THE ARRAY STORAGE AREA.
                    jsr       DONEXP              ; GO GET THE SUBSCRIPT.
                    iny                           ; BUMP IP PAST THE CLOSING PAREN OF THE SUBSCRIPT.
                    pulx                          ; RESTORE X.
                    jsr       PULNUM              ; GET SUBSCRIPT FROM THE OPERAND STACK.
                    cpd       ,x                  ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       Done@@              ; YES. CONTINUE.
                    lda       #SUBORERR           ; NO. SUBSCRIPT OUT OF RANGE ERROR.
                    bra       Fail@@              ; GO REPORT IT.
Done@@              inx:2                         ; BYPASS THE SUBSCRIPT LIMIT.
                    rts

;*******************************************************************************

PULNUM              proc
                    pshx                          ; SAVE THE X-REG.
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    ldd       ,x                  ; GET THE OPERAND.
                    inx:2                         ; BUMP THE STACK POINTER.
                    stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    pulx                          ; RESTORE THE X-REG.
                    cpd       #0                  ; "TEST" THE OPERAND BEFORE WE RETURN.
                    rts                           ; RETURN.

;*******************************************************************************
;        ***** chcknfun() *****
;
;        checks for a numeric function and performs it if present

CHKNFUN             proc
                    cmpa      #FUNCTFLG           ; IS THIS A FUNCTION CALL?
                    beq       Go@@                ; YES. GO DO THE FUNCTION.
                    rts                           ; NO. JUST RETURN.

Go@@                lda       1,y                 ; GET THE FUNCTION CODE BYTE IN B.
                    deca                          ; SUBTRACT 1 FOR INDEXING.
                    ldb       #3                  ; BUMP THE IP.
                    aby                           ; POINT TO THE FIRST ELEMENT IN THE EXPRESSION.
                    tab                           ; PUT THE FUNCTION NUMBER INTO B.
                    aslb                          ; MULT BY THE NUMBER OF BYTES/ADDRESS.
                    ldx       #Table@@            ; POINT TO THE FUNCTION ADDRESS TABLE.
                    abx                           ; POINT TO THE PROPER FUNCTION.
                    ldx       ,x                  ; GET THE ADDRESS INTO X.
                    jsr       ,x                  ; GO DO THE FUNCTION.
                    iny                           ; PUT IP PAST THE CLOSING PAREN.
                    lda       ,y                  ; GET NEXT CHARACTER.
                    rts                           ; RETURN.

;-------------------------------------------------------------------------------

Table@@             dw        RFDIV
                    dw        ICHRS               ; "ICHRS" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    dw        RADC
                    dw        RABS
                    dw        RRND
                    dw        RSGN
                    dw        ITAB                ; "ITAB" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    dw        RCALL
                    dw        RPEEK
                    dw        RFEEP               ; "EEP" AS A FUNCTION.
                    dw        IHEX                ; "IHEX" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    dw        RFPORTA
                    dw        RFPORTB
                    dw        RFPORTC
                    dw        RFPORTD
                    dw        RFPORTE
                    dw        RFTIME
                    dw        IHEX2               ; "IHEX2" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    dw        RFPACC

;*******************************************************************************
;        ***** chckee() *****
;
;        if the current token is a semicolon, comma, colon, or space
;        all pending operations on the math stack are performed and
;        we return with the carry set

CHCKEE              proc
                    cmpa      #CPARNTOK           ; IS IT A CLOSED PAREN?
                    beq       PushOp@@            ; YES.
                    cmpa      #MEOLTOK            ; IS IT ONE OF THE "EXPRESSION END" TOKENS?
                    bhs       ExprEnd@@           ; YES.
                    clc                           ; FLAG "NOT AT THE END OF EXPRESSION".
                    rts                           ; RETURN.

ExprEnd@@           lda       #CPARNTOK           ; END OF EXPRESSION FOUND. PERFORM ALL PENDING
PushOp@@            bsr       PSHOP               ; OPERATIONS.
                    sec                           ; FLAG END OF EXPRESSION.
                    rts

;*******************************************************************************

PSHOP               proc
                    ldx       OPSTACK             ; GET THE OPERATOR STACK POINTER.
                    dex                           ; DECREMENT THE STACK POINTER.
                    cpx       EOPSTK              ; DID THE STACK OVERFLOW?
                    bne       Go@@                ; NO. CONTINUE.
                    lda       #MSTKOERR           ; YES.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.
Go@@                stx       OPSTACK
                    sta       ,x                  ; PUT IT ON THE STACK.
Loop@@              ldx       OPSTACK
                    lda       ,x                  ; GET THE NEW OPERATOR OFF THE TOP OF STACK.
                    cmpa      #OPARNTOK           ; IS IT AN OPEN PAREN?
                    beq       Done@@              ; YES. GO PUSH IT.
                    ldb       1,x                 ; GET THE PREVIOUS OPERATOR OFF THE STACK.
                    andb      #$F0                ; MASK ALL BUT THE PRECIDENCE VALUE.
                    anda      #$F0                ; MASK ALL BUT THE OPERATOR PRECIDENCE.
                    cba                           ; IS THE PRECIDENCE OF THE CURRENT OPERATOR >=
                                                  ; THE OPERATOR ON THE TOP OF THE STACK?
                    bhi       Done@@              ; NO. JUST GO PUSH IT ON THE STACK.
                    lda       1,x                 ; YES. GET THE PREVIOUS OPERATOR FROM THE STACK.
                    ldb       ,x                  ; GET THE CURRENT OPERATOR FROM THE STACK.
                    cmpb      #CPARNTOK           ; IS THE CURRENT OPERATOR A CLOSED PAREN?
                    bne       Save@@              ; NO. CONTINUE.
                    cmpa      #OPARNTOK           ; YES. IS THE PREVIOUS OPERATOR AN OPEN PAREN?
                    bne       Save@@              ; NO. CONTINUE.
                    inx:2                         ; YES. KNOCK BOTH OPERATORS OFF THE STACK.
                    stx       OPSTACK             ; SAVE THE STACK POINTER.
Done@@              rts                           ; RETURN.
Save@@              stb       1,x                 ; PUT IT ON THE STACK.
                    inx                           ; UPDATE THE STACK POINTER.
                    stx       OPSTACK
                    bsr       DOOP                ; GO DO THE OPERATION.
                    bra       Loop@@              ; GO TRY FOR ANOTHER OPERATION.

;*******************************************************************************

DOOP                proc
                    cmpa      #$70                ; IS IT A UINARY OPERATOR?
                    blo       _1@@                ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$70                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR7              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

_1@@                cmpa      #$60                ; IS IT THE "^" OPERATOR?
                    blo       _2@@                ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$60                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR6              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

_2@@                cmpa      #$50                ; IS IT MULTIPLY, DIVIDE, OR MOD?
                    blo       _3@@                ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$50                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR5              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

_3@@                cmpa      #$40                ; IS IT ADD OR SUBTRACT?
                    blo       _4@@                ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$40                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR4              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

_4@@                cmpa      #$30                ; IS IT A LOGICAL OPERATOR?
                    blo       _5@@                ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$30                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR3              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

_5@@                cmpa      #$20                ; IS IT AND, OR, OR EOR?
                    blo       Fail@@              ; NO. ERROR.
                    suba      #$20                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR2              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

Fail@@              lda       #ILTOKERR           ; ILLEGAL OPERATOR TOKEN ENCOUNTERED.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                tab                           ; PUT THE OFFSET IN B.
                    aslb                          ; MULTIPLY THE OFFSET BY 2.
                    abx                           ; POINT TO THE ROUTINE ADDRESS.
                    ldx       ,x                  ; GET THE ADDRESS.
                    jmp       ,x                  ; GO DO THE OPERATION & RETURN.

;-------------------------------------------------------------------------------

HEIR7               fdb       RINDIR
                    fdb       RNOT
                    fdb       RNEG
HEIR6               fdb       RPWR
HEIR5               fdb       RMULT
                    fdb       RDIV
                    fdb       RMOD
HEIR4               fdb       RPLUS
                    fdb       RMINUS
HEIR3               fdb       RLT
                    fdb       RGT
                    fdb       RLTEQ
                    fdb       RGTEQ
                    fdb       REQ
                    fdb       RNOTEQ
HEIR2               fdb       RAND
                    fdb       RORV
                    fdb       REOR

;*******************************************************************************

REOR                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    eora      ,x
                    eorb      1,x
?REOR               std       ,x
                    rts

;*******************************************************************************

RORV                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    ora       ,x
                    orb       1,x
                    bra       ?REOR

;*******************************************************************************

RAND                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    anda      ,x
                    andb      1,x
                    bra       ?REOR

;*******************************************************************************

RPLUS               proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    addd      ,x
                    bra       ?REOR

;*******************************************************************************

RMINUS              proc
                    ldx       NUMSTACK
                    ldd       2,x
                    subd      ,x
                    inx:2
                    std       ,x
                    stx       NUMSTACK
                    rts

;*******************************************************************************

RDIV                proc
                    bsr       RDIVS               ; GO DO A SIGNED DIVIDE.
                    jsr       PULNUM              ; GET INTEGER RESULT OFF STACK.
                    ldx       NUMSTACK            ; POINT TO NUMERIC STACK.
                    std       ,x                  ; OVERWRITE REMAINDER.
                    rts                           ; RETURN.

;*******************************************************************************

RDIVS               proc
                    ldx       NUMSTACK            ; POINT TO NUMERIC STACK.
                    lda       ,x                  ; GET UPPER BYTE OF DIVISOR.
                    eora      2,x                 ; GET SIGN OF THE RESULT.
                    psha                          ; SAVE RESULT.
                    ldd       ,x                  ; GET DIVISOR OFF NUMERIC STACK. IS IT ZERO?
                    bne       Go@@                ; NO. CONTINUE.
                    lda       #ZDIVERR            ; YES. GET DIVIDE BY ZERO ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.
Go@@                bpl       Pos@@               ; IF POSITIVE IT'S OK.
                    jsr       RNEG                ; IF NOT MAKE IT POSITIVE.
Pos@@               tst       2,x                 ; IS THE DIVIDEND NEGATIVE?
                    bpl       Cont@@              ; NO. CONTINUE.
                    ldd       2,x                 ; YES. GET THE NUMBER.
                    negd                          ; NEGATE IT.
                    std       2,x                 ; SAVE THE RESULT.
Cont@@              ldd       ,x                  ; GET THE DIVISOR.
                    ldx       2,x                 ; GET THE DIVIDEND.
                    xgdx                          ; PUT THEM IN THE PROPER REGISTERS.
                    idiv                          ; DO AN UNSIGNED DIVIDE.
                    pshx                          ; SAVE THE QUOTIENT.
                    ldx       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    std       2,x                 ; SAVE THE REMAINDER.
                    puld                          ; GET THE QUOTIENT.
                    std       ,x                  ; PUT IT ON THE NUMERIC STACK.
                    pula                          ; GET THE SIGN OF THE RESULT.
                    tsta                          ; SET THE CONDITION CODES.
                    bpl       Done@@              ; IF PLUS, RESULT OK AS IS.
                    jsr       RNEG                ; MAKE THE QUOTIENT NEGATIVE.
                    ldd       2,x                 ; GET THE REMAINDER.
                    negd                          ; MAKE IT NEGATIVE.
                    std       2,x                 ; SAVE THE RESULT.
Done@@              rts                           ; RETURN.

;*******************************************************************************

RMOD                proc
                    bsr       RDIVS               ; GO GET QUOTIENT & REMAINDER.
                    jsr       PULNUM              ; REMOVE INTEGER RESULT & LEAVE REMAINDER.
                    rts                           ; RETURN.

;*******************************************************************************

RMULT               proc
                    pshy
                    ldx       NUMSTACK
                    lda       1,x
                    ldb       3,x
                    mul
                    pshd
                    tsy
                    lda       1,x
                    ldb       2,x
                    mul
                    addb      ,y
                    stb       ,y
                    lda       ,x
                    ldb       3,x
                    mul
                    addb      ,y
                    stb       ,y
                    inx:2
                    puld
                    std       ,x
                    stx       NUMSTACK
                    puly
                    rts

;*******************************************************************************

RINDIR              proc
                    rts

;*******************************************************************************

RNOT                proc
                    ldx       NUMSTACK
                    ldd       ,x
                    comd
                    std       ,x
                    rts

;*******************************************************************************

RNEG                proc
                    bsr       RNOT
                    incd
                    std       ,x
                    rts

;*******************************************************************************

RLT                 proc
                    bsr       CMPNUM
                    bge       ?RLT
?RLT2               inc       3,x
?RLT                inx:2
                    stx       NUMSTACK
                    rts

;*******************************************************************************

RGT                 proc
                    bsr       CMPNUM
                    ble       ?RLT
                    bra       ?RLT2

;*******************************************************************************

RLTEQ               proc
                    bsr       CMPNUM
                    bgt       ?RLT
                    bra       ?RLT2

;*******************************************************************************

RGTEQ               proc
                    bsr       CMPNUM
                    blt       ?RLT
                    bra       ?RLT2

;*******************************************************************************

REQ                 proc
                    bsr       CMPNUM
                    bne       ?RLT
                    bra       ?RLT2

;*******************************************************************************

RNOTEQ              proc
                    bsr       CMPNUM
                    beq       ?RLT
                    bra       ?RLT2

;*******************************************************************************

CMPNUM              proc
                    ldx       NUMSTACK
                    ldd       2,x
                    clr       2,x
                    clr       3,x
                    cpd       ,x
                    rts

;*******************************************************************************

RPWR                proc
                    rts

;*******************************************************************************

RABS                proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    bpl       ?RABS
?RABS_Neg           negd
?RABS               std       ,x
                    rts

;*******************************************************************************

RSGN                proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    beq       ?RABS
                    ldd       #1
                    tst       ,x
                    bpl       ?RABS
                    bra       ?RABS_Neg

;*******************************************************************************

RCALL               proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    jsr       ,x
                    bra       RPEEK1

;*******************************************************************************

RPEEK               proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    ldb       ,x
                    clra
;                   bra       RPEEK1

;*******************************************************************************

RPEEK1              proc
                    ldx       NUMSTACK
                    std       ,x
                    rts

;*******************************************************************************

RFEEP               proc
                    jsr       DONEXP              ; GO GET SUBSCRIPT OF EEPROM ARRAY.
                    ldx       NUMSTACK            ; POINT TO THE OPERAND STACK.
                    ldd       ,x                  ; GET THE SUBSCRIPT OFF THE STACK.
                    cpd       #MAXEESUB           ; IS IT WITHIN THE LIMIT?
                    bls       Go@@                ; YES. GO GET THE VALUE.
                    lda       #EESUBERR           ; NO. SUBSCRIPT ERROR.
RFEEP2              jmp       RPTRERR             ; REPORT THE ERROR.
Go@@                lsld                          ; MULT THE SUBSCRIPT BY 2.
                    addd      #EEPBASAD           ; ADD IN THE BASE ADDRESS OF THE EEPROM ADDRESS.
                    xgdx                          ; PUT THE ADDRESS IN X.
                    ldd       ,x                  ; GET THE DATA.
                    bra       RPEEK1              ; GO STEAL SOME CODE.

;*******************************************************************************

RFDIV               proc
                    jsr       DONEXP              ; GO EVALUATE THE DIVIDEND EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; PASS UP THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    jsr       DONEXP              ; EVALUATE THE DIVISOR EXPRESSION.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    ldd       2,x                 ; GET THE DIVIDEND.
                    ldx       ,x                  ; GET THE DIVISOR.
                    fdiv                          ; DO THE FRACTIONAL DIVIDE.
                    bvc       Go@@                ; ALL IS OK IF V=0. (IX > D).
                    lda       #OVDV0ERR           ; ERROR. EITHER OVERFLOW OR /0 ERROR.
?RFEEP2             bra       RFEEP2              ; GO REPORT IT.

Go@@                xgdx                          ; PUT QUOTIENT IN D.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    inx:2                         ; REMOVE DIVISOR FROM STACK.
                    std       ,x                  ; PUT QUITIENT ON OPERAND STACK.
                    stx       NUMSTACK            ; SAVE NEW VALUE OF STACK POINTER.
                    rts                           ; RETURN.

;*******************************************************************************

RADC                proc
                    jsr       DONEXP              ; GO GET THE CHANNEL NUMBER TO CONVERT.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    ldd       ,x                  ; GET THE CHANNEL NUMBER.
                    bmi       Fail@@              ; NEGATIVE CHANNEL NUMBERS ARE ILLEGAL.
                    cpd       #7                  ; IS IT A VALID CHANNEL NUMBER?
                    bls       Go@@                ; YES. GO CONVERT IT.
Fail@@              lda       #INVCHERR           ; NO. INVALID CHANNEL NUMBER.
                    bra       ?RFEEP2             ; GO REPORT THE ERROR.

Go@@                ldx       IOBaseV
                    stb       ADCTL,x             ; START THE CONVERSION ON THE SELECTED.
WaitAD@@            tst       ADCTL,x             ; IS THE CONVERSION COMPLETE?
                    bpl       WaitAD@@            ; NO. WAIT FOR 4 CONVERSIONS ON 1 CHANNEL.
                    clra                          ; YES. NOW AVERAGE THE 4 CONVERSIONS.
                    ldb       ADR1,x              ; GET 1ST RESULT.
                    addb      ADR2,x              ; ADD IN THE SECOND.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR3,x              ; ADD IN THE THIRD.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR4,x              ; ADD IN THE FOURTH.
                    adca      #0                  ; ADD IN CARRY.
                    lsrd:2                        ; DIVIDE RESULT BY 4.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    std       ,x                  ; PUT THE RESULT ON THE OPERAND STACK.
                    rts                           ; RETURN.

;*******************************************************************************

RRND                proc
                    jsr       DONEXP              ; GO GET FUNCTION ARGUMENT.
                    ldx       NUMSTACK            ; GET ARGUMENT OFF STACK. GET NEW RANDOM NUMBER?
                    ldd       ,x
                    beq       NextNum@@           ; YES. GO GET NEXT RANDOM NUMBER IN THE SERIES.
                    bmi       New@@               ; IF NEG., START A NEW SERIES.
                    ldd       RANDOM              ; IF POSITIVE, GET LAST RANDOM NUMBER.
                    bra       Done@@              ; RETURN.

New@@               ldx       IOBaseV
                    ldd       TCNT,x              ; USE THE TIMER VALUE AS THE NEW SEED.
                    std       RANDOM              ; SAVE IT.
NextNum@@           ldd       RANDOM              ; GET PREVIOUS RANDOM NUMBER (USE AS SEED).
                    aslb                          ; DO SOME OPERATIONS.
                    aba
                    ldb       RANDOM+1
                    asld:2
                    addd      RANDOM
                    addd      #$3619
                    std       RANDOM
Done@@              lsrd                          ; MAKE THE NUMBER POSITIVE.
                    std       ,x                  ; PUT THE NUMBER ON THE STACK.
                    rts                           ; RETURN.

;*******************************************************************************

ITAB                proc
ICHRS               proc
IHEX                proc
IHEX2               proc
                    lda       #PRFUNERR           ; THESE FUNCTIONS MUST BE USED ONLY IN
                    jmp       RPTRERR             ; PRINT STATEMENTS.

;*******************************************************************************

RFTIME              proc
                    ldd       TIMEREG             ; GET THE TIME IN SECONDS.
                    bra       RFPORTA2            ; GO PUT NUMBER ON THE STACK.

;*******************************************************************************

RFPACC              proc
                    ldx       IOBaseV
                    ldb       PACNT,x             ; GET THE CURRENT VALUE OF THE PULSE ACCUMULATOR.
                    clra
                    bra       RFPORTA2            ; GO PUT THE NUMBER ON THE STACK.

;*******************************************************************************

RFPORTA             proc
                    ldb       #PORTAIO            ; GET DATA FROM PORTA.
?RFPORT             ldx       IOBaseV
                    abx
                    ldb       ,x
                    clra                          ; CLEAR UPPER BYTE OF WORD.
RFPORTA2            dey:2                         ; DECREMENT IP BECAUSE CALLING ROUTINE WILL TRY
                                                  ; TO BUMP IT PAST AN OPENING & CLOSING PAREN
                                                  ; WHICH ISN'T THERE.
                    jmp       PushOpStackReturn   ; GO PUSH VALUE ON OPERAND STACK & RETURN.

;*******************************************************************************

RFPORTB             proc
                    ldb       #PORTBIO
                    bra       ?RFPORT

;*******************************************************************************

RFPORTC             proc
                    ldb       #PORTCIO
                    bra       ?RFPORT

;*******************************************************************************

RFPORTD             proc
                    ldb       #PORTDIO
                    bra       ?RFPORT

;*******************************************************************************

RFPORTE             proc
                    ldb       #PORTEIO
                    bra       ?RFPORT
;                   opt       lis
;                   include   'iopkg.mod'
                    title     IOPKG

;*******************************************************************************

OUTBYTE             proc
                    inc       PRINTPOS            ; INCREMENT THE CURRENT PRINT POSITION.
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #OUTABLE            ; POINT TO THE OUTPUT VECTOR TABLE.
OUTBYTE1            ldb       DEVNUM              ; GET THE CURRENT DEVICE NUMBER.
                    aslb                          ; MULT BY 2.
                    abx                           ; POINT TO THE ADDRESS OF THE OUTPUT ROUTINE.
                    ldx       ,x                  ; GET THE ADDRESS. HAS THE VECTOR BEEN INITIALIZED?
                    bne       Done@@              ; YES. GO OUTPUT THE CHARACTER.
                    clr       DEVNUM              ; NO. RESET TO DEVICE #0.
                    lda       #UNINIERR           ; GO REPORT AN UNINITIALIZED I/O VECTOR ERROR.
                    jmp       RPTRERR
Done@@              jsr       ,x                  ; GO OUTPUT THE CHARACTER.
                    pulx                          ; RESTORE X.
                    pulb                          ; RESTORE B.
                    rts                           ; RETURN.

;*******************************************************************************

INBYTE              proc
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #INTABLE            ; POINT TO THE INPUT VECTOR TABLE.
                    bra       OUTBYTE1            ; GO USE THE SAME CODE AS OUTBYTE.

          #if * > $FF00
                    #Fatal    BASIC is too large
          #endif
                    org       $FF00

;*******************************************************************************

ACIAIN              proc
                    bsr       ACIAINNE            ; GO GET CHARACTER FROM ACIA, NO ECHO.
;                   bra       ACIAOUT             ; GO ECHO CHARACTER.

;*******************************************************************************

ACIAOUT             proc
                    psha                          ; SAVE THE CHARACTER TO OUTPUT.
Loop@@              lda       ACIAST              ; GET THE ACIA STATUS.
                    bita      #$02                ; IS THE XMIT DATA REGISTER EMPTY?
                    beq       Loop@@              ; NO. WAIT TILL IT IS.
                    pula                          ; YES. GET BYTE TO SEND.
                    sta       ACIADT              ; SEND IT.
                    rts                           ; RETURN.

;*******************************************************************************

ACIAINNE            proc
Loop@@              lda       ACIAST              ; GET THE ACIA STATUS.
                    bita      #$01                ; HAS A CHARACTER BEEN RECIEVED?
                    beq       Loop@@              ; NO. WAIT TILL WE HAVE ONE.
                    lda       ACIADT              ; YES. GET THE CHARACTER.
                    rts                           ; RETURN.

;*******************************************************************************

ACIASTAT            proc
                    psha                          ; SAVE THE A-REG.
                    lda       ACIAST              ; GET THE ACIA STATUS.
                    bita      #$01                ; CHECK FOR A CHARACTER.
                    pula                          ; RESTORE A.
                    rts                           ; RETURN.

;*******************************************************************************

SCIIN               proc
                    pshx                          ; Save the index register.
                    ldx       IOBaseV
Loop@@              lda       SCSR,x              ; GET SCI STATUS.
                    anda      #$20                ; HAS A CHARACTER BEEN RECIEVED?
                    beq       Loop@@              ; NO. WAIT FOR CHARACTER TO BE RECIEVED.
                    lda       SCDR,x              ; GET THE CHARACTER.
                    pulx                          ; Restore X.
                    rts                           ; RETURN.

;*******************************************************************************

SCIOUT              proc
                    pshx                          ; Save the index register.
                    ldx       IOBaseV
                    psha                          ; SAVE THE CHARACTER TO SEND.
Loop@@              lda       SCSR,x              ; GET THE SCI STATUS.
                    bita      #$80                ; HAS THE LAST CHARACTER BEEN SHIFTED OUT?
                    beq       Loop@@              ; NO. WAIT TILL IT HAS.
                    pula                          ; RESTORE CHARACTER TO SEND.
                    sta       SCDR,x              ; SEND THE CHARACTER.
                    pulx                          ; Restore X.
                    rts                           ; RETURN.

;*******************************************************************************

SCISTAT             proc
                    pshx                          ; Save the index register.
                    ldx       IOBaseV
                    psha                          ; SAVE THE A-REG.
                    lda       SCSR,x              ; GET THE SCI STATUS.
                    bita      #$20                ; CHECK TO SEE IF A CHARACTER HAS BEEN RECIEVED.
                    pula                          ; RESTORE STATUS.
                    pulx                          ; Restore X.
                    rts                           ; RETURN W/ STATUS.

;*******************************************************************************

IODevInit           proc
                    bsr       InitACIA
                    bsr       InitSCI
                    lda       #JMPOP
                    sta       CONSTAT             ; INITIALIZE THE CONSOLE STATUS VECTOR.
                    sta       INCONNE             ; INITIALIZE THE INPUT FROM CONSOLE NO ECHO VECT.
                    ldd       #ACIASTAT           ; CONSOLE IS INITIALLY THE ACIA.
                    std       CONSTAT+1
                    ldd       #ACIAINNE           ; GET BYTE FROM ACIA, DON'T ECHO IT.
                    std       INCONNE+1
                    rts

;*******************************************************************************

InitSCI             proc
                    pshx                          ; Save the index register.
                    ldx       IOBaseV
                    lda       #$30                ; SET BAUD RATE TO 9600.
                    sta       BAUD,x
                    clr       SCCR1,x             ; SET FOR 8 BIT OPERATION, DISABLE WAKEUP.
                    lda       #$0C                ; ENABLE THE TRANSMITER & RECEIVER.
                    sta       SCCR2,x
                    lda       #$11                ; GET THE XON CHARACTER (CONTROL-Q).
                    sta       XONCH               ; INITIALIZE THE XON REGISTER.
                    lda       #$13                ; GET THE XOFF CHARACTER (CONTROL-S).
                    sta       XOFFCH              ; INITIALIZE THE XOFF CHARACTER.
                    pulx
                    rts                           ; RETURN.

;*******************************************************************************

InitACIA            proc
                    lda       #$13                ; VALUE TO RESET THE ACIA.
                    sta       ACIAST              ; RESET IT.
                    lda       #$56                ; SET /64, RTS=HI, 8-DATA/1 STOP
                    sta       ACIAST
                    rts                           ; RETURN.

;*******************************************************************************

PROUT               proc                          ; SEND A CHARACTER TO THE PRINTER.
                    bsr       SCISTAT             ; WAS AN "X-OFF" RECIEVED?
                    beq       Send@@              ; NO. GO SEND THE CHARACTER.
                    psha                          ; SAVE THE CHARACTER TO SEND.
                    bsr       SCIIN               ; YES. GO RESET THE SCI RECEIVER STATUS.
                    cmpa      XOFFCH              ; WAS IT AN XOFF?
                    bne       Done@@              ; NO. SO GO SEND THE CHARACTER.
Loop@@              bsr       SCIIN               ; GO WAIT FOR AN "X-ON" CHARACTER.
                    cmpa      XONCH               ; IS IT AN X-ON CHARACTER?
                    bne       Loop@@              ; NO. GO WAIT FOR AN X-ON CHARACTER.
Done@@              pula                          ; GET THE CHARACTER TO SEND.
Send@@              bra       SCIOUT              ; SEND THE CHARACTER TO THE PRINTER & RETURN.

;                   include   'vectors.mod'
                    title     Config/Reset/Interrupt Vectors
          #if * > $FFA0
                    #Fatal    BASIC is too large
          #endif
                    org       $FFA0

IOVects             dw        ACIAIN              ; Inputs
                    dw        SCIIN
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0

                    dw        ACIAOUT             ; Outputs
                    dw        PROUT
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0

                    org       $FFC0
RAMStart            dw        $C000               ; starting address of system RAM.
RAMSize             dw        $2000               ; size of BASIC11 RAM Buffer.
EEStart             dw        $6000               ; starting address of program storage EEPROM
EESize              dw        $2000               ; size of the program storage EEPROM
IOBase              dw        $1000               ; Base Address of the I/O Registers
TimeVal             dw        62500               ; value used for generating 'Time' Interrupt
UserInit            dw        IODevInit           ; Used to initialize console/other hardware.
DFLOPADR            dw        $4000               ; Address of flip-flop used to connect the HC11 SCI
                                                  ; to the host port connector.
                    org       ROMBEG+ROMSIZE-{2*21}  ; START OF VECTOR TABLE.
                    dw        SCISS               ; SCI SERIAL SYSTEM
                    dw        SPITC               ; SPI TRANSFER COMPLETE
                    dw        PACCIE              ; PULSE ACCUMULATOR INPUT EDGE
                    dw        PACCOVF             ; PULSE ACCUMULATOR OVERFLOW
                    dw        TIMEROVF            ; TIMER OVERFLOW
                    dw        TOC5                ; TIMER OUTPUT COMPARE 5
                    dw        TOC4                ; TIMER OUTPUT COMPARE 4
                    dw        TOC3                ; TIMER OUTPUT COMPARE 3
                    dw        TOC2                ; TIMER OUTPUT COMPARE 2
                    dw        TOC1                ; TIMER OUTPUT COMPARE 1
                    dw        TIC3                ; TIMER INPUT CAPTURE 3
                    dw        TIC2                ; TIMER INPUT CAPTURE 2
                    dw        TIC1                ; TIMER INPUT CAPTURE 1
                    dw        REALTIMI            ; REAL TIME INTERRUPT
                    dw        IRQI                ; IRQ INTERRUPT
                    dw        XIRQ                ; XIRQ INTERRUPT
                    dw        SWII                ; SOFTWARE INTERRUPT
                    dw        ILLOP               ; ILLEGAL OPCODE TRAP
                    dw        COP                 ; WATCH DOG FAIL
                    dw        CMF                 ; CLOCK MONITOR FAIL
                    dw        POWERUP             ; RESET
;                   opt       nol
                    end       MAIN

;*******************************************************************************

?                   macro
          #ifz HC11
                    #temp     8174
                    #temp1    208
                    #temp2    $18D0
            #ifdef BUGFIX
                    #temp     8176
                    #temp2    $544B
            #endif
          #else
                    #temp     8164
                    #temp1    208
                    #temp2    $3FEB
            #ifdef BUGFIX
                    #temp     8166
                    #temp2    $39D0
            #endif
          #endif
                    #Hint     Verification~'..................................................................................................................................'.1.{:width-75}~ {:temp} bytes, RAM:.. {:temp1}, CRC: {:temp2(h)}
                    endm

                    @?
