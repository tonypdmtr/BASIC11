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
; 9. Slowly converting to 100% ASM11 syntax [WORK-IN-PROGRESS]
;
;===============================================================================
;
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
;
;===============================================================================

                    #Macro
                    #SpacesOff
                    #ExtraOn
                    #CaseOn
                    #OptRelOff
                    #OptRtsOff

Include             macro
                    mset      #
                    #Include  ~1~
                    endm

Name                macro
                    mset      #
                    title     ~1~
                    endm

Title               macro
                    mset      #
                    #Message  Processing \@~1~\@
                    endm

Page                macro
                    #Page
                    endm

;*******************************************************************************

BUS_KHZ             def       2000

;*******************************************************************************
;
;                   OPT       NOL

HC11                equ       1

;                   include   'defines.inc'
                    title     DEFINES

          #ifnz HC11

;***** HC11EVB defines *****/

ROMBEG              equ       $E000
ROMSIZE             equ       $2000
ACIAST              equ       $9800
ACIADT              equ       ACIAST+1
DFLOP               equ       $4000
SWPRE               equ       02                  ; SOFTWARE PRESCALER VALUE.

SBASBEG             equ       0
SBASEND             equ       2
SVARBEG             equ       4
SVAREND             equ       6
SHILINE             equ       8
AUTOSTF             equ       10
SSTART              equ       11

          #else

;***** 6809/FLEX development defines *****/

ROMBEG              equ       $8000
ROMSIZE             equ       $2000
RAMBEG              equ       $4000
RAMSIZE             equ       $2000
STACKP              equ       RAMBEG+RAMSIZE-1
SWSTACK             equ       RAMBEG+RAMSIZE-512
ACIAST              equ       $E010
ACIADT              equ       ACIAST+1

                    org       $6000

SEEPROM             equ       *
SBASBEG             rmb       2
SBASEND             rmb       2
SVARBEG             rmb       2
SVAREND             rmb       2
AUTOSTF             rmb       1
SHILINE             rmb       2
SSTART              equ       *

          #endif

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
SPC                 equ       32                  ; space character
MIDEOL              equ       ':'                 ; mid EOL character
COMMA               equ       ','                 ; comma
SEMI                equ       ';'                 ; semicolon
NUM                 equ       1                   ; getvar return flag
STRING              equ       2                   ; getvar return flag
NULL                equ       0                   ; null value
CNTRLC              equ       3                   ; control-c (break character)

IBUFLEN             equ       80                  ; input buffer max length
TBUFLEN             equ       128                 ; token buffer max length
SWSTKSize           equ       592

OPSLEN              equ       30                  ; operator stack length
NUMSLEN             equ       60                  ; operand stack length
FORSLEN             equ       80                  ; FOR..NEXT stack length
WHSLEN              equ       16                  ; WHILE..ENDWH stack length
GOSLEN              equ       16                  ; GOSUB stack length

;***** define error codes *****/

LINRANG             equ       1                   ; line number range error
SYTXERR             equ       2                   ; syntax error
IVEXPERR            equ       3                   ; invalid expression error
UPARNERR            equ       4                   ; unbalanced parentheses error
DTMISERR            equ       5                   ; data type mismatch error
OPRTRERR            equ       6                   ; illegal operator error
ILVARERR            equ       7                   ; illegal variable error
ILTOKERR            equ       8                   ; illegal token error
OMEMERR             equ       9                   ; out of memory error
INTOVERR            equ       10                  ; integer overflow error
IVHEXERR            equ       11                  ; invalid hex digit error
HEXOVERR            equ       12                  ; hex number overflow
MISQUERR            equ       13                  ; missing quote error
MPARNERR            equ       14                  ; missing open or closing parenthisis
IONSYERR            equ       15                  ; "ON" syntax error
MTHENERR            equ       16                  ; missing "THEN" in "IF" statement
MTOERR              equ       17                  ; missing "TO" in "FOR" statement
LINENERR            equ       18                  ; line number error
IDTYERR             equ       19                  ; illegal data type error
EXPCXERR            equ       20                  ; expression too complex (xlator token buff ovf.)
MCOMAERR            equ       21                  ; missing comma
MCMSMERR            equ       22                  ; missing comma or semicolon
MSTKOERR            equ       23                  ; math stack overflow error
UNDIMERR            equ       24                  ; undimentioned array error
SUBORERR            equ       25                  ; subscript out of range error
ZDIVERR             equ       26                  ; divide by zero error
LNFERR              equ       27                  ; line not found error
GOSOVERR            equ       28                  ; too many nested GOSUB's
RWOGERR             equ       29                  ; RETURN w/o GOSUB error
WHSOVERR            equ       30                  ; too many active WHILE's
ENDWHERR            equ       31                  ; ENDWH statement w/o WHILE
ONARGERR            equ       32                  ; ON argument is negative, zero, or too large
NOSUBERR            equ       33                  ; non-subscriptable variable found in DIM statem.
REDIMERR            equ       34                  ; variable has already been DIMensioned
FORNXERR            equ       35                  ; too many active FOR -- NEXT loops active
MFRNXERR            equ       36                  ; mismatched FOR -- NEXT statements.
CNTCNERR            equ       37                  ; can't continue
ODRDERR             equ       38                  ; out of data in read or restore statement
NEGSUBER            equ       39                  ; negative subscripts not allowed
EESUBERR            equ       40                  ; EEP() subscript negative or > 200
PRFUNERR            equ       41                  ; function only allowed in print statement
TABARGER            equ       42                  ; argument <0 or >255 in TAB() function
CHRARGER            equ       43                  ; argument <0 or >255 in CHR$() function
OVDV0ERR            equ       44                  ; overflow or /0 error in FDIV() function
INVCHERR            equ       45                  ; invalid channel number in ADC() function
PRTASERR            equ       46                  ; tried to assign a value <0 or >255 to PORT(X)
ILPRTERR            equ       47                  ; illegal port error
ILLIOERR            equ       48                  ; illegal I/O vector number <0 or >7
UNINIERR            equ       49                  ; uninitalized I/O vector
HEX2AERR            equ       50                  ; argument <0 or >255 in HEX2 function
NOTALERR            equ       51                  ; statement not allowed in direct mode
NOTINTER            equ       52                  ; an RETI statement executed when not in interrupt
PACCARGE            equ       53                  ; tried to assign a value of <0 or >255 to PACC
INTMODER            equ       54                  ; interrupt or count mode error in ONPACC
EETOSMAL            equ       55                  ; program storage EEPROM is Too Small

; *mathematical operator tokens

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

; *keyword tokens

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

; *function tokens

FDIVTOK             equ       $01                 ; FDIV
CHRTOK              equ       $02                 ; CHR$
ADCTOK              equ       $03                 ; ADC
ABSTOK              equ       $04                 ; ABS
RNDTOK              equ       $05                 ; RND
SGNTOK              equ       $06                 ; SGN
TABTOK              equ       $07                 ; TAB
CALLTOK             equ       $08                 ; CALL
PEEKTOK             equ       $09                 ; PEEK
FEEPTOK             equ       $0A                 ; EEP
HEXTOK              equ       $0B                 ; HEX
FPRTATOK            equ       $0C                 ; PORTA
FPRTBTOK            equ       $0D                 ; PORTB
FPRTCTOK            equ       $0E                 ; PORTC
FPRTDTOK            equ       $0F                 ; PORTD
FPRTETOK            equ       $10                 ; PORTE
FTIMETOK            equ       $11                 ; TIME
HEX2TOK             equ       $12                 ; HEX2
FPACCTOK            equ       $13                 ; PACC

; *numerical/variable tokens

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

; *misc. tokens

MSCNTOK             equ       $7F                 ; multiple space count token
SSCNTOK             equ       $7E                 ; single space token
EOLTOK              equ       $7D                 ; end of line token
COMMATOK            equ       $7C                 ; ,
SEMITOK             equ       $7B                 ; ;
MEOLTOK             equ       $7A                 ; :
EQUALTOK            equ       $79                 ; '='
PNUMTOK             equ       $78                 ; '#'

JMPOP               equ       $7E                 ; OP-CODE FOR "JMP" (USED TO INITALIZE INTERRUPT TABLE)

;         *********** define variables ***********

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
IMMID               rmb       1                   ; immidiate mode flag
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
                    #Fatal    "Ran out of Page 0 RAM"
          #endif

                    org       $009E

CONSTAT             rmb       3                   ; GET CONSOLE STATUS FOR BREAK ROUTINE.
INCONNE             rmb       3                   ; GET BYTE DIRECTLY FROM CONSOLE FOR BREAK ROUTINE.

                    org       $00A4

INTABLE             rmb       16                  ; RESERVE SPACE FOR 8 DIFFERENT INPUT ROUTINES.
OUTABLE             rmb       16                  ; RESERVE SPACE FOR 8 DIFFERENT OUTPUT ROUTINES.

                    org       $00C4               ; START OF RAM INTERRUPT VECTORS.

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
                    org       ROMBEG
                    title     BASICLB1
;******************************************************************************
;                                                                            *
;                      MC68HC11 BASIC INTERPRETER                            *
;                                                                            *
;                             WRITTEN BY:                                    *
;                                                                            *
;                           GORDON DOUGHMAN                                  *
;                                                                            *
;                        COPYRIGHT 1985-1990 BY                              *
;                                                                            *
;                           GORDON DOUGHMAN                                  *
;                                                                            *
;******************************************************************************
;
;
;
;       include "1.DEFINES.C"
;
; *main()
; *{
; initvars();            initalize all variables & pointers
; outheader();           send startup message to console
; outrdy();              output ready message

                    jmp       POWERUP

MAIN                equ       *
MAINC               jsr       INITVARS            ; INITALIZE ALL INTERNAL VARIABLES.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; get the auto start flag.
                    cmpa      #$55
                    bne       MAIN9
                    cli                           ; ALLOW ALL INTERRUPTS TO BE SERVICED.
                    jsr       CRUN
MAIN9               jsr       OUTHEADR            ; PRINT HEADER.
MAINW               equ       *
MAIN2               ldd       RAMStart            ; RESET STACK VALUE.
                    addd      RAMSize
                    xgdx
                    txs
                    cli                           ; (J.I.C.)
                    clra                          ; CLEAR D.
                    clrb
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

MAIN1               equ       *
                    clr       IMMID               ; CLEAR THE IMMIDIATE MODE FLAG.
                    clr       ERRCODE             ; CLEAR THE ERROR CODE BYTE.
                    clr       RUNFLAG             ; CLEAR THE RUN MODE FLAG.
                    jsr       OUTPRMPT            ; SEND PROMPT TO CONSOLE.
                    jsr       GETLINE             ; GO GET LINE FROM OPERATOR.
                    jsr       SKIPSPCS            ; SKIP ANY LEADING SPACES.
                    jsr       CHCKCMDS            ; GO CHECK FOR ANY COMMANDS.
                    cpd       #0                  ; WERE THERE ANY?
                    bne       MAIN2               ; YES. CONTINUE.
                    ldx       VAREND              ; SAVE CURRENT END OF VARIABLE AREA IN CASE LINE
                    stx       FENCE               ; HAS AN ERROR IN IT. (SO NO SPURIOUS VARIABLES
                                                  ; ARE ALLOCATED)
                    jsr       PARSE
                    tst       IMMID               ; DID WE EXECUTE IN IMMIDATE MODE?
                    bne       MAIN2               ; YES. PRINT READY MESSAGE.
                    bra       MAIN1               ; NO. JUST PRINT PROMPT.

MAIN3               ldx       FENCE               ; GET THE VAREND FENCE.
                    clr       ,x                  ; MARK "OLD" END OF VARIABLE AREA IN CASE ANY
                                                  ; VARIABLES WERE ALLOCATED.
                    stx       VAREND              ; RESTORE THE POINTER.
                    bra       MAIN2               ; CONTINUE AFTER ERROR.

;        ***** skipspcs() *****
;
; *skipspcs()
; *{
; while(*ibufptr==SPC) ++ibufptr;
; return;
; *}

SKIPSPCS            equ       *
SKIPSPC1            jsr       GETCHR
                    cmpa      #SPC
                    bne       SKIPSPC2
                    jsr       INCIBP
                    bra       SKIPSPC1

SKIPSPC2            rts

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

OUTHEADR            equ       *
                    ldx       #HEADER
                    jmp       PL

HEADER              equ       *
                    fcb       CR,LF,CR,LF
                    fcc       "BASIC11 v1.55",CR,LF
                    fcc       "Copyright 1985-1990 by",CR,LF
                    fcs       "Gordon Doughman",CR,LF

;        ***** outrdy() *****
;
; *outrdy()
; *{
; nl();
; pl("READY");
; return;
; *}

OUTRDY              equ       *
                    ldx       #READY
                    jmp       PL

READY               equ       *
                    fcs       CR,LF,"READY",CR,LF

;        ***** getline() *****

; *getline()
; *{
; short chrcnt;
; char c;
; chrcnt=IBUFLEN;
; ibufptr=inbuff;

GETLINE             equ       *
                    ldb       #IBUFLEN-1
                    ldx       INBUFFS

; while((c=inbyte())!=EOL && (chrcnt>0))
; {

GETLINE3            jsr       INBYTE
                    cmpa      #EOL
                    beq       GETLINE1
                    tstb
                    beq       GETLINE1

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
                    bne       GETLINE2
                    cmpb      #IBUFLEN-1
                    beq       GETLINE1
                    dex
                    lda       #SPC
                    jsr       OUTBYTE
                    lda       #BS
                    jsr       OUTBYTE
                    incb

;  if(c < SPC) continue;  ignore all other control characters

GETLINE2            equ       *
                    cmpa      #CNTRLC             ; IS IT A "BREAK"?
                    bne       GETLINE4            ; NO. GO PROCESS THE CHARACTER.
                    inc       CONTFLAG            ; DONT ALLOW A "CONT".
                    jmp       CHCKBRK2            ; GO DO A BREAK.

GETLINE4            cmpa      #SPC
                    blo       GETLINE3

;  *ibufptr=c;            save character in input buffer
;  ibufptr+=1;           point to next location
;  chrcnt-=1;
; }
                    sta       ,x
                    inx
                    decb
                    bne       GETLINE3

; *ibufptr=EOL;          put EOL in input buffer
; ibufptr=inbuff;        initalize the input buffer pointer
; return;
; *}

GETLINE1            equ       *
                    lda       #EOL
                    sta       ,x
                    ldx       INBUFFS
                    stx       IBUFPTR
                    rts

ToUpper             equ       *
                    cmpa      #'a'                ; is the character less than a lower case 'a'?
                    blo       ToUpper1
                    cmpa      #'z'
                    bhi       ToUpper1
                    anda      #$df
ToUpper1            rts

;        ***** outprmpt() *****

; *outprmpt()
; *{
; nl();          go to new line
; outbyte('#');  send prompt to console
; return;
; *}

OUTPRMPT            equ       *
                    ldx       #PRMPTSTR
                    bra       PL

PRMPTSTR            equ       *
                    fcs       CR,LF,"#"

;        ***** nl() *****

; nl()                    send CR/LF to console
; *{
; outbyte(CR);
; outbyte(LF);
; return;
; *}

NL2                 bsr       NL
NL                  equ       *
                    ldx       #CRLFSTR
                    bsr       PL
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    rts

;

CRLFSTR             fcb       $0A,$0D,$00

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

PL                  equ       *
PL1                 lda       ,x
                    beq       PL2
                    jsr       OUTBYTE
                    inx
                    bra       PL1

PL2                 rts

;        ***** parse() *****

; *parse()
; *{
; int num;
; tbufptr=tknbuf;                initalize the token buffer pointer

PARSE               equ       *
                    ldx       TKNBUFS             ; Get the start of the token buffer
                    stx       TBUFPTR

; if(num=getlinum())             get line number if present
; {
;  if(*ibufptr==EOL)             was line # followed by CR?
;  {                             yes.
;   delline(num);                go delete the line from the prog buffer
;   return;
;  }
;  immid=0;                      flag as not immidiate
;  putlinum(num);                put line number in buffer
; }
;  else immid=1;                  if no line # flag as immidiate*/

                    bsr       GETLINUM
                    bcc       PARSE2
                    psha
                    jsr       GETCHR
                    cmpa      #EOL
                    pula
                    bne       PARSE1
                    jsr       DELLINE
                    ldx       VAREND
                    inx
                    stx       STRASTG
PARSE7              rts

PARSE1              clr       IMMID
PARSE8              bsr       PUTLINUM
                    bra       PARSE3

PARSE2              jsr       GETCHR
                    cmpa      #EOL
                    beq       PARSE7
                    lda       #1
                    sta       IMMID
                    ldd       #0
                    bra       PARSE8

; if(errcode) return;             if line number error, return
; xlate();                        if translation error, return
; if(errcode) return;
; if(immid) runline();            if immidiate mode run 1 line
;  else storlin();                if not store pgm line
; return;                         go get next line
; *}

PARSE3              jsr       XLATE
                    tst       IMMID
                    beq       PARSE5
                    jmp       RUNLINE             ; GO RUN THE LINE & RETURN.

PARSE5              jsr       STORLIN             ; GO STORE LINE & RETURN.
                    ldx       VAREND
                    inx
                    stx       STRASTG
                    rts

;        ***** getlinum *****

; *getlinum()
; *{
; int num;
; num=0;

GETLINUM            equ       *

                    pshy
                    clra
                    psha
                    psha
                    tsy

; if(numeric(*ibufptr)==0) return(0);    if 1st char not numeric, rtn 0

                    ldx       IBUFPTR
                    lda       ,x
                    bsr       NUMERIC
                    bcc       GTLNUM4

; while(numeric(*ibufptr))       while *ibufptr is numeric
; {
;  num=num*10+(*ibufptr-'0');    get a digit
;  ibufptr++;                    advance input buffer pointer
;  if(num<=0) { errcode=LINRANG; return(0); }
; }
; return(num);
; *}

GTLNUM2             lda       ,x
                    bsr       NUMERIC
                    bcs       GTLNUM3
                    sec
GTLNUM1             ldd       ,y
                    bne       GTLNUM4
                    lda       #LINENERR
                    bra       GTLNUM5

GTLNUM4             ins
                    ins
                    puly
                    stx       IBUFPTR
                    rts

GTLNUM3             bsr       ADDDIG
                    bpl       GTLNUM2
                    lda       #LINRANG
GTLNUM5             jmp       RPTERR

ADDDIG              equ       *
                    ldd       ,y
                    asld
                    asld
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

;        ***** putlinum *****

; *putlinum(lnum)
; *int lnum;
; *{
; putint(lnum);          put line # in token buffer
; *tbufptr++=0;          hold place for length of line
; return;
; *}

PUTLINUM            equ       *
                    jsr       PUTDTOK
                    clra
                    jmp       PUTTOK

;        ***** numeric() *****

; *numeric(c)
; *char c;
; *{
; c=c&127;
; return((c>='0')&(c<='9'));
; *}

NUMERIC             equ       *
                    cmpa      #'0'
                    blo       NUMERIC1
                    cmpa      #'9'
                    bhi       NUMERIC1
                    sec
                    rts

NUMERIC1            clc
                    rts

;        ***** alpha() *****

; *alpha(c)
; *char c;
; *{
; c=c&127;
; return((c>='A')&(c<='Z'));
; *}

ALPHA               equ       *
                    cmpa      #'A'
                    blo       ALPHA1
                    cmpa      #'Z'
                    bls       ALPHA2
                    cmpa      #'a'
                    blo       ALPHA1
                    cmpa      #'z'
                    bhi       ALPHA1
ALPHA2              sec
                    rts

ALPHA1              clc
                    rts

;        ***** alphanum *****

; *alphanum(c)
; *char c;
; *{ return ((alpha(c)) | (numeric(c))); }

ALPHANUM            equ       *
                    bsr       ALPHA
                    bcc       ALPHANU1
                    rts

ALPHANU1            bra       NUMERIC

;*****************************************
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

XLATE               equ       *
                    jsr       GETCHR              ; GET NEXT CHAR.
                    cmpa      #EOL                ; AT THE END OF THE LINE?
                    beq       XLATE1              ; YES.
                    clr       IFWHFLAG            ; NOT XLATING "IF" OR "WHILE"
                    jsr       BLANKS              ; SKIP BLANKS.
                    ldx       #KEYWORDS           ; POINT TO KEYWORD TABLE.
XLATE4              jsr       STREQ               ; IS KEYWORD IS IN THE INPUT BUFFER?
                    bcs       XLATE2              ; YES GO PROCESS IT.
XLATE3              inx                           ; NO. POINT TO NEXT CHAR.
                    lda       ,x                  ; AT THE END OF THIS KEYWORD?
                    bne       XLATE3              ; NO.
                    ldb       #4                  ; NUMBER OF BYTES TO SKIP.
                    abx
                    tst       ,x                  ; AT THE END OF THE TABLE?
                    bne       XLATE4              ; NO. CHCK FOR NEXT KEYWORD.
                    lda       #IMLETTOK           ; ASSUME AN IMPLIED LET.
;                   jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    ldx       #XIMPLET            ; GET ADDR OF XLATION ROUTINE.
;                   jsr       ,x                  ; GO DO IT.
;                   bra       XLATE6              ; GO FINISH UP.
                    bra       XLATE9

XLATE2              lda       1,x                 ; GET KEYWORD TOKEN.
                    ldx       2,x                 ; GET ADDR OF XLATION ROUTINE.
XLATE9              jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    cmpa      #DATATOK            ; SPECIAL CASE, DONT SKIP BLANKS AFTER KEYWORD.
                    beq       XLATE5
                    cmpa      #REMTOK             ; SAME SPECIAL CASE AS FOR DATA.
                    beq       XLATE5
                    jsr       BLANKS              ; SKIP BLANKS BETWEEN KEYWORD & NEXT OBJECT.
XLATE5              jsr       ,x                  ; GO DO IT.
XLATE6              jsr       BLANKS              ; SKIP BLANKS.
                    jsr       GETNXCHR            ; GET NEXT CHAR.
                    cmpa      #MIDEOL             ; IS IT A MID EOL?
                    bne       XLATE7              ; NO. CHCK FOR EOL.
                    lda       #MEOLTOK            ; GET MID EOL TOKEN.
                    jsr       PUTTOK              ; PUT IT IN BUFFER.
                    bra       XLATE               ; CONTINUE.

XLATE7              cmpa      #EOL                ; EOL?
                    beq       XLATE1              ; YES. FINISH UP.
                    lda       #SYTXERR            ; NO. SYNTAX ERROR.
                    jmp       RPTERR              ; REPORT XLATION ERROR.

XLATE1              lda       #EOLTOK             ; GET EOL TOKEN.
                    jsr       PUTTOK              ; PUT IT IN BUFFER.
                    ldd       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    subd      TKNBUFS             ; Compute the TOKEN BUFFER LENGTH.
                    ldx       TKNBUFS             ; POINT TO BUFFER.
                    stb       2,x                 ; STORE LENGTH.
                    rts                           ; RETURN.

;        KEYWORD LOOK UP TABLE

KEYWORDS            equ       *
DATA                fcs       "DATA"
                    fcb       DATATOK
                    fdb       XDATA
LET                 fcs       "LET"
                    fcb       LETTOK
                    fdb       XLET
READ                fcs       "READ"
                    fcb       READTOK
                    fdb       XREAD
RESTORE             fcs       "RESTORE"
                    fcb       RESTRTOK
                    fdb       XRESTORE
GOSUB               fcs       "GOSUB"
                    fcb       GOSUBTOK
                    fdb       XGOSUB
GOTO                fcs       "GOTO"
                    fcb       GOTOTOK
                    fdb       XGOTO
ONTIME              fcs       "ONTIME"
                    fcb       ONTIMTOK
                    fdb       XONTIME
ONIRQ               fcs       "ONIRQ"
                    fcb       ONIRQTOK
                    fdb       XONIRQ
ONPACC              fcs       "ONPACC"
                    fcb       ONPACTOK
                    fdb       XONPACC
ON                  fcs       "ON"
                    fcb       ONTOK
                    fdb       XON
RETURN              fcs       "RETURN"
                    fcb       RETNTOK
                    fdb       XRETURN
IIF                 fcs       "IF"
                    fcb       IFTOK
                    fdb       XIF
INPUT               fcs       "INPUT"
                    fcb       INPUTTOK
                    fdb       XINPUT
PRINT               fcs       "PRINT"
                    fcb       PRINTTOK
                    fdb       XPRINT
                    fcs       "?"
                    fcb       PRINTTOK
                    fdb       XPRINT
FOR                 fcs       "FOR"
                    fcb       FORTOK
                    fdb       XFOR
NEXT                fcs       "NEXT"
                    fcb       NEXTTOK
                    fdb       XNEXT
STOPSS              fcs       "STOP"
                    fcb       STOPTOK
                    fdb       XSTOP
ENDWH               fcs       "ENDWH"
                    fcb       ENDWHTOK
                    fdb       XENDWH
ENDS                fcs       "END"
                    fcb       ENDTOK
                    fdb       XEND
REM                 fcs       "REM"
                    fcb       REMTOK
                    fdb       XREM
TRON                fcs       "TRON"
                    fcb       TRONTOK
                    fdb       XTRON
TROFF               fcs       "TROFF"
                    fcb       TROFFTOK
                    fdb       XTROFF
WHILE               fcs       "WHILE"
                    fcb       WHILETOK
                    fdb       XWHILE
POKE                fcs       "POKE"
                    fcb       POKETOK
                    fdb       XPOKE
DIM                 fcs       "DIM"
                    fcb       DIMTOK
                    fdb       XDIM
EEP                 fcs       "EEP"
                    fcb       EEPTOK
                    fdb       XEEP
PORTA               fcs       "PORTA"
                    fcb       PORTATOK
                    fdb       XPORTA
PORTB               fcs       "PORTB"
                    fcb       PORTBTOK
                    fdb       XPORTB
PORTC               fcs       "PORTC"
                    fcb       PORTCTOK
                    fdb       XPORTC
PORTD               fcs       "PORTD"
                    fcb       PORTDTOK
                    fdb       XPORTD
INBYTES             fcs       "INBYTE"
                    fcb       INBYTTOK
                    fdb       XINBYTE
TIME                fcs       "TIME"
                    fcb       TIMETOK
                    fdb       XTIME
RETI                fcs       "RETI"
                    fcb       RETITOK
                    fdb       XRETI
PACC                fcs       "PACC"
                    fcb       PACCTOK
                    fdb       XPACC
SLEEP               fcs       "SLEEP"
                    fcb       SLEEPTOK
                    fdb       XSLEEP
RTIMES              fcs       "RTIME"
                    fcb       RTIMETOK
                    fdb       XRTIME

                    fcb       0                   ; END OF TABLE MARKER.

; *blanks()
; *{
; short spcnt;
; spcnt=0;
; while(*ibufptr==SPC) { ibufptr++; spcnt++; }

BLANKS              equ       *
                    pshx
                    ldx       IBUFPTR
                    clrb
BLANKS1             lda       ,x
                    cmpa      #SPC
                    bne       BLANKS2
                    incb
                    inx
                    bra       BLANKS1

; if(spcnt==0) return;

BLANKS2             tstb
                    bne       BLANKS3
                    pulx
                    rts

; if(spcnt>1)
;  {
;   *tbufptr++=MSCNTOK;
;   *tbufptr++=spcnt;
;  }

BLANKS3             stx       IBUFPTR
                    ldx       TBUFPTR
                    cmpb      #1
                    beq       BLANKS4
                    lda       #MSCNTOK
                    sta       ,x
                    inx
BLANKS5             stb       ,x
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

BLANKS4             ldb       #SSCNTOK
                    bra       BLANKS5

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

STREQ               equ       *
                    ldd       IBUFPTR             ; SAVE VALUE OF POINTER.
                    pshd
STREQU4             lda       ,x
                    beq       STREQU2
STREQU1             bsr       GETNXCHR
                    jsr       ToUpper             ; Make the character upper case.
                    cmpa      ,x
                    beq       STREQU3
                    puld
                    std       IBUFPTR
                    clc
                    rts

STREQU3             inx
                    bra       STREQU4

STREQU2             puld
                    sec
                    rts

;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER.

GETCHR              pshx                          ; SAVE THE X REGISTER.
                    ldx       IBUFPTR             ; GET POINTER.
                    lda       ,x                  ; GET A CHARACTER.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER
;        AND ADVANCES THE POINTER TO POINT TO THE NEXT CHARACTER.

GETNXCHR            bsr       GETCHR

          ; FALL THROUGH TO INCIBP.

;        THIS ROUTINE JUST INCREMENTS THE INPUT BUFFER POINTER.

INCIBP              pshx                          ; SAVE X.
                    ldx       IBUFPTR             ; GET POINTER.
INCIBP1             inx                           ; ADVANCE POINTER.
                    stx       IBUFPTR             ; UPDATE POINTER.
INCIBP2             pulx                          ; RESTORE X
                    rts                           ; RETURN.

;        THIS ROUTINE PUTS THE WORD IN THE D-REG. INTO THE TOKEN BUFFER
;        AND ADVANCES THE TOKEN BUFFER POINTER.

PUTDTOK             bsr       PUTTOK              ; PUT THE FIRST BYTE INTO THE TOKEN BUFFER.
                    tba                           ; PUT THE 2ND BYTE INTO A.

          ; FALL THROUGH TO PUTTOK.

;        THIS ROUTINE PUTS THE CHARACTER IN THE A-REG. INTO THE TOKEN
;        BUFFER AND ADVANCES THE TOKEN BUFFER POINTER.

PUTTOK              pshx                          ; SAVE X.
                    pshd                          ; (9/12/89).
                    ldx       TBUFPTR             ; GET POINTER.
                    sta       ,x                  ; PUT CHARACTER.
PUTTOK1             inx                           ; ADVANCE POINTER.
                    stx       TBUFPTR             ; SAVE POINTER.

                    ldd       TKNBUFS             ; get the starting address of the token buffer.
                    addd      #TBUFLEN            ; add the length of the buffer to it.
                    cpd       TBUFPTR             ; IS THE TOKEN BUFFER FULL?
                    puld                          ; (9/12/89). restore the b reg.
                    bhi       INCIBP2             ; NO. RESTORE X AND RETURN.
                    lda       #EXPCXERR           ; YES. FLAG THE ERROR.
                    jmp       RPTERR              ; GO REPORT IT.
;                   include   'basiclb2.mod'
                    title     BASICLB2
                    page

; *<><><><><><> NOTE: FUNCTION PERFORMED IN "XLATE" <><><><><><>
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

XGOSUB              equ       *
XGOTO               equ       *
;                   jsr       BLANKS
                    lda       #LCONTOK
                    bsr       PUTTOK
                    jsr       GETLINUM
XGOTO2              bra       PUTDTOK

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

XRETURN             equ       *
XSTOP               equ       *
XEND                equ       *
XTRON               equ       *
XTROFF              equ       *
XRESTORE            equ       *
XENDWH              equ       *
XRETI               equ       *
XSLEEP              equ       *
XRTIME              equ       *
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

XDATA               equ       *
XREM                equ       *
                    ldx       TBUFPTR             ; GET POINTER TO TOKEN BUFFER.
                    pshx                          ; SAVE IT. (POINTER TO LENGTH OF REM OR DATA)
                    lda       #0                  ; SAVE A BYTE FOR THE LENGTH.
                    bsr       PUTTOK
                    ldb       #2                  ; INITALIZE LENGTH TO 2 (INCLUDES LENGTH & EOL.
XREM1               bsr       GETCHR
                    cmpa      #EOL
                    beq       XREM2
                    bsr       PUTTOK
                    bsr       INCIBP
                    incb                          ; UP THE BYTE COUNT.
                    bra       XREM1

XREM2               bsr       PUTTOK
                    pulx                          ; GET POINTER TO LENGTH BYTE.
                    stb       ,x                  ; PUT IT IN THE TOKEN BUFFER.
                    rts

XPORTA              equ       *
XPORTB              equ       *
XPORTC              equ       *
XPORTD              equ       *
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

XLET                equ       *
XIMPLET             equ       *
;                   jsr       BLANKS
;XLET1              jmp       ASIGNMT

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

ASIGNMT             equ       *
                    jsr       GETVAR
                    tab
ASIGNMT1            bsr       GETNXCHR
                    cmpa      #'='
                    beq       ASIGNMT2
                    lda       #IVEXPERR
                    jmp       RPTERR

ASIGNMT2            lda       #EQUALTOK
                    bsr       PUTTOK
                    tba

          ; FALL THROUGH TO XEXPRES.

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

XEXPRES             equ       *
                    pshy
                    psha
                    tsy
XEXPRS29            ldx       #UINARYOP
                    jsr       TBLSRCH
                    bcc       XEXPRS30
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

XEXPRS30            jsr       GETCHR
                    cmpa      #'('
                    bne       XEXPRS1
                    jsr       INCIBP
                    lda       #OPARNTOK
                    jsr       PUTTOK
                    lda       ,y
                    jsr       XEXPRES
XEXPRS2             jsr       GETNXCHR
                    cmpa      #')'
                    beq       XEXPRS3
                    lda       #UPARNERR
                    jmp       RPTERR

XEXPRS3             lda       #CPARNTOK
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

XEXPRS1             equ       *
                    jsr       NUMERIC
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
                    beq       XEXPRS9
                    lda       #DTMISERR
                    jmp       RPTERR

XEXPRS9             equ       *

; *now look for operator or end of expression
;
;  chkoprtr:
;  c=*ibufptr;
;  if(c==EOL | c==MIDEOL | c==SPC | c==COMMA | c==SEMI | c==')')
;  {
;   return(c);
;  }

CHKOPRTR            equ       *
                    jsr       GETCHR
                    cmpa      #EOL
                    beq       XEXPRS24
                    cmpa      #MIDEOL
                    beq       XEXPRS24
                    cmpa      #SPC
                    beq       XEXPRS24
                    cmpa      #COMMA
                    beq       XEXPRS24
                    cmpa      #SEMI
                    beq       XEXPRS24
                    cmpa      #')'
                    beq       XEXPRS24

;  if(type==NUM)
;  {
;   if(c=cknumop()) ;
;   else if(c=ckbolop()) ;
;   else if(ifwhflag) c=cklogop();
;   else c=NULL;
;  }

XEXPRS15            equ       *
                    lda       ,y
                    cmpa      #NUM
                    bne       XEXPRS21
                    jsr       CKNUMOP
                    bcs       XEXPRS17
                    jsr       CKBOLOP
                    bcs       XEXPRS17
                    tst       IFWHFLAG
                    beq       XEXPRS18
                    jsr       CKLOGOP
                    bra       XEXPRS17

XEXPRS18            lda       #NULL
                    bra       XEXPRS17

;  else { errcode=IDTYERR; return; }

XEXPRS21            equ       *
                    lda       #IDTYERR
                    jmp       RPTERR

;  if(c==NULL) { errcode=OPRTRERR; return; }
;  *tbufptr++=c;
; }
; return;
; *}

XEXPRS17            equ       *
                    tsta
                    bne       XEXPRS23
                    lda       #OPRTRERR
                    jmp       RPTERR

XEXPRS24            ins
                    puly
                    rts

XEXPRS23            jsr       PUTTOK
                    jmp       XEXPRS29

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

CKNUMOP             equ       *
                    ldx       #NUMOPTBL

CKOP                jsr       TBLSRCH
                    bcs       CKOP1
                    lda       #NULL
CKOP1               rts

;***** ckbolop() *****/
;
; *ckbolop()
; *{
; if(match("AND")) return(ANDTOK);
; else if(match("OR")) return(ORTOK);
; else if(match("EOR")) return(EORTOK);
; else return(NULL);
; *}

CKBOLOP             equ       *
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

CKLOGOP             equ       *
                    ldx       #LOGOPTBL
                    bra       CKOP

; *<><><><><> NOTE: THIS ROUTINE HAS NO 'C' COUNTER PART <><><><><><>

TBLSRCH             equ       *
                    jsr       STREQ               ; SEARCH FOR STRING.
                    bcs       TBLSRCH1            ; IF FOUND GO GET TOKEN & RETURN.
TBLSRCH2            inx                           ; BUMP POINTER TO NEXT CHAR.
                    lda       ,x                  ; GET IT.
                    bne       TBLSRCH2            ; KEEP LOOKING FOR END OF ENTRY.
                    inx                           ; FOUND IT. BUMP POINTER TO NEXT ENTRY.
                    inx
                    lda       ,x                  ; AT THE END OF THE TABLE?
                    bne       TBLSRCH             ; NO. GO CHECK THE NEXT ENTRY.
                    clc                           ; YES. FLAG AS NOT FOUND.
                    rts                           ; RETURN.

TBLSRCH1            lda       1,x                 ; GET TOKEN.
                    sec                           ; FLAG AS FOUND.
                    rts                           ; RETURN.

NUMOPTBL            equ       *
PLUS                fcs       "+"
                    fcb       PLUSTOK
MINUS               fcs       "-"
                    fcb       MINUSTOK
MULT                fcs       "*"
                    fcb       MULTTOK
DIV                 fcs       "/"
                    fcb       DIVTOK
MODS                fcs       "\"
                    fcb       MODTOK

                    fcb       0                   ; END OF TABLE FLAG.

BOLOPTBL            equ       *
ANDS                fcs       ".AND."
                    fcb       ANDTOK
ORS                 fcs       ".OR."
                    fcb       ORTOK
EORS                fcs       ".EOR."
                    fcb       EORTOK

                    fcb       0                   ; END OF TABLE FLAG.

LOGOPTBL            equ       *
LTEQ                fcs       "<="
                    fcb       LTEQTOK
GTEQ                fcs       ">="
                    fcb       GTEQTOK
NOTEQ               fcs       "<>"
                    fcb       NOTEQTOK
LT                  fcs       "<"
                    fcb       LTTOK
GT                  fcs       ">"
                    fcb       GTTOK
EQ                  fcs       "="
                    fcb       EQTOK

                    fcb       0                   ; END OF TABLE FLAG.

UINARYOP            equ       *
NEGS                fcs       "-"
                    fcb       NEGTOK
NOTS                fcs       "NOT"
                    fcb       NOTTOK

                    fcb       0                   ; END OF TABLE MARKER.
;                   include   'basiclb3.mod'
                    title     BASICLB3
                    page

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

GETVAR              equ       *
                    pshy
                    clra
                    psha
                    psha
                    psha
                    psha
                    tsy
                    jsr       GETCHR
                    jsr       ALPHA
                    bcs       GETVAR1
                    lda       #ILVARERR
                    jmp       RPTERR

GETVAR1             jsr       ToUpper
                    sta       ,y
                    jsr       INCIBP

; if(alphanum(*ibufptr)) { varname[1]=*ibufptr++; }
; if((vartype=chcktyp())==0) { vartype=FVARTOK; }
;  else { ++ibufptr; }

                    jsr       GETCHR
                    jsr       ALPHANUM
                    bcc       GETVAR2
                    jsr       ToUpper
                    sta       1,Y
                    jsr       INCIBP
GETVAR2             jsr       CHCKTYP
                    sta       3,Y

; if((offset=findvar(vartype,varname))==-1) is var already in table?
; {
;  if(errcode) return;
;  if((offset=putvar(vartype,varname))==-1) return;  no. put it there
; }
; if(errcode) return;

                    jsr       FINDVAR
                    cpd       #-1
                    bne       GETVAR5
GETVAR4             lda       3,Y
                    jsr       PUTVAR

; *tbufptr++=vartype;    put variable type byte in token buffer
; putint(offset);        put offset after it
; if((vartype==IVARTOK) | (vartype==FVARTOK)) return(NUM);
; return(STRING);
; *}

GETVAR5             equ       *
                    pshd
                    lda       3,Y
                    jsr       PUTTOK
                    puld
                    jsr       PUTDTOK
                    lda       3,Y                 ; GET VARIABLE TYPE AGAIN.
                    bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       GETVAR7             ; NO. CONTINUE.
                    jsr       INCIBP              ; MOVE THE INPUT BUFFER POINTER PAST THE OPEN (.
                    lda       #OPARNTOK
                    jsr       PUTTOK
                    lda       #NUM                ; YES. SUBSCRIPT EXPRESSION MUST BE NUMERIC.
                    jsr       XEXPRES             ; GO GET THE SUBSCRIPT.
                    jsr       GETNXCHR            ; GET THE TERMINATING CHARACTER.
                    cmpa      #')'                ; IS IT A CLOSING PAREN?
                    beq       GETVAR8             ; YES. GO FINISH UP.
                    lda       #MPARNERR           ; NO. ERROR.
                    jmp       RPTERR

GETVAR8             lda       #CPARNTOK           ; GET CLOSING PAREN TOKEN.
                    jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
GETVAR7             lda       #NUM                ; NO. RETURN PROPER TYPE.
                    ldb       3,Y
                    bitb      #2
                    beq       GETVAR6
                    lda       #STRING
GETVAR6             ins
                    ins
                    ins
                    ins
                    puly
                    rts

;***** chcktype *****/
;
; *chcktyp()
; *{
; if(*ibufptr=='%') return(IVARTOK);
;  else if(*ibufptr=='$') return(SVARTOK);
;  else return(0);
; *}

CHCKTYP             equ       *
                    lda       #IVARTOK            ; IN V1.0 ONLY INTEGER VARIABLES ARE SUPPORTED.
                    psha                          ; IN V2.0 FLOATING POINT VARIABLES WILL BE
                    jsr       GETCHR              ; SUPPORTED.
                    cmpa      #'('                ; IS A SUBSCRIPT FOLLOWING THE NAME?
                    pula                          ; RESTORE THE TOKEN TYPE.
                    bne       CHCKTYP4            ; NO. RETURN.
                    adda      #$10                ; YES. MAKE IT AN ARRAY VARIABLE.
CHCKTYP4            rts                           ; RETURN.

;***** findvar *****/
;
; *findvar(vartype,varname)
; *short vartype;
; *char *varname;
; *{
; char *varptr;
; varptr=varbegin;               point to the start of the var table
; while(*varptr)                 we're not to the end of the table

FINDVAR             equ       *
                    ldx       VARBEGIN
FINDVAR1            tst       ,x
                    beq       FINDVAR2

; {
;  if(*varptr==vartype)          is the current var the same type?
;  {                             yes.
;   if(streq(varptr+1,varname))  is the name the same?
;   {                            yes.
;    return(varptr-varbegin);    return the offset from the table start
;   }
;  }

                    cmpa      ,x
                    bne       FINDVAR3
                    ldb       1,x
                    cmpb      ,y
                    bne       FINDVAR3
                    ldb       2,x
                    cmpb      1,Y
                    bne       FINDVAR3
                    xgdx
                    subd      VARBEGIN
                    rts

;  if not, advance to the next variable in the table
;  if(*varptr==IVARTOK) varptr=varptr+ISIZ+3;
;  else if(*varptr==SVARTOK) varptr=varptr+SSIZ+3;
;  else if(*varptr==FVARTOK) varptr=varptr+FSIZ+3;
;  else { errcode=ILTOKERR; return(-1); }
; }

FINDVAR3            equ       *
                    ldb       ,x
                    bitb      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       FINDVAR8            ; NO CONTINUE.
                    ldb       #ASIZ+3             ; YES. GET ARRAY SIZE +3.
                    bra       FINDVAR7

FINDVAR8            cmpb      #IVARTOK
                    bne       FINDVAR6
                    ldb       #ISIZ+3
FINDVAR7            abx
                    bra       FINDVAR1

FINDVAR6            lda       #ILTOKERR
                    jmp       RPTERR

FINDVAR2            ldd       #-1
                    rts

; return(-1);
; *}
;
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

PUTVAR              equ       *
                    ldx       VAREND
                    pshx
                    sta       ,x
                    inx
                    ldb       ,y
                    stb       ,x
                    inx
                    ldb       1,Y
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
                    bls       PUTVAR5
                    lda       #OMEMERR
                    bra       CLRVAR6

PUTVAR5             equ       *
                    puld
                    subd      VARBEGIN
                    pshd                          ; SAVE THE OFFSET TO THIS VARIABLE.
                    jsr       CCLEAR3             ; CLEAR ALL VARIABLES SINCE WE MAY HAVE TRASHED
                                                  ; ANY ARRAYS THAT HAD BEEN ALLOCATED.
                    puld                          ; RESTORE THE "NEW" VARIABLE OFFSET.
                    rts

CLRVAR              equ       *
                    bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       CLRVAR8             ; NO. CONTINUE.
                    ldb       #ASIZ               ; YES. GET THE DICTIONARY SIZE+1.
                    bra       CLRVAR1             ; PUT THE VARIABLE IN THE DICTIONARY.

CLRVAR8             cmpa      #IVARTOK
                    bne       CLRVAR4
                    ldb       #ISIZ
CLRVAR1             equ       *
                    clr       ,x
                    inx
                    decb
                    bne       CLRVAR1
                    rts

CLRVAR4             lda       #ILTOKERR
CLRVAR6             jmp       RPTERR

;***** getcon() *****/
;
; *getcon()
; *{
; int const;
; char *litp;
; short count;
; litp=ibufptr;          save a pointer to start of constant
; if(*ibufptr=='"') { getscon(); return(STRING); } if " get strng

GETCON              equ       *
                    jsr       GETCHR

;  else if(*ibufptr=='$') { ++ibufptr; const=gethex(); } if '$' get hex
;  else const=getdeci();         else assume its a decimal constant
; if(errcode) return(0);         if error abort

GETCON2             equ       *
                    ldx       IBUFPTR
                    pshx
                    cmpa      #'$'
                    bne       GETCON3
                    jsr       INCIBP
                    jsr       GETHEX
                    bra       GETCON4

GETCON3             jsr       GETDECI

; *tbufptr++=ICONTOK;            put integer constant token in buffer
; putint(const);                 follow it with the constant
; count=ibufptr-litp;    get number of bytes in source form of const.
; *tbufptr++=count;       put it in the token buffer
; while(litp < ibufptr) *tbufptr++=*litp++; copy source form into buffer
; return(NUM);           return the constant type
; }

GETCON4             equ       *
                    psha
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
GETCON5             lda       ,x
                    jsr       PUTTOK
                    inx
                    decb
                    bne       GETCON5
                    lda       #NUM
                    rts

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

GETDECI             equ       *
                    pshy
                    clra
                    psha
                    psha
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    jsr       NUMERIC
                    bcs       GETDECI1
                    lda       #SYTXERR
                    bra       CHCKERR

GETDECI1            lda       ,x
                    jsr       NUMERIC
                    bcc       GETDECI3
                    jsr       ADDDIG
                    bpl       GETDECI1
                    lda       #INTOVERR
                    bra       CHCKERR

GETDECI3            stx       IBUFPTR
                    ldd       ,y
                    ins
                    ins
                    puly
                    rts

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

GETHEX              equ       *
                    pshy
                    clra
                    psha
                    psha
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    jsr       HEXDIG
                    bcs       GETHEX1
                    lda       #IVHEXERR
CHCKERR             tst       RUNFLAG
                    beq       GETHEX5
                    jmp       RPTRERR

GETHEX5             jmp       RPTERR

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

GETHEX1             equ       *
                    lda       ,x
                    jsr       HEXDIG
                    bcc       GETDECI3
                    ldd       ,y
                    lsld
                    bcs       GETHEX3
                    lsld
                    bcs       GETHEX3
                    lsld
                    bcs       GETHEX3
                    lsld
                    bcs       GETHEX3
                    std       ,y
                    lda       ,x
                    jsr       ToUpper
                    tab
                    inx
                    subb      #'0'
                    cmpb      #9
                    bls       GETHEX4
                    subb      #7
GETHEX4             clra
                    addd      ,y
                    std       ,y
                    bra       GETHEX1

GETHEX3             lda       #HEXOVERR
                    bra       CHCKERR

;***** hexdig() *****/
;
; *hexdig(c)
; *char c;
; *{
; return(numeric(c) | (c>='A' & c<='F')); return true if c is hex
; *}

HEXDIG              equ       *
                    jsr       NUMERIC
                    bcc       HEXDIG1
                    rts

HEXDIG1             jsr       ToUpper
                    cmpa      #'A'
                    blo       HEXDIG2
                    cmpa      #'F'
                    bhi       HEXDIG2
                    sec
                    rts

HEXDIG2             clc
                    rts

;***** getscon *****/
;
; *getscon()
; *{
; short count;
; char *bufptr,c;
; count=2;       initalize byte count to 2
; *tbufptr++=SCONTOK;   put string constant token in buffer
; bufptr=tbufptr++;   save value of tbufptr, advance to next byte,
;                     and reserve a byte for string length
; *tbufptr++=*ibufptr++;   put 1st quote in token buffer

GETSCON             equ       *
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

GETSCON1            equ       *
                    jsr       GETNXCHR
                    cmpa      #'"'
                    beq       GETSCON2
                    cmpa      #EOL
                    bne       GETSCON3
                    lda       #MISQUERR
                    jmp       RPTERR

GETSCON3            jsr       PUTTOK
                    incb
                    bra       GETSCON1

; *tbufptr++=c;          put closing quote in token buffer
; ++ibufptr;             advance input buffer pointer
; *bufptr=count;         put string byte count in token buffer
; return;
; *}

GETSCON2            equ       *
                    jsr       PUTTOK
GETSCON4            pulx
                    stb       ,x
                    rts
;                   include   'basiclb5.mod'
                    title     BASICLB5
                    page

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

GETFUN              equ       *
                    ldx       #FUNCTBL
GETFUN1             jsr       STREQ
                    bcs       GETFUN2
GETFUN3             inx
                    lda       ,x
                    bne       GETFUN3
                    ldb       #4
                    abx
                    tst       ,x
                    bne       GETFUN1
                    clra
                    rts

GETFUN2             lda       #FUNCTFLG
                    jsr       PUTTOK
                    lda       1,x
                    ldx       2,x
                    jmp       ,x

FUNCTBL             equ       *
FDIVS               fcs       "FDIV"
                    fcb       FDIVTOK
                    fdb       BNUMFUN
CHRS                fcs       "CHR$"
                    fcb       CHRTOK
                    fdb       UNUMFUN
ABS                 fcs       "ABS"
                    fcb       ABSTOK
                    fdb       UNUMFUN
RND                 fcs       "RND"
                    fcb       RNDTOK
                    fdb       UNUMFUN
SGN                 fcs       "SGN"
                    fcb       SGNTOK
                    fdb       UNUMFUN
TABS                fcs       "TAB"
                    fcb       TABTOK
                    fdb       UNUMFUN
ADCS                fcs       "ADC"
                    fcb       ADCTOK
                    fdb       UNUMFUN
CALL                fcs       "CALL"
                    fcb       CALLTOK
                    fdb       UNUMFUN
PEEK                fcs       "PEEK"
                    fcb       PEEKTOK
                    fdb       UNUMFUN
                    fcs       "EEP"
                    fcb       FEEPTOK
                    fdb       UNUMFUN
HEX2                fcs       "HEX2"
                    fcb       HEX2TOK
                    fdb       UNUMFUN
HEX                 fcs       "HEX"
                    fcb       HEXTOK
                    fdb       UNUMFUN
                    fcs       "PORT"
                    fcb       FPRTATOK
                    fdb       FINDPORT
                    fcs       "TIME"
                    fcb       FTIMETOK
                    fdb       XTIMEF
                    fcs       "PACC"
                    fcb       FPACCTOK
                    fdb       XPACCF

                    fcb       0                   ; END OF TABLE MARKER.

XPOKE               equ       *
                    ldx       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    dex                           ; DEC. TO COMPENSATE FOR PUTTOK DONE IN XLATE.
                    stx       TBUFPTR             ; SAVE NEW POINTER VALUE. FALL THROUGH TO BNUMFUN.
                    lda       ,x                  ; GET TOKEN BACK INTO THE A-REG.

;***** xfdiv() *****/
;
; *xfdiv()
; *{
; short type[2];
; type[0]=type[1]=NUM;            both arguments must be type NUM
; dofunct(FDIVTOK,2,type);
; return(NUM);
; *}

BNUMFUN             equ       *
                    pshy
                    ldb       #NUM
                    pshb
                    pshb
                    tsy
                    ldb       #2
                    jsr       DOFUNCT
;                   lda       #NUM
                    pula
                    pula
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

XEEP                equ       *                   ; PROGRAM A WORD OF EEPROM.
                    ldx       TBUFPTR             ; COMPENSATE FOR TOKEN PLACEMENT BU UNUMFUN
                    dex                           ; ROUTINE.
                    stx       TBUFPTR             ; SAVE POINTER.
                    lda       ,x                  ; GET TOKEN FROM BUFFER.
                    bsr       UNUMFUN             ; GO TREAT AS A UNIARY NUMERIC FUNCTION.
                    jmp       ASIGNMT1            ; GO USE ASSIGNMENT CODE FOR REST OF FUNCTION.

UNUMFUN             equ       *
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

;***** dofunct() *****/
;
; *dofunct(functok,nargs,type)
; *short functok,nargs,*type;
; *{
; *tbufptr++=functok;            put function token in buffer
; if(*ibufptr!='(') { errcode=ILFSYERR; return; }
; *tbufptr++=OPARNTOK;           put open paren in token buffer
; ++ibufptr;

DOFUNCT             equ       *
                    jsr       PUTTOK
                    jsr       GETCHR
                    cmpa      #'('
                    beq       DOFUNCT1
DOFUNCT5            lda       #MPARNERR
                    jmp       RPTERR

DOFUNCT1            jsr       INCIBP
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

DOFUNCT4            lda       ,y
                    iny
                    pshb
                    jsr       XEXPRES
                    pulb
                    decb
                    beq       DOFUNCT3
                    jsr       CHKCOMA
                    bcc       DOFUNCT5
                    bra       DOFUNCT4

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

DOFUNCT3            equ       *
                    jsr       GETCHR
                    cmpa      #')'
                    bne       DOFUNCT5
                    jsr       INCIBP
                    lda       #CPARNTOK
                    jmp       PUTTOK              ; PUT TOKEN IN BUFFER & RETURN.

FINDPORT            equ       *
                    jsr       GETNXCHR            ; GO GET PORT "NUMBER".
                    jsr       ToUpper             ; Translate the character to upper case.
                    cmpa      #'A'                ; IS IT AN A OR HIGHER?
                    bhs       FINDPRT1            ; YES. GO CHECK UPPER LIMIT.
FINDPRT2            lda       #ILPRTERR           ; NO. ILLEGAL PORT "NUMBER".
                    jmp       RPTERR              ; REPORT ERROR.

FINDPRT1            cmpa      #'E'                ; IS IT HIGHER THAN AN "E"?
                    bhi       FINDPRT2            ; YES. ILLEGAL PORT.
                    suba      #'A'                ; SUBTRACT "BASE" PORT OF A
                    adda      #FPRTATOK           ; ADD IN "BASE" TOKEN.
                                                  ; STEAL SOME CODE.
XPACCF              equ       *
XTIMEF              jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    lda       #NUM                ; RETURN TYPE "NUM".
                    rts                           ; RETURN.
;                   include   'basiclb4.mod'
                    title     BASICLB4
                    page
;
;
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

XON                 equ       *
;                   jsr       BLANKS
                    lda       #NUM
                    jsr       XEXPRES
XON1                jsr       BLANKS
                    ldx       #GOTO
                    jsr       STREQ
                    bcc       XON2
                    lda       #GOTOTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       XGOTO
                    bra       XON5

XON2                ldx       #GOSUB
                    jsr       STREQ
                    bcs       XON4
                    lda       #IONSYERR
                    jmp       RPTERR

XON4                lda       #GOSUBTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       XGOSUB
XON5                jsr       BLANKS

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
                    bcs       XON6
                    rts

XON6                jsr       BLANKS
                    lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XON8                jsr       PUTDTOK
                    bra       XON5

XONIRQ              equ       *                   ; "ONIRQ" HAS A FUNCTION CODE & LINE NUMBER.
XONTIME             equ       *
                    lda       #NUM                ; GO GET THE VALUE OF THE TIMER WE SHOULD GO ON.
                    jsr       XEXPRES
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       CHKCOMA             ; GO CHECK FOR A COMMA.
                    bcs       XONTIME1            ; IF PRESENT, IT'S OK.
XONTIME2            lda       #MCOMAERR           ; IF NOT, REPORT AN ERROR.
                    jmp       RPTERR

XONTIME1            jsr       BLANKS
                    lda       #LCONTOK            ; GET THE LINE CONSTANT TOKEN.
                    jsr       PUTTOK              ; PUT IT IN THE TOKEN BUFFER.
                    jsr       GETLINUM            ; GO GET A LINE NUMBER.
                    jmp       PUTDTOK             ; PUT THE LINE NUMBER IN THE TOKEN BUFFER.

XONPACC             equ       *
                    bsr       GETARG              ; GET AN ARGUMENT AND A COMMA.
                    bra       XONTIME             ; GO USE SOME OTHER CODE.

GETARG              lda       #NUM                ; GO GET THE "OPERATING MODE" EXPRESSION.
                    jsr       XEXPRES
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       CHKCOMA             ; GO CHECK FOR COMMA.
                    bcc       XONTIME2            ; NO COMMA. REPORT ERROR.
                    jmp       BLANKS              ; SKIP BLANKS AFTER COMMA AND RETURN.

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

XIF                 equ       *
;                   jsr       BLANKS
                    inc       IFWHFLAG
                    lda       #NUM
                    jsr       XEXPRES
XIF1                jsr       BLANKS

; if(match("THEN"))              check for "THEN" clause
; {
;  *tbufptr++=THENTOK;           put THEN token in the buffer
;  blanks();                     skip any blanks after "THEN"

                    ldx       #THENS
                    jsr       STREQ
                    bcs       XIF2
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

XIF2                equ       *
                    lda       #THENTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcc       XIF9

XIF3                lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XIF6                jsr       PUTDTOK

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

XIF4                equ       *
                    jsr       BLANKS
                    ldx       #ELSES
                    jsr       STREQ
                    bcs       XIF7
                    rts

XIF7                lda       #ELSETOK
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
                    bcs       XIF8
XIF9                lda       #LINENERR
                    jmp       RPTERR

XIF8                lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XIF10               jmp       PUTDTOK

; return;                in any case, return
; *}

THENS               fcs       "THEN"
ELSES               fcs       "ELSE"

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

XFOR                equ       *
;                   jsr       BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       XFOR1
XFOR2               lda       #IVEXPERR
                    jmp       RPTERR

XFOR1               jsr       GETNXCHR
                    cmpa      #'='
                    bne       XFOR2

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
                    bcs       XFOR4
                    lda       #MTOERR
                    jmp       RPTERR

XFOR4               lda       #TOTOK
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
                    bcs       XFOR3
                    rts

XFOR3               lda       #STEPTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    lda       #NUM
                    jmp       XEXPRES

TO                  fcs       "TO"
STEP                fcs       "STEP"

;***** xnext() *****/
;
; *xnext()
; *{
; *tbufptr++=NEXTTOK;    put NEXT token in buffer
; blanks();              skip blanks
; if(getvar()!=NUM) errcode=SYTXERR;     get variable, must be numeric
; return;
; *}

XNEXT               equ       *
;                   jsr       BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       XNEXT1
                    lda       #SYTXERR
                    jmp       RPTERR

XNEXT1              rts

;***** xprint() *****/
;
; *xprint()
; *{
; *tbufptr++=PRINTTOK;   put PRINT token in buffer
; blanks();             skip blanks

XPRINT              equ       *
;                   jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #'#'                ; HAS AN ALTERNATE PORT BEEN SPECIFIED?
                    bne       XPRINT9             ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.
                    lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    bra       XPRINT7             ; GO GET EXPRESSION & CONTINUE.

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

XPRINT9             jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #EOL
                    beq       XPRINT2
                    cmpa      #MIDEOL
                    bne       XPRINT3
XPRINT2             rts

XPRINT3             jsr       GETCHR              ; GET THE NEXT CHARACTER IN THE BUFFER.
                    cmpa      #'"'                ; IS IT A STRING CONSTANT?
                    bne       XPRINT7
                    jsr       GETSCON             ; YES. GO GET A STRING CONSTANT.
                    bra       XPRINT8             ; CONTINUE.

XPRINT7             lda       #NUM
                    jsr       XEXPRES
XPRINT8             jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #EOL
                    beq       XPRINT2
                    cmpa      #MIDEOL
                    beq       XPRINT2
                    bsr       CHKCOMA
                    bcs       XPRINT9
XPRINT4             cmpa      #SEMI
                    beq       XPRINT6
                    lda       #MCMSMERR
                    jmp       RPTERR

XPRINT6             lda       #SEMITOK
                    bsr       CHKCOMA2
                    bra       XPRINT9

CHKCOMA             jsr       GETCHR              ; GET CHARACTER FROM INPUT BUFFER.
                    cmpa      #COMMA              ; IS IT A COMMA?
                    beq       CHKCOMA1            ; YES. PUT IT IN THE TOKEN BUFFER.
                    clc                           ; NO. FLAG NO COMMA FOUND.
                    rts                           ; RETURN.

CHKCOMA1            lda       #COMMATOK           ; GET THE COMMA TOKEN.
CHKCOMA2            jsr       PUTTOK              ; PUT THE TOKEN IN THE BUFFER.
                    jsr       INCIBP              ; BUMP THE INPUT BUFFER POINTER.
                    sec
                    rts                           ; RETURN.

;***** xinput() *****/
;
; *xinput()
; *{
; *tbufptr++=INPUTTOK;           put INPUT token in buffer
; blanks();                      skip blanks

XINPUT              equ       *
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

XINPUT2             jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #'"'
                    bne       INREADCM
                    jsr       GETSCON
                    bsr       CHKCOMA             ; IF COMMA PRESENT, PUT IN TOKEN BUFFER.
                    bcs       INREADCM
XINPUT3             lda       #MCOMAERR
                    jmp       RPTERR

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

XDIM                equ       *
INREADCM            equ       *
XREAD               equ       *
;                   jsr       BLANKS
                    jsr       GETVAR
XREAD1              jsr       BLANKS
                    bsr       CHKCOMA
                    bcs       XREAD
                    rts

XCHKDEV             equ       *
                    jsr       GETCHR
                    cmpa      #'#'                ; HAS AN ALTERNATE PORT BEEN SPECIFIED?
                    beq       XCHKDEV1            ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.
                    rts                           ; RETURN.

XCHKDEV1            lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    lda       #NUM                ; EXPRESSION MUST BE NUMERIC.
                    jsr       XEXPRES             ; GO GET THE EXPRESSION.
                    jsr       BLANKS              ; SKIP SPACES.
                    bsr       CHKCOMA             ; GO GET COMMA THAT MUST FOLLOW THE EXPRESSION.
                    bcc       XINPUT3             ; MISSING COMMA. GO REPORT THE ERROR.
                    rts                           ; IT WAS THERE. GO PROCESS THE REST OF THE STATEMENT.

XINBYTE             equ       *
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

XWHILE              equ       *
;                   jsr       BLANKS
                    inc       IFWHFLAG
                    lda       #NULL
                    jmp       XEXPRES

;***** xendwh() *****/
;
; *xendwh()
; *{
; *tbufptr++=ENDWHTOK;   put ENDWH token in buffer
; return;
; *}

XPACC               equ       *
XTIME               equ       *
                    ldb       #NUM                ; SETUP TO USE CODE IN "ASIGNMT".
                    jmp       ASIGNMT1            ; GO DO ASSIGNMENT STATEMENT.

;***** rpterr() *****/
;
; *rpterr()
; *{
; *char *ptr,c;
; ptr=inbuff;    point to start of input buffer
; nl();
; nl();
; while((c=*ptr++)!=EOL) outbyte(c);     print the input buffer

RPTERR              equ       *
                    sta       ERRCODE
                    jsr       NL2
                    ldx       INBUFFS
RPTERR1             lda       ,x
                    cmpa      #EOL
                    beq       RPTERR2
                    jsr       OUTBYTE
                    inx
                    bra       RPTERR1

; nl();          go to next line
; ptr=inbuff;    point to begining of input buffer
; while(ptr++ < ibufptr-2) outbyte('*');    output '*' to point of error
; pl("^^^");  point to error
; nl();

RPTERR2             equ       *
                    jsr       NL
                    ldx       IBUFPTR
                    dex
                    dex
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
RPTERR6             lda       #1
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

ARROWS              fcs       "^^^"
ERRORS              fcs       "ERROR # "

RPTRERR             equ       *                   ; REPORT A RUN TIME ERROR.
                    sta       ERRCODE
                    jsr       RPTERR5
RPTRERR1            ldx       #INLINE
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
                    jmp       MAINW

BREAKS              fcc       "BREAK"
INLINE              fcs       " IN LINE # "

          #ifz HC11

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

OUTDECI             equ       *
                    cpd       #0
                    bne       OUTDECI7
                    lda       #'0'
                    jmp       OUTBYTE

OUTDECI7            pshy
                    pshd
                    clrb
                    pshb
                    pshb
                    tsy
                    ldd       2,Y
                    bpl       OUTDECI1
                    negd
                    std       2,Y
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

OUTDECI1            equ       *
                    ldx       #PWRTEN
OUTDECI2            ldd       2,Y
                    clr       1,Y
OUTDECI3            subd      ,x
                    bmi       OUTDECI5
                    inc       1,Y
                    bra       OUTDECI3

OUTDECI5            addd      ,x
                    std       2,Y
                    lda       1,Y
                    bne       OUTDECI6
                    tst       ,y
                    beq       OUTDECI4
OUTDECI6            adda      #$30
                    ldb       #1
                    stb       ,y
                    jsr       OUTBYTE
OUTDECI4            inx
                    inx
                    tst       1,x
                    bne       OUTDECI2
                    ins
                    ins
                    ins
                    ins
                    puly
                    rts

PWRTEN              fdb       10000
                    fdb       1000
                    fdb       100
                    fdb       10
                    fdb       1
                    fdb       0

          #else

OUTDECI             equ       *
                    cpd       #0
                    bne       OUTDECI1
                    lda       #'0'
                    jmp       OUTBYTE

OUTDECI1            equ       *
                    pshy
                    pshd                          ; SAVE THE NUMBER TO PRINT.
                    ldd       #10000              ; NUMBER TO START DIVIDING BY.
                    pshd
                    clrb                          ; SET INITAL VALUE OF LEADING ZERO SUPRESS FLAG.
                    pshb
                    tsy
                    ldd       3,Y                 ; IS THE NUMBER NEGATIVE?
                    bpl       OUTDECI2            ; NO. GO PRINT THE NUMBER.
                    negd                          ; YES. MAKE THE NUMBER POSITIVE.
                    std       3,Y                 ; SAVE THE RESULT.
                    lda       #'-'                ; PRINT A MINUS SIGN TO SHOW IT'S NEGATIVE.
                    jsr       OUTBYTE
OUTDECI2            ldd       3,Y                 ; GET THE DIVIDEND.
                    ldx       1,Y                 ; GET THE DIVISOR.
                    idiv                          ; DO THE DIVIDE.
                    std       3,Y                 ; SAVE THE REMAINDER.
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
OUTDECI4            ldd       1,Y                 ; GET CURRENT DIVISOR.
                    ldx       #10                 ; DIVIDE IT BY 10.
                    idiv
                    stx       1,Y                 ; SAVE RESULT. ARE WE DONE?
                    bne       OUTDECI2            ; NO KEEP GOING.
                    ldb       #5                  ; DEALLOCATE LOCALS.
                    aby
                    tys
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.
          #endif
;                   include   'leditor.mod'
                    title     LEDITOR
                    page

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

STORLIN             equ       *
                    lda       #1                  ; set the continue flag.
                    sta       CONTFLAG            ; we don't allow continues if the program has been altered.
                    ldx       TKNBUFS             ; point to the start of the token buffer
                    ldd       ,x                  ; get the first 2 bytes of the token buffer (the line number).
                    cpd       HILINE              ; was the entered lines number higher than the highest so far?
                    bls       STORLIN1            ; no. go do an insert or replace operation.
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

STORLIN1            equ       *
                    bsr       FINDLINE
                    ldd       ,x
                    pshx
                    ldx       TKNBUFS
                    cpd       ,x
                    pulx
                    bne       INSRTLIN
                    jmp       REPLINE

; ***** delline() *****
;
; *delline(num)            delete line from basic buffer
; *int num;
; *{
; int *linum;
; char *ptr;
; if(num > hiline) return;       line number can't be there, return
; linum=findline(num);           look for the requested line #

DELLINE             equ       *
                    pshd                          ; SAVE THE LINE NUMBER TO DELETE.
                    tsy                           ; POINT TO THE LINE NUMBER WE SAVED.
                    cpd       HILINE              ; IS IT HIGHER THAN THE HIGHEST LINE ENTERED SO FAR?
                    bls       DELLINE1            ; NO. GO SEE IF THE LINE EXISTS.
DELLINE2            lda       #1                  ; YES. THE LINE CANNOT EXIST.
                    sta       CONTFLAG
                    puld                          ; PULL THE LINE NUMBER OFF THE STACK.
                    rts                           ; RETURN.

DELLINE1            bsr       FINDLINE            ; GO SEE IF THE LINE EXISTS.
; RETURN A POINTER TO A LINE NUMBER IN THE BASIC PROGRAM BUFFER.

; if(*linum!=num) return;        if the line # doesn't exist, return
; ptr=linum;                     make the int pointer a char pointer
; closespc(ptr[2],ptr);          go delete the line
; if(num==hiline) hiline=findhiln();
; return;
; *}

                    ldd       ,x                  ; GET THE LINE NUMBER THAT WAS FOUND.
                    cpd       ,y                  ; WAS THE LINE NUMBER FOUND THE ONE THAT WAS REQUESTED TO BE DELETED.
                    bne       DELLINE2            ; NO. THE LINE DOESN'T EXIST. JUST RETURN.
                    ldb       2,x                 ; YES. GET THE LENGTH OF THE LINE.
                    bsr       CLOSESPC            ; GO CLOSE THE SPACE IN THE PROGRAM BUFFER.
                    ldd       HILINE              ; GET THE HIGHEST LINE NUMBER ENTERED.
                    cpd       ,y                  ; DID WE DELETE THE HIGHEST LINE NUMBER?
                    bne       DELLINE2            ; NO. JUST RETURN.
                    bsr       FINDHILN            ; YES. GO FIND THE HIGHEST LINE NUMBER.
                    std       HILINE              ; SAVE IT.
                    bra       DELLINE2            ; RETURN.

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

CLOSESPC            equ       *                   ; ENTERED WITH
                    pshy                          ; SAVE THE CURRENT VALUE OF Y.
                    pshx                          ; TRANSFER X TO Y BY... PUSHING X AND THEN
                    puly                          ; PULLING Y.
                    aby                           ; ADD THE LENGTH TO Y.
CLOSESP1            cpy       BASEND              ; HAVE WE MOVED ALL THE BYTES?
                    bhs       CLOSESP2            ; YES. RETURN.
                    lda       ,y                  ; NO. GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    inx                           ; ADVANCE THE DESTINATION POINTER.
                    iny                           ; ADVANCE THE SOURCE POINTER.
                    bra       CLOSESP1            ; GO CHECK TO SEE IF WE'RE DONE.

CLOSESP2            stx       BASEND              ; SAVE THE NEW 'END OF BASIC PROGRAM' POINTER.
                    jsr       MoveVarsDn          ; MOVE ALL THE VARIABLES DOWN.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

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

FINDLINE            equ       *
                    ldx       BASBEG
FINDLIN1            cpd       ,x
                    bls       FINDLIN2
                    pshb
                    ldb       2,x
                    abx
                    pulb
                    bra       FINDLIN1

FINDLIN2            rts

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

FINDHILN            equ       *
                    ldx       BASBEG
FINDHIL1            cpx       BASEND
                    beq       FINDHIL2
                    ldd       ,x
                    pshb
                    ldb       2,x
                    abx
                    pulb
                    bra       FINDHIL1

FINDHIL2            rts

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

INSRTLIN            equ       *
                    pshx
                    ldx       TKNBUFS
                    ldb       2,x
                    pulx
                    pshx
                    bsr       OPENSPC
                    pulx
                    bra       PUTLINE

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

OPENSPC             equ       *
                    pshy
                    pshx
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       OPENSPC4
                    jsr       MoveVarsUp
                    ldx       BASEND
                    pshx
                    abx
                    pshx
                    tsy
                    ldd       ,y
OPENSPC1            std       BASEND
OPENSPC3            ldd       2,Y
                    cpd       4,Y
                    blo       OPENSPC2
                    ldx       2,Y
                    lda       ,x
                    dex
                    stx       2,Y
                    ldx       ,y
                    sta       ,x
                    dex
                    stx       ,y
                    bra       OPENSPC3

OPENSPC4            lda       #OMEMERR
                    jmp       RPTERR

OPENSPC2            pulx
                    pulx
                    pulx
                    puly
                    rts

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

PUTLINE             equ       *
                    pshx
                    ldx       TKNBUFS
                    ldb       2,x
                    pulx
                    ldy       TKNBUFS
PUTLINE1            lda       ,y
                    iny
                    sta       ,x
                    inx
                    decb
                    bne       PUTLINE1
                    rts

;***** apendlin() *****/        appent line to end of program buffer
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

APENDLIN            equ       *
                    ldx       TKNBUFS
                    ldb       2,x
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       APENDLN1
;                   ldb       TKNBUF+2
                    jsr       MoveVarsUp
                    ldx       BASEND
                    abx
                    xgdx
                    ldx       BASEND
                    std       BASEND
                    bra       PUTLINE

APENDLN1            lda       #OMEMERR
                    jmp       RPTERR

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

REPLINE             equ       *
                    ldb       2,x
                    pshx
                    ldx       TKNBUFS
                    subb      2,x
                    pulx
                    bne       REPLINE1
                    bra       PUTLINE

; else if(lendif<0)              if line in tknbuf is larger
;      {
;       lendif=-lendif;          make it a positive number
;       openspc(lendif,ptr);     tru to open up a space
;       if(errcode) return;      if not enough memory, return
;       putline(ptr);            if ok, copy line to program buffer
;      }

REPLINE1            equ       *
                    bpl       REPLINE2
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

REPLINE2            equ       *
                    pshx
                    jsr       CLOSESPC
                    pulx
                    bra       PUTLINE

MoveVarsUp          pshy                          ; SAVE THE Y REGISTER.
                    pshb                          ; SAVE THE BYTE COUNT.
                    ldx       VAREND              ; POINT TO THE END OF THE VARIABLE MEMORY SPACE.
                    ldy       VAREND              ; POINT TO THE END OF VARIABLE MEMORY SPACE.
                    abx                           ; ADD THE NUMBER OF BYTES TO MOVE TO THE POINTER.
                    ldd       VAREND              ; GET THE CURRENT VARIABLE TABLE ENDING ADDRESS.
                    stx       VAREND              ; SAVE THE NEW END OF VARIABLE TABLE POINTER.
                    subd      VARBEGIN            ; CALCULATE THE NUMBER OF BYTES TO MOVE.
                    beq       MOVEUP2             ; JUST RETURN IF THERE IS NOTHING TO MOVE.
                    std       VarSize             ; save the size of the variable table (9/12/89).
MOVEUP1             lda       ,y                  ; GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    dex
                    dey
                    bsr       DecCount            ; DECREMENT THE BYTE COUNT. ARE WE DONE? (9/12/89).
                    bpl       MOVEUP1             ; GO TILL WE'RE DONE.
                    inx                           ; ADJUST THE POINTER
MOVEUP2             stx       VARBEGIN            ; SAVE THE NEW START OF VARIABLE TABLE POINTER.
                    pulb                          ; RESTORE THE BYTE COUNT.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

MoveVarsDn          pshy                          ; SAVE Y.
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
                    beq       MOVEDN2             ; IF THE SIZE IS 0 (NO VARIABLES ALLOCATED) EXIT.
MOVEDN1             lda       ,y                  ; GET A BYTE.
                    sta       ,x                  ; MOVE IT.
                    inx                           ; MOVE THE DESTINATION POINTER.
                    iny                           ; MOVE THE SOURCE POINTER.
                    bsr       DecCount            ; DECREMENT THE BYTE COUNT. ARE WE DONE? (9/12/89).
                    bpl       MOVEDN1             ; NO. KEEP MOVIN' THEM BYTES.
                    dex
MOVEDN2             stx       VAREND              ; SAVE THE NEW POINTER TO THE END OF THE VARIABLE TABLE.
                    pulb                          ; RESTORE THE BYTE COUNT.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

DecCount            ldd       VarSize             ; get the size of the variable table.
                    decd                          ; decrement it.
                    std       VarSize             ; save the new value.
                    rts                           ; return.
;                   include   'inits.mod'
                    title     Inits

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

INITVARS            equ       *
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
INIT1               clr       ,x                  ; CLEAR THE STORAGE TO ZERO.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       BASMEND             ; ARE WE DONE?
                    bls       INIT1               ; NO. KEEP CLEARING.
                                                  ; YES. POINT TO THE PHYSICAL END OF MEMORY.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG AGAIN.
                    cmpa      #$55                ; IS THE AUTO START MODE SET?
                    bne       INIT5               ; NO. DO A NORMAL INIT.

                    jsr       AUTOLOAD            ; GO LOAD the program and VARIABLES INTO RAM.
INIT5               ldd       #0                  ; MAKE THE HIGHEST LINE IN THE PROGRAM 0.
                    std       HILINE
                    std       CURLINE             ; MAKE THE CURRENT LINE #0.
                    jsr       RUNINIT             ; GO INITALIZE ALL THE SOFTWARE STACKS.
                    clr       TRFLAG              ; TURN THE TRACE MODE OFF.
                    lda       #1                  ; "CONT" COMMAND NOT ALLOWED.
                    sta       CONTFLAG
                    ldx       DFLOPADR            ; point to the D-Flip flop address.
                    sta       ,x                  ; CONNECT SCI RX PIN TO THE HOST CONNECTOR.
                    clr       DEVNUM              ; MAKE THE DEFAULT DEVICE NUMBER 0 (CONSOLE).
                    clr       IMMID               ; clear the immediate mode flag (added 9/17/90).
                    ldx       VAREND              ; GET THE POINTER TO THE END OF VARIABLE STORAGE.
                    inx                           ; BUMP IT BY 1.
                    stx       STRASTG             ; POINT TO THE DYNAMIC ARRAY STORAGE.
                    rts                           ; RETURN.

POWERUP             equ       *
                    ldd       IOBase              ; GET THE BASE ADDRESS OF THE I/O REGISTERS.
                    std       IOBaseV
                    lsra
                    lsra
                    lsra
                    lsra
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
POWERUP1            sta       ,x                  ; STORE IT.
                    inx                           ; POINT TO THE NEXT VECTOR.
                    sty       ,x                  ; INITALIZE VECTOR TO "RTI".
                    inx
                    inx
                    decb                          ; DONE?
                    bne       POWERUP1            ; NO. INITALIZE ALL VECTORS.
                    ldx       #ILLOP              ; POINT TO THE ILLEGAL OP-CODE VECTOR.
                    ldd       #POWERUP            ; GET THE ADDRESS OF THE POWER UP VECTOR.
                    std       1,x                 ; INITALIZE ILLEGAL OP-CODE VECTOR.
                    std       4,x                 ; INITALIZE WATCH DOG FAIL VECTOR.
                    std       7,x                 ; INITALIZE CLOCK MONITOR FAIL VECTOR.

                    ldx       #INTABLE            ; POINT TO THE START OF THE I/O VECTOR TABLE.
                    ldy       #IOVects            ; point to the default table in ROM.
                    ldb       #32                 ; GET NUMBER OF BYTES IN THE TABLE.
POWERUP2            lda       ,y                  ; Move a byte of the table from ROM into RAM.
                    sta       ,x
                    inx                           ; POINT TO THE NEXT BYTE.
                    iny
                    decb                          ; DECREMENT THE COUNT.
                    bne       POWERUP2            ; GO TILL WE'RE DONE.

                    ldx       #TIMEINT            ; GET THE ADDRESS OF THE OUTPUT COMPARE 1 ROUTINE.
                    stx       TOC1+1              ; PUT IT IN THE INTERRUPT VECTOR.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    clra
                    clrb
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
                    std       ONTIMLIN            ; INITALIZE THE LINE POINTERS.
                    std       ONIRQLIN
                    std       ONPACLIN

                    ldx       UserInit
                    jsr       ,x                  ; INITALIZE THE ACIA & SCI.
                    jmp       MAIN                ; GO TO BASIC.

TIMEINT             bsr       TIMINTS
RETII               rti                           ; RETURN FROM ALL INTERRUPT SOURCES.

TIMINTS             ldx       IOBaseV             ; Point to the I/O Base Address.
                    ldd       TOC1REG,x           ; GET THE VALUE OF THE TIMER/COUNTER.
TIMINTS3            addd      TimeVal             ; ADD IN 62500 FOR NEXT COMPARE ( 2 HZ INT.).
                    std       TOC1REG,x           ; PUT IT IN THE OUTPUT COMPARE REGISTER.
                    lda       #$80                ; SETUP TO CLEAR THE OC1 FLAG.
                    sta       TFLAG1,x
                    dec       TIMEPRE             ; HAVE TWO OUTPUT COMPARES OCCURED?
                    bne       TIMINTS1            ; NO. JUST RETURN.
                    lda       #SWPRE              ; YES. RELOAD THE REGISTER.
                    sta       TIMEPRE
                    ldd       TIMEREG             ; GET THE CURRENT VALUE OF "TIME".
                    incd                          ; ADD 1 SECOND TO THE COUNT.
                    std       TIMEREG             ; UPDATE THE TIME REGISTER.
                    ldd       TIMECMP             ; GET THE VALUE TO COMPARE TO FOR "ONTIME".
                    beq       TIMINTS1            ; IF IT'S 0, THE "ONTIME" FUNCTION IS OFF.
                    cpd       TIMEREG             ; DOES THE COMPARE VALUE MATCH THE TIME REGISTER?
                    bne       TIMINTS1            ; NO. JUST RETURN.
                    ldy       ONTIMLIN            ; MAKE THE POINTER TO THE LINE NUMBER THE NEW IP.
                    ins                           ; GET RID OF THE RETURN ADDRESS.
                    ins
TIMINTS2            inc       IMMID               ; FAKE THE GOTO ROUTINE OUT.
                    ldd       CURLINE             ; SAVE THE CURRENT LINE NUMBER IN MAIN PROGRAM.
                    std       SCURLINE
                    ldd       ADRNXLIN            ; SAVE THE ADDRESS OF THE NEXT LINE IN MAIN PROG.
                    std       SADRNXLN
                    jmp       RGOTO3              ; GOTO THE SERVICE ROUTINE.

TIMINTS1            rts                           ; RETURN.

IRQINT              equ       *
                    ldy       ONIRQLIN            ; GET POINTER TO LINE NUMBER OF THE IRQ SERVICE.
                    bne       TIMINTS2            ; GO DO IT.
                    rti                           ; IF IT'S 0, "ONIRQ" HAS NOT BEEN EXECUTED.

PACCINT             equ       *
                    ldx       IOBaseV
                    lda       #$30                ; RESET BOTH THE TIMER OVERFLOW & INPUT FLAG.
                    sta       TFLG2,x
                    ldy       ONPACLIN            ; GET POINTER TO LINE NUMBER OF THE SERVICE ROUT.
                    bne       TIMINTS2
                    rti
;                   include   'command1.mod'
                    title     COMMAND1
                    page

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

CHCKCMDS            equ       *
                    jsr       GETCHR              ; GET FIRST CHAR FROM THE INPUT BUFFER.
                    cmpa      #EOL                ; IS IT AN EOL?
                    bne       CHKCMDS1            ; NO. GO CHECK FOR COMMANDS.
CHKCMDS5            ldd       #0                  ; YES. JUST RETURN.
                    rts

CHKCMDS1            ldx       #CMDTBL             ; POINT TO COMMAND TABLE.
CHKCMDS2            jsr       STREQ               ; GO CHECK FOR A COMMAND.
                    bcs       CHKCMDS3            ; IF WE FOUND ONE GO EXECUTE IT.
CHKCMDS4            inx                           ; ADVANCE POINTER TO NEXT CHAR IN TABLE ENTRY.
                    lda       ,x                  ; GET THE CHAR. ARE WE AT THE END OF THIS ENTRY?
                    bne       CHKCMDS4            ; NO. KEEP GOING TILL WE ARE PAST IT.
                    inx                           ; BYPASS END OF COMMAND MARKER & EXECUTION ADDR.
                    inx
                    inx
                    tst       ,x                  ; ARE WE AT THE END OF THE TABLE?
                    bne       CHKCMDS2            ; NO. GO CHECK THE NEXT TABLE ENTRY.
                    bra       CHKCMDS5            ; YES. RETURN W/ ENTRY NOT FOUND INDICATION.

CHKCMDS3            ldx       1,x                 ; GET ADDRESS OF COMMAND.
                    jsr       ,x                  ; GO DO IT.
                    ldd       #1                  ; SHOW WE EXECUTED A COMMAND.
                    rts                           ; RETURN.

CMDTBL              equ       *
                    fcs       "LIST"
                    fdb       CLIST

                    fcs       "RUN"
                    fdb       CRUN

                    fcs       "NEW"
                    fdb       CNEW

                    fcs       "CONT"
                    fdb       CCONT

                    fcs       "CLEAR"
                    fdb       CCLEAR

                    fcs       "ESAVE"
                    fdb       CESAVE

                    fcs       "ELOAD"
                    fdb       CELOAD

                    fcs       "LLIST"
                    fdb       CLLIST

                    fcs       "AUTOST"
                    fdb       CAUTOST

                    fcs       "NOAUTO"
                    fdb       CNOAUTO

                    fcs       "FREE"
                    fdb       CFREE

                    fcb       0                   ; END OF TABLE MARKER.

;***** clist() *****/
;
; *clist()
; *{
; int *intptr;
; char token;
; if(basbeg==basend) return;             if basic buffer empty, return
; skipspcs();                    skip any spaces after "LIST"

CLIST               equ       *
                    jsr       NL2
                    ldd       BASBEG
                    cpd       BASEND
                    bne       CLIST1
                    rts

CLIST1              jsr       SKIPSPCS

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
                    beq       CLIST3
                    ldd       FIRSTLIN
                    std       LASTLIN
                    cpd       HILINE
                    bls       CLIST4
                    rts

CLIST3              jsr       INCIBP
                    jsr       GETLINUM
                    cpd       HILINE
                    bls       CLIST13
                    ldd       HILINE
CLIST13             std       LASTLIN
                    bra       CLIST4

CLIST2              cmpa      #EOL
                    beq       CLIST14
                    rts

CLIST14             ldx       BASBEG
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

CLIST6              ldd       TOKPTR
                    cpd       LASTLIN
                    bne       CLIST7
                    rts

CLIST7              ldx       TOKPTR
                    ldd       ,x
                    inx
                    inx
                    inx
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
                    bra       CLIST6

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

LVARCON             equ       *
                    ldx       TOKPTR
                    lda       ,x
                    anda      #$EF                ; MASK OFF ARRAY INDICATOR IF PRESENT.
                    ldx       #VCTOKTBL
LVARCON1            cmpa      ,x
                    beq       LVARCON2
                    inx
                    inx
                    inx
                    tst       ,x
                    bne       LVARCON1
                    lda       #ILTOKERR
                    jmp       RPTERR

LVARCON2            ldx       1,x
                    jsr       ,x
                    rts

VCTOKTBL            equ       *
                    fcb       IVARTOK
                    fdb       LIVAR
                    fcb       SCONTOK
                    fdb       LSCON
                    fcb       LCONTOK
                    fdb       LLCON
                    fcb       ICONTOK
                    fdb       LICON
                    fcb       0                   ; END OF TABLE MARKER.

;***** livar() *****/
;
; *livar()
; *{
; lfvar();
; outbyte('%');
; return;
; *}

LIVAR               equ       *
                    ldx       TOKPTR
                    inx
                    ldd       ,x
                    addd      VARBEGIN
                    inx
                    inx
                    stx       TOKPTR
                    xgdx
LIVAR2              lda       1,x
                    jsr       OUTBYTE
                    lda       2,x
                    beq       LIVAR1
                    jsr       OUTBYTE
LIVAR1              rts

LFCON               equ       *
                    ldd       TOKPTR
                    addd      #FSIZ+1
LFCON2              xgdx
                    ldb       ,x
                    inx
LFCON1              lda       ,x
                    jsr       OUTBYTE
                    inx
                    decb
                    bne       LFCON1
                    stx       TOKPTR
                    rts

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

LICON               equ       *
                    ldd       TOKPTR
                    addd      #ISIZ+1
                    bra       LFCON2

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

LSCON               equ       *
                    ldd       TOKPTR
                    incd
                    bra       LFCON2

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

LLCON               equ       *
                    ldx       TOKPTR
                    inx
                    ldd       ,x
                    inx
                    inx
                    stx       TOKPTR
                    jsr       OUTDECI
                    rts

;***** lkeyword *****/
;
; *lkeyword()
; *{
; char *charptr,token;
; token=*tokptr++

LKEYWORD            equ       *
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    cmpa      #MSCNTOK
                    bne       LKEYWRD3
                    jmp       LMSPCS

LKEYWRD3            cmpa      #REMTOK
                    bne       LKEYWRD4
                    jmp       LREMLINE

LKEYWRD4            cmpa      #DATATOK
                    bne       LKEYWRD5
                    jmp       LDATALIN

LKEYWRD5            cmpa      #FUNCTFLG
                    bne       LKEYWRD6
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    ldx       #LFUNCTBL
                    bra       LKEYWRD1

LKEYWRD6            ldx       #TOKTBL
LKEYWRD1            cmpa      ,x
                    beq       LKEYWRD2
                    inx
                    inx
                    inx
                    tst       ,x
                    bne       LKEYWRD1
                    lda       #ILTOKERR
                    jmp       RPTERR

LKEYWRD2            ldx       1,x
                    jmp       PL

LMSPCS              equ       *
                    ldx       TOKPTR
                    ldb       ,x
                    inx
                    stx       TOKPTR
                    lda       #$20
LMSPCS1             jsr       OUTBYTE
                    decb
                    bne       LMSPCS1
                    rts

LDATALIN            equ       *
                    ldx       #DATA
                    jsr       PL
                    bra       LREM3

LREMLINE            equ       *
                    ldx       #REM
                    jsr       PL
LREM3               ldx       TOKPTR
                    inx                           ; PUT POINTER PAST LENGTH BYTE.
LREM1               lda       ,x
                    cmpa      #EOL
                    bne       LREM2
                    inx
                    stx       TOKPTR
                    rts

LREM2               jsr       OUTBYTE
                    inx
                    bra       LREM1

TOKTBL              equ       *
                    fcb       LETTOK
                    fdb       LET
                    fcb       READTOK
                    fdb       READ
                    fcb       RESTRTOK
                    fdb       RESTORE
                    fcb       GOSUBTOK
                    fdb       GOSUB
                    fcb       GOTOTOK
                    fdb       GOTO
                    fcb       ONTOK
                    fdb       ON
                    fcb       RETNTOK
                    fdb       RETURN
                    fcb       IFTOK
                    fdb       IIF
                    fcb       THENTOK
                    fdb       THENS
                    fcb       ELSETOK
                    fdb       ELSES
                    fcb       INPUTTOK
                    fdb       INPUT
                    fcb       PRINTTOK
                    fdb       PRINT
                    fcb       FORTOK
                    fdb       FOR
                    fcb       NEXTTOK
                    fdb       NEXT
                    fcb       STOPTOK
                    fdb       STOPSS
                    fcb       ENDTOK
                    fdb       ENDS
                    fcb       TRONTOK
                    fdb       TRON
                    fcb       TROFFTOK
                    fdb       TROFF
                    fcb       WHILETOK
                    fdb       WHILE
                    fcb       ENDWHTOK
                    fdb       ENDWH
                    fcb       STEPTOK
                    fdb       STEP
                    fcb       TOTOK
                    fdb       TO
                    fcb       COMMATOK
                    fdb       COMMAC
                    fcb       SEMITOK
                    fdb       SEMIC
                    fcb       MEOLTOK
                    fdb       COLLINC
                    fcb       IMLETTOK
                    fdb       IMLET
                    fcb       POKETOK
                    fdb       POKE
                    fcb       EQUALTOK
                    fdb       EQ
                    fcb       OPARNTOK
                    fdb       OPARN
                    fcb       CPARNTOK
                    fdb       CPARN
                    fcb       ANDTOK
                    fdb       ANDS
                    fcb       ORTOK
                    fdb       ORS
                    fcb       EORTOK
                    fdb       EORS
                    fcb       LTTOK
                    fdb       LT
                    fcb       GTTOK
                    fdb       GT
                    fcb       LTEQTOK
                    fdb       LTEQ
                    fcb       GTEQTOK
                    fdb       GTEQ
                    fcb       EQTOK
                    fdb       EQ
                    fcb       NOTEQTOK
                    fdb       NOTEQ
                    fcb       PLUSTOK
                    fdb       PLUS
                    fcb       MINUSTOK
                    fdb       MINUS
                    fcb       MULTTOK
                    fdb       MULT
                    fcb       DIVTOK
                    fdb       DIV
                    fcb       MODTOK
                    fdb       MODS
                    fcb       NOTTOK
                    fdb       NOTS
                    fcb       RTIMETOK
                    fdb       RTIMES
                    fcb       NEGTOK
                    fdb       NEGS
                    fcb       SSCNTOK
                    fdb       SPACE
                    fcb       DIMTOK
                    fdb       DIM
                    fcb       EEPTOK
                    fdb       EEP
                    fcb       PORTATOK
                    fdb       PORTA
                    fcb       PORTBTOK
                    fdb       PORTB
                    fcb       PORTCTOK
                    fdb       PORTC
                    fcb       PORTDTOK
                    fdb       PORTD
                    fcb       PNUMTOK
                    fdb       POUNDSGN
                    fcb       INBYTTOK
                    fdb       INBYTES
                    fcb       TIMETOK
                    fdb       TIME
                    fcb       ONTIMTOK
                    fdb       ONTIME
                    fcb       ONIRQTOK
                    fdb       ONIRQ
                    fcb       RETITOK
                    fdb       RETI
                    fcb       PACCTOK
                    fdb       PACC
                    fcb       ONPACTOK
                    fdb       ONPACC
                    fcb       SLEEPTOK
                    fdb       SLEEP
                    fcb       0                   ; END OF TABLE MARKER.

LFUNCTBL            equ       *
                    fcb       FDIVTOK
                    fdb       FDIVS
                    fcb       CHRTOK
                    fdb       CHRS
                    fcb       ADCTOK
                    fdb       ADCS
                    fcb       ABSTOK
                    fdb       ABS
                    fcb       RNDTOK
                    fdb       RND
                    fcb       SGNTOK
                    fdb       SGN
                    fcb       TABTOK
                    fdb       TABS
                    fcb       CALLTOK
                    fdb       CALL
                    fcb       PEEKTOK
                    fdb       PEEK
                    fcb       FEEPTOK
                    fdb       EEP
                    fcb       HEXTOK
                    fdb       HEX
                    fcb       FPRTATOK
                    fdb       PORTA
                    fcb       FPRTBTOK
                    fdb       PORTB
                    fcb       FPRTCTOK
                    fdb       PORTC
                    fcb       FPRTDTOK
                    fdb       PORTD
                    fcb       FPRTETOK
                    fdb       PORTE
                    fcb       FTIMETOK
                    fdb       TIME
                    fcb       HEX2TOK
                    fdb       HEX2
                    fcb       FPACCTOK
                    fdb       PACC
IMLET               fcb       0                   ; NO KETWORD TO PRINT FOR AN IMPLIED LET.
COLLINC             fcs       ":"
SEMIC               fcs       ";"
COMMAC              fcs       ","
OPARN               fcs       "("
CPARN               fcs       ")"
SPACE               fcs       " "
PORTE               fcs       "PORTE"
POUNDSGN            fcs       "#"

CRUN                equ       *
                    jsr       NL2                 ; DO 2 CR/LF SEQUENCES.
                    jsr       RUNINIT             ; INITALIZE RUNTIME VARIABLES.
                    lda       #1                  ; SET THE RUN MODE FLAG.
                    sta       RUNFLAG

;        END OF POINTER INITIALIZATIONS

                    ldy       BASBEG              ; POINT TO THE START OF THE PROGRAM.
                    cpy       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       CRUN5               ; YES. GO RUN IT.
                    rts                           ; NO. RETURN.

CRUN5               ldd       ,y                  ; GET NUMBER OF FIRST/NEXT LINE OF BASIC PROGRAM.
                    std       CURLINE             ; MAKE IT THE CURRENT LINE.
                    tst       TRFLAG              ; IS THE TRACE MODE TURNED ON?
                    beq       CRUN6               ; NO. CONTINUE.
                    lda       #'['                ; YES. PRINT THE CURRENT LINE.
                    jsr       OUTBYTE
                    ldd       CURLINE
                    jsr       OUTDECI
                    lda       #']'
                    jsr       OUTBYTE
                    jsr       NL
CRUN6               pshy                          ; SAVE POINTER TO START OF NEW LINE.
                    ldb       2,Y                 ; GET LENGTH OF LINE.
                    aby                           ; POINT TO START OF NEXT LINE.
                    sty       ADRNXLIN            ; SAVE THE ADDRESS OF THE NEXT LINE.
                    puly
                    ldb       #3                  ; BYTE COUNT OF LINE NUMBER & LENGTH.
                    aby                           ; POINT TO THE FIRST TOKEN.
CRUN4               bsr       RSKIPSPC            ; SKIP SPACES IF PRESENT.
                    ldb       ,y                  ; GET KEYWORD TOKEN.
                    iny                           ; POINT PAST THE KEYWORD.
                    bsr       RSKIPSPC            ; SKIP SPACES AFTER KEYWORD.
                    decb                          ; SUBTRACT ONE FOR INDEXING.
                    lslb                          ; MULTIPLY BY THE # OF BYTES PER ADDRESS.
                    ldx       #RKEYWORD           ; POINT TO RUN TIME ADDRESS TABLE.
                    abx                           ; POINT TO ADDRESS
                    ldx       ,x                  ; POINT TO RUNTIME ROUTINE.
                    jsr       ,x                  ; GO DO IT.

CRUN2               dec       BREAKCNT            ; SHOULD WE CHECK FOR A BREAK YET?
                    bne       CRUN7               ; NO. CONTINUE.
                    jsr       CHCKBRK             ; CHECK FOR BREAK FROM CONSOLE.
;
CRUN7               bsr       RSKIPSPC            ; SKIP ANY SPACES.
                    lda       ,y                  ; GET THE NEXT TOKEN IN THE LINE.
                    cmpa      #EOLTOK             ; ARE WE AT THE END OF THE LINE?
                    bne       CRUN3
                    iny                           ; YES. POINT TO START OF THE NEXT LINE.
CRUN1               cpy       BASEND              ; HAVE WE REACHED THE END OF THE BASIC PROGRAM?
                    bne       CRUN5               ; NO. GO EXECUTE THE NEXT LINE.
                    jmp       REND                ; GO DO AN "END".

CRUN3               iny                           ; MUST BE A MID EOL.
                    bra       CRUN4               ; GO DO NEXT KEYWORD.

RSKIPSPC            lda       ,y                  ; GET A CHARACTER.
                    bmi       RSKIP2
                    cmpa      #SSCNTOK            ; IS IT A SINGLE SPACE?
                    beq       RSKIP1              ; YES. BUMP IP BY 1.
                    blo       RSKIP2
                    iny                           ; BUMP IP BY 2 FOR MULTIPLE SPACES.
RSKIP1              iny                           ; BUMP IP.
RSKIP2              rts                           ; RETURN.

RKEYWORD            equ       *
                    fdb       RLET
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

RUNLINE             jsr       NL2
                    ldy       TKNBUFS             ; POINT TO THE TOKEN BUFFER.
                    ldd       ,y                  ; GET CURRENT LINE NUMBER.
                    std       CURLINE             ; MAKE "0" THE CURRENT LINE #.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER & LENGTH.
                    aby
RUNLINE2            bsr       RSKIPSPC            ; SKIP SPACES.
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
                    bne       RUNLINE1
                    rts

RUNLINE1            iny                           ; MUST BE A MID EOL.
                    bra       RUNLINE2

CHCKBRK             equ       *
                    lda       #10                 ; RELOAD THE BREAK CHECK COUNT.
                    sta       BREAKCNT
                    jsr       CONSTAT             ; GET CONSOLE STATUS. CHARACTER TYPED?
                    bne       CHCKBRK1            ; YES. GO CHECK IT OUT.
                    rts                           ; NO. RETURN.

CHCKBRK1            jsr       INCONNE             ; GET BYTE FROM CONSOLE BUT DON'T ECHO.
                    cmpa      #$03                ; WAS IT A CONTROL-C?
                    beq       CHCKBRK2            ; YES. GO DO A BREAK.
                    rts                           ; NO. RETURN.

CHCKBRK2            sty       IPSAVE              ; SAVE THE IP POINTER IN CASE OF A CONTINUE.
                    jsr       NL
                    ldx       #BREAKS             ; POINT TO BREAK STRING.
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    jsr       NL
                    jmp       MAINW

RUNINIT             equ       *
                    jsr       CCLEAR              ; GO CLEAR ALL VARIABLE STORAGE.
RUNINIT1            ldx       STNUMS              ; GET START OF NUMERIC OPERAND STACK.
                    stx       NUMSTACK            ; INITALIZE THE OPERAND STACK POINTER.
                    ldx       STOPS               ; GET THE START OF THE OPERATOR STACK.
                    stx       OPSTACK             ; INITALIZE THE OPREATOR STACK POINTER.
                    ldx       STFORSTK            ; GET THE START OF THE FOR-NEXT STACK.
                    stx       FORSTACK            ; INITALIZE THE FOR NEXT STACK POINTER.
                    ldx       STWHSTK             ; GET THE START OF THE WHILE STACK.
                    stx       WHSTACK             ; INITALIZE THE WHILE STACK POINTER.
                    ldx       STGOSTK             ; GET THE START OF THE GOSUB STACK.
                    stx       GOSTACK             ; SET THE START OF THE GOSUB STACK.
                    ldx       VAREND              ; GET THE VARIABLE END POINTER.
                    inx                           ; POINT TO THE NEXT AVAILABLE BYTE.
                    stx       STRASTG             ; INITALIZE THE STRING/ARRAY STORAGE POINTER.
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    lda       #10                 ; SET COUNT FOR BREAK CHECK.
                    sta       BREAKCNT
                    clr       CONTFLAG            ; CLEAR THE CONTINUE FLAG.
                    ldx       #0                  ; CLEAR THE DATA POINTER.
                    stx       DATAPTR
                    rts

CCONT               equ       *
                    jsr       NL2
                    tst       CONTFLAG
                    bne       CCONT1
                    ldy       IPSAVE
                    jmp       CRUN7

CCONT1              lda       #CNTCNERR
                    sta       ERRCODE
                    jmp       RPTERR5

CNEW                equ       *
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG.
                    cmpa      #$55                ; IS IT SET?
                    bne       CNEW1               ; NO. GO INITIALIZE EVERYTHING.
                    lda       #$FF                ; YES. RESET (ERASE) IT.
                    sta       AUTOSTF,x
                    jsr       DLY10MS
CNEW1               jsr       INITVARS            ; INITIALIZE EVERYTHING.
                    rts                           ; RETURN.

CCLEAR              equ       *
                    jsr       RUNINIT1            ; GO INITALIZE ALL STACKS ETC.
CCLEAR3             ldx       VARBEGIN
CCLEAR1             lda       ,x
                    beq       CCLEAR2
                    inx
                    inx
                    inx
                    jsr       CLRVAR
                    bra       CCLEAR1

CCLEAR2             ldx       VAREND
                    inx
                    stx       STRASTG
                    rts
;                   include   'command2.mod'
                    name      COMMAND2
                    page

CESAVE              equ       *
                    ldd       BASBEG              ; GET POINTER TO THE START OF THE BASIC PROGRAM.
                    cpd       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       CESAVE1             ; YES. GO SAVE IT.
                    rts                           ; NO. RETURN.

CESAVE1             ldd       VAREND
                    subd      BASBEG
                    cpd       EESize
                    bls       CESAVE5
                    lda       #EETOSMAL
                    jmp       RPTERR

CESAVE5             ldx       EEStart             ; point to the start of the EEPROM.
                    ldy       #BASBEG
                    ldb       #4
                    stb       COUNT
CESAVE3             ldd       ,y
                    subd      RAMStart
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    tba
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    iny
                    iny
                    dec       COUNT
                    bne       CESAVE3

                    ldd       ,y
                    sta       ,x
                    jsr       DLY10MS
                    inx
                    tba
                    sta       ,x
                    jsr       DLY10MS

                    ldx       EEStart
                    ldy       BASBEG
CESAVE4             lda       ,y
                    sta       SSTART,x
                    jsr       DLY10MS
                    inx
                    iny
                    cpy       VAREND
                    bls       CESAVE4
                    rts                           ; RETURN.

CELOAD              equ       *
                    ldx       EEStart             ; point to the start of the program storage EEPROM.
                    ldy       #BASBEG             ; point to the start of the program pointer storage area.
                    ldb       #4                  ; number of words to move.
                    stb       COUNT               ; save the count.
CELOAD3             ldd       ,x                  ; get the offset that was saved.
                    addd      RAMStart            ; add the starting address of the RAM to it.
                    std       ,y                  ; save the resulting pointer
                    inx                           ; point to the next offset.
                    inx
                    iny                           ; point to the next pointer in RAM
                    iny
                    dec       COUNT               ; have we gotten all the pointers yet?
                    bne       CELOAD3             ; no. keep going.

                    ldd       ,x                  ; yes. get the high line number.
                    std       ,y                  ; save it in RAM.

          ; now load the actual program from EEPROM

                    ldx       EEStart             ; point to the start of the EEPROM
                    ldy       BASBEG              ; point to the start of the BASIC program buffer.
CELOAD4             lda       SSTART,x            ; get a byte of the program.
                    sta       ,y                  ; put it in the program buffer.
                    inx                           ; point to the next program byte
                    iny                           ; point to the next buffer location.
                    cpy       VAREND              ; have we finished loading the program.
                    bls       CELOAD4             ; no. keep loading.
                    sty       STRASTG             ; yes. initialize the array storage area.
                    rts                           ; RETURN.

CLLIST              equ       *
                    lda       #$01                ; USE DEVICE #1 FOR HARD COPY LISTING.
                    sta       DEVNUM
                    jsr       CLIST               ; GO DO A STANDARD LIST COMMAND.
                    clr       DEVNUM
                    rts                           ; RETURN.

CAUTOST             equ       *                   ; SET AUTO START MODE FOR BASIC PROGRAM.
                    lda       #$55                ; GET FLAG.
CAUTOST1            ldx       EEStart
                    sta       AUTOSTF,x           ; PROGRAM IT INTO THE EEPROM
                    jsr       DLY10MS             ; WAIT WHILE IT PROGRAMS.
                    rts                           ; RETURN.

CNOAUTO             equ       *
                    lda       #$FF
                    bra       CAUTOST1

AUTOLOAD            equ       *
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
                    bra       CELOAD4

CFREE               equ       *
                    jsr       NL2
                    ldd       VARMEND
                    subd      STRASTG
                    jsr       OUTDECI
                    jsr       NL
                    rts

CDUMP               equ       *
                    rts

#ifdef ;------------------------------------------------------------------------
                    jsr       NL2                 ; PRINT TWO BLANK LINES.
                    clr       DNAME+2             ; ZERO THE LAST BYTE OF THE VARIABLE NAME 'ARRAY'
                    ldx       VARBEGIN            ; POINT TO THE START OF THE VARIABLE TABLE.
CDUMP2              lda       ,x                  ; GET AN ENTRY. IS IT THE END OF THE TABLE?
                    bne       CDUMP3              ; YES. WE'RE DONE.
                    rts

CDUMP3              lda       1,x                 ; NO. GET THE FIRST CHARACTER OF THE NAME.
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
                    bra       CDUMP2

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
                    inx
                    inx
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
                    page

RREM                equ       *                   ; NON-EXECUTIBLE STATEMENT JUST SKIP IT.
RDATA               equ       *
                    ldb       ,y                  ; GET LENGTH OF REMARK OR DATA LINE.
                    aby                           ; POINT TO THE EOLTOK.
                    rts                           ; RETURN.

RLET                equ       *
                    lda       ,y                  ; GET VARIABLE FLAG.
                    bsr       RVARPTR             ; GET POINTER TO ASIGNMENT VARIABLE.
                    pshd                          ; SAVE POINTER TO VARIABLE.
                    iny                           ; PUT IP PAST THE "=" TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET VALUE INTO D.
                    pulx                          ; POINT TO THE DICTIONARY ENTRY.
                    std       ,x                  ; STORE VALUE.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

RVARPTR             lda       ,y                  ; GET VARIABLE FLAG.
                    bita      #$02                ; IS IT A STRING VARIABLE?
                    bne       RVARPTR2            ; YES. GO GET POINTER FOR A STRING DESCRIPTOR.
                    bita      #$10                ; IS IT A NUMERIC ARRAY VARIABLE?
                    bne       RVARPTR1            ; YES. GO CALCULATE THE SUBSCRIPT.
RVARPTR3            ldd       1,Y                 ; GET THE OFFSET TO THE DICTIONARY ENTRY.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE DCTIONARY.
                    addd      #3                  ; MAKE POINTER POINT TO THE ACTUAL STORAGE LOCATION
                    pshb                          ; SAVE B.
                    ldb       #3                  ; POINT TO THE FIRST ELEMENT PAST THE VARIABLE.
                    aby
                    pulb                          ; RESTORE B.
                    rts

RVARPTR1            equ       *
                    jsr       CALCSUB             ; GO GET BASE ADDR & EVALUATE SUBSCRIPT EXPRESSION.
                    pshx                          ; PUSH BASE ADDRESS ONTO STACK.
                    tsx                           ; POINT TO IT.
                    lsld                          ; MULT THE SUBSCRIPT BY THE # OF BYTES/ELEMENT.
RVARPTR4            addd      ,x                  ; GET ADDRESS OF ELEMENT.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

RVARPTR2            equ       *
                    bita      #$10                ; IS IT A STRING ARRAY?
                    beq       RVARPTR3            ; NO. JUST GO GET POINTER TO DESCRIPTOR.
                    jsr       CALCSUB             ; GET BASE ADDR. & CALC SUBSCRIPT.
                    pshx                          ; SAVE THE BASE ADDRESS.
                    pshd                          ; SAVE THE SUBSCRIPT VALUE.
                    tsx                           ; POINT TO THE VALUES.
                    lsld                          ; MULT BY 2.
                    addd      ,x                  ; MULT BY 3.
                    ins                           ; GET RID OF SUBSCRIPT VALUE.
                    ins
                    tsx                           ; POINT TO BASE ADDRESS.
                    bra       RVARPTR4

RGOTO               equ       *
                    tst       IMMID               ; DID WE ENTER HERE FROM THE IMMIDIATE MODE?
                    beq       RGOTO7              ; NO. JUST GO DO A GOTO.
                    ldd       BASEND              ; YES. SET ADRNXLIN TO END OF PROGRAM SO THE
                    std       ADRNXLIN            ; SEARCH STARTS AT THE FIRST LINE.
RGOTO7              ldx       ADRNXLIN            ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; IS THIS THE LAST LINE OF THE PROGRAM?
                    bne       RGOTO1              ; NO. SEARCH STARTING AT THE NEXT LINE.
RGOTO3              ldx       BASBEG              ; YES. POINT TO THE START OF THE BASIC PROGRAM.
                    bra       RGOTO2

RGOTO1              ldd       ,x                  ; GET THE NEXT LINE NUMBER IN THE PGM.
                    cpd       1,Y                 ; IS IT > THE LINE NUMBER WE ARE TO "GOTO"?
                    bhi       RGOTO3              ; YES. START THE SEARCH AT THE BEGINING.
RGOTO2              ldd       ,x                  ; GET THE NEXT LINE NUMBER INTO D.
                    cpd       1,Y                 ; IS THIS THE CORRECT LINE?
                    beq       RGOTO4              ; YES. "GOTO" THE NEW LINE.
                    blo       RGOTO5              ; NO. IS IT LESS THAN THE "TARGET LINE"?
RGOTO6              lda       #LNFERR             ; NO. THE LINE MUST NOT EXIST.
                    jmp       RPTRERR             ; REPORT THE ERROR & RETURN TO MAIN LOOP.

RGOTO5              ldb       2,x                 ; GET THE LENGTH OF THIS LINE.
                    abx                           ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; DID WE HIT THE END OF THE PROGRAM?
                    beq       RGOTO6              ; YES. THE LINE DOESN'T EXIST.
                    bra       RGOTO2              ; NO. GO SEE IF THIS IS THE CORRECT LINE.

RGOTO4              xgdx                          ; MAKE IT THE NEW IP.
                    xgdy
                    tst       IMMID
                    beq       RGOTO8
                    clr       IMMID
RGOTO9              jmp       CRUN1

RGOTO8              ins
                    ins
                    bra       RGOTO9

RGOSUB              equ       *
                    pshy                          ; SAVE THE I.P. TO THE LINE NUMBER.
                    tst       IMMID               ; DID WE GET HERE FROM THE IMMIDIATE MODE?
                    beq       RGOSUB3             ; NO. GO DO A NORMAL GOSUB.
                    ldy       BASEND              ; YES. MAKE RETURN POINT TO THE LAST EOL TOKEN
                    dey                           ; IN THE PROGRAM.
                    bra       RGOSUB2             ; GO PUT IT ON THE ARGUMENT STACK.

RGOSUB3             ldb       #3                  ; BYPASS THE LINE NUMBER.
                    aby
RGOSUB2             jsr       RSKIPSPC            ; SKIP SPACES AFTER THE LINE NUMBER.
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    dex                           ; POINT TO THE NEXT ENTRY ON THE STACK.
                    dex
                    cpx       EGOSTK              ; OUT OF STACK SPACE?
                    bhs       RGOSUB1             ; NO. GO PUSH THE "RETURN ADDRESS" ON THE STACK.
                    lda       #GOSOVERR           ; YES. GET THE ERRCODE.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

RGOSUB1             stx       GOSTACK             ; SAVE THE "GOSUB" STACK POINTER.
                    sty       ,x                  ; PUT THE RETURN ADDRESS ON THE STACK.
                    puly                          ; GET THE POINTER TO THE LINE NUMBER.
                    jmp       RGOTO               ; GO DO A "GOTO".

RRETURN             proc
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    cpx       STGOSTK             ; IS THERE A RETURN ADDRESS ON THE GOSUB STACK?
                    bne       Go@@                ; YES. GO RETURN.
                    lda       #RWOGERR            ; NO. RETURN W/O GOSUB ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

Go@@                ldy       ,x                  ; GET THE RETURN ADDRESS IN THE IP.
                    inx                           ; REMOVE THE ADDRESS FROM THE STACK.
                    inx
                    stx       GOSTACK             ; SAVE THE STACK POINTER.
                    rts                           ; BACK TO THE MAIN INTERPRET LOOP.

;*******************************************************************************

RSTOP               proc
                    ldx       #STOPSTR
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    sty       IPSAVE
                    bra       REND1

;*******************************************************************************

REND                proc
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
REND1               ldd       #0
                    std       CURLINE
                    jmp       MAINW

STOPSTR             fcs       LF,CR,"STOPPED AT LINE # "

;*******************************************************************************

RWHILE              proc
                    ldx       WHSTACK             ; GET THE WHILE STACK POINTER.
                    dex                           ; POINT TO THE NEXT STACK LOCATION.
                    dex
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
                    pshb                          ; BugFix: (2017.10.17 <tonyp@acm.org>)
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
                    inx                           ; TAKE ADDRESS OFF OF WHILE STACK.
                    inx
                    stx       WHSTACK             ; SAVE STACK POINTER.
                    bra       Done@@              ; GO TO INTERPRET LOOP.

Exec@@              ins                           ; REMOVE POINTER TO STATEMENT AFTER "ENDWH"
                    ins                           ; FROM STACK.
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
                    bpl       RON1                ; IS IT NEGATIVE?
                    bne       RON1                ; OR ZERO?
RON5                lda       #ONARGERR           ; YES. REPORT ERROR.
                    jmp       RPTRERR

RON1                ldd       ,x                  ; GET THE EXPRESSION VALUE.
                    subd      #1                  ; SUBTRACT 1. HAVE WE FOUND THE LINE NUMBER?
                    beq       RON4                ; YES. GO DO "GOTO" OR "GOSUB".
                    std       ,x                  ; NO. SAVE REMAINDER.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER VALUE.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES PAST THE LINE NUMBER.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #EOLTOK             ; HAVE WE HIT THE END OF THE LINE?
                    beq       RON5                ; YES. ERROR.
RON3                iny                           ; NO. MUST BE A COMMA. BYPASS IT.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    bra       RON1                ; GO SEE IF THE NEXT LINE NUMBER IS THE ONE.

RON4                jsr       PULNUM              ; GET RID OF ARGUMENT.
                    pula                          ; GET "GO" TOKEN.
                    cmpa      #GOTOTOK            ; IS IT A "GOTO" TOKEN?
                    bne       RON6                ; NO. MUST BE A "GOSUB"
                    jmp       RGOTO               ; GO DO A "GOTO".

RON6                pshy                          ; SAVE THE POINTER TO THE LINE NUMBER.
RON8                ldb       #3                  ; POINT PAST THE LINE NUMBER.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER LINE NUMBER.
                    lda       ,y                  ; GET NEXT TERMINATOR CHARACTER.
                    cmpa      #EOLTOK             ; HIT THE END OF THE LINE YET?
                    beq       RON7                ; YES. GO DO THE GOSUB.
                    cmpa      #MEOLTOK            ; NO. HIT THE LOGICAL END OF THE LINE YET?
                    beq       RON7                ; YES. GO DO THE GOSUB.
                    iny                           ; NO. MUST BE A COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    bra       RON8                ; GO FIND THE END OF THE LINE.

RON7                jmp       RGOSUB2             ; GO DO A "GOSUB".

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
RPORTA1             ldx       IOBaseV             ; GET ADDRESS OF PORTA I/O REGISTER.
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
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTC              proc
                    ldb       #PORTCIO            ; GET ADDRESS OF PORTC I/O REGISTER.
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTD              proc
                    ldb       #PORTDIO            ; GET ADDRESS OF PORTD I/O REGISTER.
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.
;                   include   'runtime2.mod'
                    title     RUNTIME2
                    page

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
                    beq       RPRINT1             ; YES. JUST PRINT A CR/LF.
                    cmpa      #MEOLTOK            ; IS IT A MID EOL TOKEN?
                    bne       RPRINT2             ; NO. GO PRINT A STRING OR NUMBER.
RPRINT1             jsr       NL                  ; YES. JUST PRINT A CR/LF.
                    clr       DEVNUM              ; GO BACK TO DEVICE #0.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

RPRINT2             cmpa      #SCONTOK            ; IS IT A STRING CONSTANT?
                    bne       RPRINT3             ; NO. GO CHECK FOR A "PRINT FUNCTION".
                    pshy
                    ldb       #2                  ; COMPENSATE FOR CONSTANT & LENGTH BYTE.
                    addb      1,Y                 ; ADD IN LENGTH BYTE.
                    aby                           ; POINT BEYOND PROMPT.
                    pulx                          ; GET POINTER INTO X.
                    inx                           ; POINT TO LENGTH BYTE.
                    ldb       ,x                  ; GET IT.
                    subb      #2                  ; SUBTRACT OUT THE DELIMETER COUNT.
                    inx                           ; POINT TO STRING.
                    inx
                    jsr       OUTSTR              ; GO PRINT THE STRING.
                    bra       RPRINT4             ; GO DO NEXT EXPRESSION.

RPRINT3             cmpa      #FUNCTFLG           ; IS IT A FUNCTION?
                    bne       RPRINT10            ; NO. GO EVALUATE A NUMERIC EXPRESSION.
                    lda       1,Y                 ; GET THE FUNCTION TYPE.
                    cmpa      #TABTOK             ; IS IT A TAB?
                    bne       RPRINT11            ; NO GO CHECK FOR "CHR$".
                    jsr       RTAB                ; GO DO TAB.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT11            cmpa      #CHRTOK             ; IS IT THE CHR$ FUNCTION.
                    bne       RPRINT12            ; NO. GO CHECK FOR HEX().
                    jsr       RCHRS               ; YES. GO DO CHR$.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT12            cmpa      #HEXTOK             ; IS IT THE HEX() FUNCTION?
                    bne       RPRINT10            ; NO. GO DO A NUMERIC EXPRESSION.
                    jsr       RHEX                ; YES. GO PRINT THE NUMBER AS HEX.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT10            cmpa      #HEX2TOK            ; IS IT THE HEX2() FUNCTION?
                    bne       RPRINT14            ; NO. GO DO A NUMERIC EXPRESSION.
                    jsr       RHEX2               ; YES GO PRINT A NUMBER >=255 AS 2 HEX BYTES.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT14            jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE NUMERIC STACK.
                    jsr       OUTDECI             ; PRINT IT.
                    lda       #SPC                ; PUT A TRAILING SPACE AFTER ALL NUMBERS.
                    jsr       OUTBYTE             ; PRINT IT.
RPRINT4             jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET SEPERATOR CHARACTER.
                    cmpa      #COMMATOK           ; IS IT A COMMA?
                    beq       RPRINT5             ; NO.
                    cmpa      #SEMITOK            ; IS IT A SEMICOLON?
                    bne       RPRINT6             ; NO. MUST BE AN EOLTOK.
                    iny                           ; DO NOTHING BUT BUMP THE IP.
                    bra       RPRINT7             ; GO CHECK FOR EOL AFTER COMMA OR SEMICOLON.

RPRINT5             iny                           ; BUMP IP PAST THE COMMATOK.
                    ldb       PRINTPOS            ; YES. "TAB" TO NEXT PRINT FIELD.
                    andb      #$07                ; MASK OFF ALL BUT THE FIELD WIDTH.
                    negb                          ; MAKE IT NEGATIVE.
                    addb      #8                  ; ADD IN THE FIELD WIDTH. ARE WE ON A FIELD BOUND?
                    beq       RPRINT7             ; YES. GO CHECK FOR AN EOL.
                    lda       #SPC                ; NO. GET A SPACE & PRINT TILL WE GET THERE.
RPRINT8             jsr       OUTBYTE             ; PRINT A SPACE.
                    decb                          ; DECREMENT THE COUNT. ARE WE DONE?
                    bne       RPRINT8             ; NO. KEEP GOING.
RPRINT7             jsr       RSKIPSPC            ; SKIP ANY SPACES.
                    lda       ,y                  ; GET THE NEXT TOKEN IN THE LINE.
                    cmpa      #EOLTOK             ; IS IT AN EOL TOKEN?
                    beq       RPRINT9             ; YES. DONT DO A CR/LF AFTER A COMMA OR SEMI.
                    cmpa      #MEOLTOK            ; NO. IS IT A MID EOL?
                    beq       RPRINT9             ; SAME AS BEFORE.
                    jmp       RPRINT2             ; IF NEITHER, GO PRINT THE NEXT EXPRESSION.

RPRINT6             jsr       NL                  ; DO A CR/LF IF EOL OR MIDEOL FOLLOWS EXPRESSION.
RPRINT9             clr       DEVNUM              ; GO BACK TO DEVICE #0.
                    rts                           ; GO DO NEXT LINE.

;*******************************************************************************

RTAB                proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       Loop@@              ; YES. GO DO TAB.
                    lda       #TABARGER           ; NO. ERROR.
RTAB3               jmp       RPTRERR             ; REPORT ERROR.

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
                    bra       RTAB3               ; REPORT ERROR.

Go@@                tba                           ; PUT BYTE INTO A
                    jmp       OUTBYTE             ; PRINT THE BYTE & RETURN.

;*******************************************************************************

RHEX2               proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       RHEX1               ; YES. GO PRINT 2 HEX CHARACTERS & RETURN.
                    lda       #HEX2AERR           ; NO. ARG. MUST BE >=0 & <=255.
                    bra       RTAB3               ; GO REPORT ERROR.

;*******************************************************************************

RHEX                proc
                    bsr       PFUNCOM             ; GO DO COMMON CODE FOR PRINT FUNCTIONS
                    bsr       PRNT2HEX            ; GO PRINT 2 HEX CHARACTERS.
RHEX1               tba                           ; PUT LOWER BYTE IN A.
;                   bra       PRNT2HEX

;*******************************************************************************

PRNT2HEX            proc
                    psha                          ; SAVE THE CHARACTER.
                    bsr       Msb@@               ; PRINT THE LEFT HEX NIBBLE.
                    pula                          ; GET BYTE BACK.
                    bra       Lsb@@               ; PRINT RIGHT NIBBLE & RETURN.

Msb@@               lsra                          ; GET UPPER NIBBLE INTO LOWER ONE.
                    lsra
                    lsra
                    lsra
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

Go@@                ldd       1,Y                 ; GET THE OFFSET INTO THE DICTIONARY.
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
                    addd      #1                  ; COMPENSATE FOR "0" SUBSCRIPT.
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
                    inx
                    inx                           ; POINT PAST THE SUBSCRIPT LIMIT.
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
                    page

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
                    addb      1,Y                 ; ADD IN LENGTH BYTE.
                    aby                           ; POINT BEYOND PROMPT.
                    pulx                          ; GET POINTER INTO X.
                    inx                           ; POINT TO LENGTH BYTE.
                    ldb       ,x                  ; GET IT.
                    subb      #2                  ; SUBTRACT OUT THE DELIMETER COUNT.
                    inx                           ; POINT TO STRING.
                    inx
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

Msg@@               fcs       "? "

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
                    bra       INNUM3

Decimal@@           jsr       INDECI
INNUM3              equ       *
                    pshd
                    jsr       SKIPSPCS
                    cmpa      #COMMA
                    beq       INNUM4
                    cmpa      #EOL
                    beq       INNUM7
                    lda       #MCOMAERR
                    jmp       RPTRERR

INNUM4              jsr       INCIBP
INNUM7              jsr       RVARPTR
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
                    bne       RREAD1              ; YES. CONTINUE TO READ DATA.
                    bsr       RRESTOR             ; NO. GO GET POINTER TO FIRST DATA STATEMENT.
                    ldx       DATAPTR             ; GET POINTER TO DATA.
RREAD1              stx       IBUFPTR             ; PUT IT IN THE INPUT BUFFER POINTER.
                    jsr       RINRDC              ; GO USE INPUT/READ COMMON CODE.
                    bcs       RREAD2              ; IF CARRY SET, MORE DATA TO READ.
                    ldx       IBUFPTR             ; GET POINTER TO DATA LINE.
                    stx       DATAPTR             ; SAVE DATA POINTER FOR NEXT READ.
                    rts                           ; RETURN.

RREAD2              pshy                          ; SAVE Y.
                    ldy       IBUFPTR
                    iny
                    iny
                    bsr       RESTOR4             ; GO FIND NEXT "DATA" STATEMENT.
                    puly                          ; RESTORE Y.
                    bra       RREAD               ; KEEP READING DATA.

;*******************************************************************************

RRESTOR             proc
                    pshy                          ; SAVE Y.
                    ldy       BASBEG              ; START SEARCH FOR "DATA" STATEMENTS AT THE BEGIN.
Loop@@              pshy                          ; SAVE POINTER TO THIS LINE.
                    ldb       2,Y                 ; GET LINE LENGTH.
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

Done@@              iny                           ; POINT PAST DATA TOKEN & THE DATA LENGTH.
                    iny
                    sty       DATAPTR             ; SAVE POINTER TO DATA.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

RESTOR4             pshy                          ; CALL TO COMPENSATE FOR PULL OF Y ON RETURN.
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
                    iny                           ; PASS UP THE CLOSING PAREN.
                    iny                           ; PASS UP THE EQUALS TOKEN.
                    jsr       DONEXP              ; GET VALUE TO FROGRAM INTO EEPROM.
                    pshy                          ; SAVE THE Y REG.
                    ldy       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    ldd       2,Y                 ; GET THE SUBSCRIPT FOR THE EEPROM LOCATION.
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
Write@@             lda       1,Y                 ; GET LS BYTE OF WORD.
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
                    stb       PPROG,Y             ; VOLTAGE OFF.
                    sta       ,x                  ; LATCH ADDRESS.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE ERASING.
                    ldb       #$17                ; TURN ON ERASE VOLTAGE
                    stb       PPROG,Y
                    bsr       DLY10MS             ; DELAY ABOUT 10 MS.
                    ldb       #$16                ; TURN PROGRAMING VOLTAGE OFF.
                    stb       PPROG,Y
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG,Y
                    puly
                    rts                           ; RETURN.

;*******************************************************************************

PROGBYTE            proc
                    pshy
                    ldy       IOBaseV             ; Point to the base address of the I/O Registers.
Loop@@              ldb       #$02                ; SET UP NORMAL PROGRAMING MODE, ADDRESS/DATA
                    stb       PPROG,Y             ; LATCHED, PROGRAMING VOLTAGE OFF.
                    sta       ,x                  ; LATCH DATA & ADDRESS.
                    psha                          ; SAVE THE DATA FOR COMPARE AFTER PROGRAMING.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE PROGRAMING.
                    ldb       #$03                ; TURN ON PROGRAMING VOLTAGE.
                    stb       PPROG,Y
                    bsr       DLY10MS             ; LEAVE IT ON FOR 10 MS.
                    ldb       #$02                ; NOW, TURN THE PROGRAMMING VOLTAGE OFF.
                    stb       PPROG,Y
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG,Y             ; PUT THE EEPROM BACK IN THE READ MODE.
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
                    clra
                    clrb
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
                    bsr       CHCKIMID            ; NOT ALLOWED IN IMMIDIATE.
                    jsr       DONEXP              ; GO EVALUATE THE TIME "MATCH" EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMECMP             ; PUT IN THE COMPARE REGISTER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; PASS UP COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    sty       ONTIMLIN            ; SAVE THE POINTER TO THE LINE NUMBER.
                    bra       RONIRQ2             ; GO FINISH UP.

;*******************************************************************************

RONIRQ              proc
                    bsr       CHCKIMID
                    jsr       DONEXP              ; GO CHECK TO SEE IF WE ARE TO ENABLE OR DISABLE.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO COMMA.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO LINE NUMBER.
                    jsr       PULNUM              ; GET MODE. SHOULD WE ENABLE THE FUNCTION?
                    bne       Enable@@            ; YES.
                    std       ONIRQLIN            ; NO. MAKE THE LINE NUMBER 0.
                    bra       RONIRQ2             ; GO FINISH UP.

Enable@@            sty       ONIRQLIN            ; SAVE THE POINTER TO THE LINE NUMBER,
RONIRQ2             ldb       #3                  ; MOVE IP PAST THE LINE NUMBER.
                    aby
                    rts                           ; RETURN.

;*******************************************************************************

RRETI               proc
                    bsr       CHCKIMID
                    tpa                           ; CHECK TO SEE IF THE INTERRUPT MASK IS SET.
                    bita      #$10                ; ARE WE IN AN INTERRUPT ROUTINE?
                    bne       Go@@                ; SINCE THE IRQ MASK IS SET WE MUST BE.
                    lda       #NOTINTER           ; NO. FLAG AN ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

Go@@                ldd       SCURLINE            ; RESTORE THE MAIN PROGRAM CURRENT LINE.
                    std       CURLINE
                    ldd       SADRNXLN            ; RESTORE MAIN PROGRAM "ADDRESS OF THE NEXT LINE".
                    std       ADRNXLIN
                    ins                           ; TAKE THE RETURN ADDRESS OFF THE STACK.
                    ins
                    rti                           ; GO BACK TO WHERE WE LEFT OFF.

;*******************************************************************************

CHCKIMID            proc
                    tst       IMMID               ; ARE WE IN THE IMMIDIATE MODE?
                    beq       Done@@              ; NO. JUST RETURN.
                    lda       #NOTALERR           ; YES. THIS COMMAND NOT ALLOWED.
                    jmp       RPTRERR             ; REPORT THE ERROR.
Done@@              rts                           ; RETURN.

;*******************************************************************************

RONPACC             proc
                    bsr       CHCKIMID            ; THIS INSTRUCTION NOT ALLOWED IN IMMID MODE.
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
                    bls       RONPACC2            ; YES. ARG. OK.
Fail@@              lda       #INTMODER           ; NO. GET ERROR CODE.
                    jmp       RPTRERR

RONPACC2            lda       #$10                ; GET BIT TO ENABLE INTERRUPT.
                    tstb                          ; WAS THE ARGUMENT 0?
                    beq       RONPACC3            ; YES. GO ENABLE INTS. ON EACH COUNT.
                    lsla                          ; NO. ENABLE INTS. ON PACC OVERFLOW ONLY.
RONPACC3            ldx       IOBaseV
                    sta       TMSK2,x
                    jsr       PULNUM              ; GET THE COUNT MODE OFF THE STACK.
                    bne       RONPACC4            ; GO SET THE MODE IF NOT 0.
                    ldx       IOBaseV
                    clr       PACTL,x             ; TURN OFF THE PULSE ACCUMULATOR.
                    std       ONPACLIN            ; CLEAR POINTER TO LINE NUMBER.
                    bra       Done@@              ; GO CLEAN UP & RETURN.

RONPACC4            cpd       #4                  ; IS THE ARGUMENT IN RANGE?
                    bhi       Fail@@              ; YES. REPORT AN ERROR.
                    addb      #3                  ; GET BIT TO ENABLE PACC.
                    lslb
                    lslb
                    lslb
                    lslb
                    ldx       IOBaseV
                    stb       PACTL,x             ; ENABLE THE PACC & SET MODE.
Done@@              pula                          ; GET OLD I-BIT STATUS OFF STACK.
                    tap                           ; RESTORE OLD STATUS.
                    ldb       #3
                    aby                           ; PASS UP LINE NUMBER.
                    rts                           ; RETURN.
;                   include   'rexpres.mod'
                    title     REXPRES
                    page
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
                    bne       PSHNUM1             ; NO. GO CHECK FOR CONSTANT.
                    ldd       1,Y                 ; YES. GET THE "OFFSET" ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE VARIABLE TABLE.
                    xgdx                          ; GET THE ADDRESS INTO X.
                    ldb       #$03                ; BUMP INTERPRETER POINTER PAST "VARIABLE".
                    aby
                    ldd       3,x                 ; GET THE VARIABLE VALUE.
                    bra       PSHNUM4             ; GO PUT IT ON THE STACK.

PSHNUM1             cmpa      #ICONTOK            ; IS IT AN INTEGER CONSTANT?
                    bne       PSHNUM2             ; NO. GO CHECK FOR AN INTEGER ARRAY VARIABLE.
                    ldx       1,Y                 ; GET THE CONSTANT VALUE INTO X.
                    ldb       #$04
                    addb      3,Y
                    aby
                    xgdx                          ; PUT THE CONSTANT VALUE INTO D.
                    bra       PSHNUM4             ; GO PUT IT ON THE STACK.

PSHNUM2             cmpa      #IAVARTOK           ; IS IT AN INTEGER ARRAY?
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
                    bra       PSHNUM4

Fail@@              lda       #ILTOKERR
                    jmp       RPTRERR

PSHNUM4             ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    dex                           ; MAKE ROOM ON THE STACK FOR NEW OPERAND.
                    dex
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
                    ldd       1,Y                 ; GET THE VARIABLE OFFSET ADDRESS.
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

Done@@              inx                           ; BYPASS THE SUBSCRIPT LIMIT.
                    inx
                    rts

;*******************************************************************************

PULNUM              proc
                    pshx                          ; SAVE THE X-REG.
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    ldd       ,x                  ; GET THE OPERAND.
                    inx                           ; BUMP THE STACK POINTER.
                    inx
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

Go@@                lda       1,Y                 ; GET THE FUNCTION CODE BYTE IN B.
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
                    inx                           ; YES. KNOCK BOTH OPERATORS OFF THE STACK.
                    inx
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
                    blo       DOOP1               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$70                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR7              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

DOOP1               cmpa      #$60                ; IS IT THE "^" OPERATOR?
                    blo       DOOP2               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$60                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR6              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

DOOP2               cmpa      #$50                ; IS IT MULTIPLY, DIVIDE, OR MOD?
                    blo       DOOP3               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$50                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR5              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

DOOP3               cmpa      #$40                ; IS IT ADD OR SUBTRACT?
                    blo       DOOP4               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$40                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR4              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

DOOP4               cmpa      #$30                ; IS IT A LOGICAL OPERATOR?
                    blo       DOOP5               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$30                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR3              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

DOOP5               cmpa      #$20                ; IS IT AND, OR, OR EOR?
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

HEIR7               equ       *
                    fdb       RINDIR
                    fdb       RNOT
                    fdb       RNEG
HEIR6               equ       *
                    fdb       RPWR
HEIR5               equ       *
                    fdb       RMULT
                    fdb       RDIV
                    fdb       RMOD
HEIR4               equ       *
                    fdb       RPLUS
                    fdb       RMINUS
HEIR3               equ       *
                    fdb       RLT
                    fdb       RGT
                    fdb       RLTEQ
                    fdb       RGTEQ
                    fdb       REQ
                    fdb       RNOTEQ
HEIR2               equ       *
                    fdb       RAND
                    fdb       RORV
                    fdb       REOR

;*******************************************************************************

REOR                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    eora      ,x
                    eorb      1,x
REOR1               std       ,x
                    rts

;*******************************************************************************

RORV                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    ora       ,x
                    orb       1,x
                    bra       REOR1

;*******************************************************************************

RAND                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    anda      ,x
                    andb      1,x
                    bra       REOR1

;*******************************************************************************

RPLUS               proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    addd      ,x
                    bra       REOR1

;*******************************************************************************

RMINUS              proc
                    ldx       NUMSTACK
                    ldd       2,x
                    subd      ,x
                    inx
                    inx
                    std       ,x
                    stx       NUMSTACK
                    rts

;*******************************************************************************

RDIV                bsr       RDIVS               ; GO DO A SIGNED DIVIDE.
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
                    bne       RDIV1               ; NO. CONTINUE.
RDIV2               lda       #ZDIVERR            ; YES. GET DIVIDE BY ZERO ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

RDIV1               bpl       RDIV3               ; IF POSITIVE IT'S OK.
                    jsr       RNEG                ; IF NOT MAKE IT POSITIVE.
RDIV3               tst       2,x                 ; IS THE DIVIDEND NEGATIVE?
                    bpl       RDIV4               ; NO. CONTINUE.
                    ldd       2,x                 ; YES. GET THE NUMBER.
                    negd                          ; NEGATE IT.
                    std       2,x                 ; SAVE THE RESULT.
RDIV4               ldd       ,x                  ; GET THE DIVISOR.
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
                    bpl       RDIV5               ; IF PLUS, RESULT OK AS IS.
                    jsr       RNEG                ; MAKE THE QUOTIENT NEGATIVE.
                    ldd       2,x                 ; GET THE REMAINDER.
                    negd                          ; MAKE IT NEGATIVE.
                    std       2,x                 ; SAVE THE RESULT.
RDIV5               rts                           ; RETURN.

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
                    inx
                    inx
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
                    bge       RLT1
RLT2                inc       3,x
RLT1                inx
                    inx
                    stx       NUMSTACK
                    rts

;*******************************************************************************

RGT                 proc
                    bsr       CMPNUM
                    ble       RLT1
                    bra       RLT2

;*******************************************************************************

RLTEQ               proc
                    bsr       CMPNUM
                    bgt       RLT1
                    bra       RLT2

;*******************************************************************************

RGTEQ               proc
                    bsr       CMPNUM
                    blt       RLT1
                    bra       RLT2

;*******************************************************************************

REQ                 proc
                    bsr       CMPNUM
                    bne       RLT1
                    bra       RLT2

;*******************************************************************************

RNOTEQ              proc
                    bsr       CMPNUM
                    beq       RLT1
                    bra       RLT2

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
                    bpl       RABS1
RABS2               negd
RABS1               std       ,x
                    rts

;*******************************************************************************

RSGN                proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    beq       RABS1
                    ldd       #1
                    tst       ,x
                    bpl       RABS1
                    bra       RABS2

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
                    bls       RFEEP1              ; YES. GO GET THE VALUE.
                    lda       #EESUBERR           ; NO. SUBSCRIPT ERROR.
RFEEP2              jmp       RPTRERR             ; REPORT THE ERROR.

RFEEP1              lsld                          ; MULT THE SUBSCRIPT BY 2.
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
RFDIV2              bra       RFEEP2              ; GO REPORT IT.

Go@@                xgdx                          ; PUT QUOTIENT IN D.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    inx                           ; REMOVE DIVISOR FROM STACK.
                    inx
                    std       ,x                  ; PUT QUITIENT ON OPERAND STACK.
                    stx       NUMSTACK            ; SAVE NEW VALUE OF STACK POINTER.
                    rts                           ; RETURN.

;*******************************************************************************

RADC                proc
                    jsr       DONEXP              ; GO GET THE CHANNEL NUMBER TO CONVERT.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    ldd       ,x                  ; GET THE CHANNEL NUMBER.
                    bmi       RADC4               ; NEGATIVE CHANNEL NUMBERS ARE ILLEGAL.
                    cpd       #7                  ; IS IT A VALID CHANNEL NUMBER?
                    bls       RADC1               ; YES. GO CONVERT IT.
RADC4               lda       #INVCHERR           ; NO. INVALID CHANNEL NUMBER.
                    bra       RFDIV2              ; GO REPORT THE ERROR.

RADC1               ldx       IOBaseV
                    stb       ADCTL,x             ; START THE CONVERSION ON THE SELECTED.
RADC2               tst       ADCTL,x             ; IS THE CONVERSION COMPLETE?
                    bpl       RADC2               ; NO. WAIT FOR 4 CONVERSIONS ON 1 CHANNEL.
                    clra                          ; YES. NOW AVERAGE THE 4 CONVERSIONS.
                    ldb       ADR1,x              ; GET 1ST RESULT.
                    addb      ADR2,x              ; ADD IN THE SECOND.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR3,x              ; ADD IN THE THIRD.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR4,x              ; ADD IN THE FOURTH.
                    adca      #0                  ; ADD IN CARRY.
                    lsrd                          ; DIVIDE RESULT BY 4.
                    lsrd
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    std       ,x                  ; PUT THE RESULT ON THE OPERAND STACK.
                    rts                           ; RETURN.

RRND                equ       *
                    jsr       DONEXP              ; GO GET FUNCTION ARGUMENT.
                    ldx       NUMSTACK            ; GET ARGUMENT OFF STACK. GET NEW RANDOM NUMBER?
                    ldd       ,x
                    beq       RRND2               ; YES. GO GET NEXT RANDOM NUMBER IN THE SERIES.
                    bmi       RRND1               ; IF NEG., START A NEW SERIES.
                    ldd       RANDOM              ; IF POSITIVE, GET LAST RANDOM NUMBER.
                    bra       RRND3               ; RETURN.

RRND1               ldx       IOBaseV
                    ldd       TCNT,x              ; USE THE TIMER VALUE AS THE NEW SEED.
                    std       RANDOM              ; SAVE IT.
RRND2               ldd       RANDOM              ; GET PREVIOUS RANDOM NUMBER (USE AS SEED).
                    aslb                          ; DO SOME OPERATIONS.
                    aba
                    ldb       RANDOM+1
                    asld
                    asld
                    addd      RANDOM
                    addd      #$3619
                    std       RANDOM
RRND3               lsrd                          ; MAKE THE NUMBER POSITIVE.
                    std       ,x                  ; PUT THE NUMBER ON THE STACK.
                    rts                           ; RETURN.

ITAB                equ       *
ICHRS               equ       *
IHEX                equ       *
IHEX2               equ       *
                    lda       #PRFUNERR           ; THESE FUNCTIONS MUST BE USED ONLY IN
                    jmp       RPTRERR             ; PRINT STATEMENTS.

RFTIME              ldd       TIMEREG             ; GET THE TIME IN SECONDS.
                    bra       RFPORTA2            ; GO PUT NUMBER ON THE STACK.

RFPACC              ldx       IOBaseV
                    ldb       PACNT,x             ; GET THE CURRENT VALUE OF THE PULSE ACCUMULATOR.
                    clra
                    bra       RFPORTA2            ; GO PUT THE NUMBER ON THE STACK.

RFPORTA             equ       *
                    ldb       #PORTAIO            ; GET DATA FROM PORTA.
RFPORTA1            ldx       IOBaseV
                    abx
                    ldb       ,x
                    clra                          ; CLEAR UPPER BYTE OF WORD.
RFPORTA2            dey                           ; DECREMENT IP BECAUSE CALLING ROUTINE WILL TRY
                    dey                           ; TO BUMP IT PAST AN OPENING & CLOSING PAREN
                                                  ; WHICH ISN'T THERE.
                    jmp       PSHNUM4             ; GO PUSH VALUE ON OPERAND STACK & RETURN.

RFPORTB             equ       *
                    ldb       #PORTBIO
                    bra       RFPORTA1

RFPORTC             equ       *
                    ldb       #PORTCIO
                    bra       RFPORTA1

RFPORTD             equ       *
                    ldb       #PORTDIO
                    bra       RFPORTA1

RFPORTE             equ       *
                    ldb       #PORTEIO
                    bra       RFPORTA1
;                   opt       lis
;                   include   'iopkg.mod'
                    title     IOPKG
                    page

;*******************************************************************************

OUTBYTE             proc
                    inc       PRINTPOS            ; INCREMENT THE CURRENT PRINT POSITION.
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #OUTABLE            ; POINT TO THE OUTPUT VECTOR TABLE.
OUTBYTE1            ldb       DEVNUM              ; GET THE CURRENT DEVICE NUMBER.
                    aslb                          ; MULT BY 2.
                    abx                           ; POINT TO THE ADDRESS OF THE OUTPUT ROUTINE.
                    ldx       ,x                  ; GET THE ADDRESS. HAS THE VECTOR BEEN INITALIZED?
                    bne       Done@@              ; YES. GO OUTPUT THE CHARACTER.
                    clr       DEVNUM              ; NO. RESET TO DEVICE #0.
                    lda       #UNINIERR           ; GO REPORT AN UNINITALIZED I/O VECTOR ERROR.
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
                    #Error    "BASIC Is Too Large"
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
                    sta       XONCH               ; INITALIZE THE XON REGISTER.
                    lda       #$13                ; GET THE XOFF CHARACTER (CONTROL-S).
                    sta       XOFFCH              ; INITALIZE THE XOFF CHARACTER.
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
                    page
          #if * > $FFA0
                    #Fatal    "BASIC Is Too Large"
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
RAMStart            dw        $c000               ; starting address of system RAM.
RAMSize             dw        $2000               ; size of BASIC11 RAM Buffer.
EEStart             dw        $6000               ; starting address of program storage EEPROM
EESize              dw        $2000               ; size of the program storage EEPROM
IOBase              dw        $1000               ; Base Address of the I/O Registers
TimeVal             dw        62500               ; value used for generating 'Time' Interrupt
UserInit            dw        IODevInit           ; Used to initialize console/other hardware.
DFLOPADR            dw        $4000               ; Address of flip-flop used to connect the HC11 SCI
                                                  ; to the host port connector.

                    org       ROMBEG+ROMSIZE-{21*2}  ; START OF VECTOR TABLE.
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

?                   macro
                    #Hint     Verification~'..................................................................................................................................'.1.{:width-75}~ 8164 bytes, RAM:.. 208, CRC: $3FEB
                    endm

                    @?
