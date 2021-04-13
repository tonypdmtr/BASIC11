;*******************************************************************************
;* Program   : BASIC11M.ASM
;* Programmer: Tony Papadimitriou, MS <tonyp@acm.org>
;* Purpose   : Modified BASIC interpreter for the 68HC11
;* Language  : Motorola/Freescale/NXP 68HC11 Assembly Language (aspisys.com/ASM11)
;* Note(s)   : Based on BASIC interpreter for the 68HC11 by Gordon Doughman
;*******************************************************************************

                    #ListOff
                    #Uses     mcu.inc
                    #ListOn
                    #Macro

;*******************************************************************************
; Macros
;*******************************************************************************

Title               macro
                    mset      0,~@~
                    #Message  Processing \@~text~\@
                    endm

;*******************************************************************************

                    title     DEFINES

;-------------------------------------------------------------------------------
; HC11EVB defines
;-------------------------------------------------------------------------------

SWPRE               equ       2                   ; SOFTWARE PRESCALER VALUE.

;SBASBEG            equ       0
SBASEND             equ       2
SVARBEG             equ       4
SVAREND             equ       6
;SHILINE            equ       8

AUTOSTF             equ       10
SSTART              equ       11

;-------------------------------------------------------------------------------
; HC11 (device-dependant) defines
;-------------------------------------------------------------------------------

EEPBASAD            equ       $B600               ; EEPROM base address
MAXEESUB            equ       255                 ; maximum EEP subscript

;-------------------------------------------------------------------------------
; I/O Register Offsets From The Base Address
;-------------------------------------------------------------------------------

PPROG               def       $3B                 ; EEPROM programing control register
ADCTL               def       $30                 ; A-TO-D control/status register
ADR1                def       $31                 ; A/D result register 1
ADR2                def       $32                 ; A/D result register 2
ADR3                def       $33                 ; A/D result register 3
ADR4                def       $34                 ; A/D result register 4
PORTAIO             def       $00                 ; PORTA I/O register
PORTBIO             def       $04                 ; PORTB I/O register
PORTCIO             def       $03                 ; PORTC I/O register
PORTDIO             def       $08                 ; PORTD I/O register
PORTEIO             def       $0A                 ; PORTE I/O register
TCNT                def       $0E                 ; TIMER/COUNTER register
TOC1REG             def       $16                 ; TIMER Output Compare 1 register
TFLAG1              def       $23                 ; TIMER Flag #1 register
TMSK1               def       $22                 ; TIMER Mask #1 register
TMSK2               def       $24                 ; TIMER Mask #2 register
OPTION              def       $39                 ; OPTION select register
BAUD                def       $2B                 ; SCI baud rate select register
SCCR1               def       $2C                 ; SCI control register #1
SCCR2               def       $2D                 ; SCI control register #2
SCSR                def       $2E                 ; SCI status register
SCDR                def       $2F                 ; SCI transmit/recieve data register
PACNT               def       $27                 ; PACC count register
PACTL               def       $26                 ; PACC control register
TFLG2               def       $25                 ; TIMER Flag #2 register
INIT                def       $3D                 ; INIT (Base address of RAM & I/O Regs) Register

;-------------------------------------------------------------------------------
; Miscellaneous defines
;-------------------------------------------------------------------------------

EOL                 def       13                  ; end of line marker
CR                  def       13                  ; same as EOL
LF                  def       10                  ; linefeed character
BS                  def       8                   ; backspace character
SPC                 equ       32                  ; space character
MIDEOL              equ       ':'                 ; mid EOL character
COMMA               equ       ','                 ; comma
SEMI                equ       ';'                 ; semicolin
NUM                 equ       1                   ; getvar return flag
STRING              equ       2                   ; getvar return flag
NULL                def       0                   ; null value
CNTRLC              equ       3                   ; control-c (break character)

IBUFLEN             equ       80                  ; input buffer max length
TBUFLEN             equ       128                 ; token buffer max length
SWSTKSize           equ       592

OPSLEN              equ       30                  ; operator stack length
NUMSLEN             equ       60                  ; operand stack length
FORSLEN             equ       80                  ; FOR..NEXT stack length
WHSLEN              equ       16                  ; WHILE..ENDWH stack length
GOSLEN              equ       16                  ; GOSUB stack length

;-------------------------------------------------------------------------------
; Error codes
;-------------------------------------------------------------------------------

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
MCMSMERR            next      :temp               ; missing comma or semicolin
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
UNINIERR            next      :temp               ; uninitalized I/O vector
HEX2AERR            next      :temp               ; argument <0 or >255 in HEX2 function
NOTALERR            next      :temp               ; statement not allowed in direct mode
NOTINTER            next      :temp               ; an RETI statement executed when not in interrupt
PACCARGE            next      :temp               ; tried to assign a value of <0 or >255 to PACC
INTMODER            next      :temp               ; interrupt or count mode error in ONPACC
EETOSMAL            next      :temp               ; program storage EEPROM is Too Small

;-------------------------------------------------------------------------------
; Mathematical operator tokens
;-------------------------------------------------------------------------------

                    #temp     $10
OPARNTOK            next      :temp               ; '('
CPARNTOK            next      :temp               ; ')'
                    #temp     $20
ANDTOK              next      :temp               ; 'AND'
ORTOK               next      :temp               ; 'OR'
EORTOK              next      :temp               ; 'EOR'
                    #temp     $30
LTTOK               next      :temp               ; '<'
GTTOK               next      :temp               ; '>
LTEQTOK             next      :temp               ; '<='
GTEQTOK             next      :temp               ; '>='
EQTOK               next      :temp               ; '='
NOTEQTOK            next      :temp               ; '<>'
                    #temp     $40
PLUSTOK             next      :temp               ; '+'
MINUSTOK            next      :temp               ; '-'
SPLUSTOK            next      :temp               ; '+'
                    #temp     $50
MULTTOK             next      :temp               ; '*'
DIVTOK              next      :temp               ; '/'
MODTOK              next      :temp               ; '%'
                    #temp     $60
PWRTOK              next      :temp               ; '^'
                    #temp     $70
INDIRTOK            next      :temp               ; '@'
NOTTOK              next      :temp               ; 'NOT'
NEGTOK              next      :temp               ; '-' (unary minus)

;-------------------------------------------------------------------------------
; Keyword tokens
;-------------------------------------------------------------------------------

                    #temp     $01
LETTOK              next      :temp               ; LET
IMLETTOK            next      :temp               ; implied LET
PRINTTOK            next      :temp               ; PRINT
FORTOK              next      :temp               ; FOR
NEXTTOK             next      :temp               ; NEXT
TRONTOK             next      :temp               ; TRON
TROFFTOK            next      :temp               ; TROFF
POKETOK             next      :temp               ; POKE
DIMTOK              next      :temp               ; DIM
REMTOK              next      :temp               ; REM
PACCTOK             next      :temp               ; PACC
DATATOK             next      :temp               ; DATA
READTOK             next      :temp               ; READ
RESTRTOK            next      :temp               ; RESTORE
GOSUBTOK            next      :temp               ; GOSUB
                    #temp     $12
GOTOTOK             next      :temp               ; GOTO
ONTOK               next      :temp               ; ON
RETNTOK             next      :temp               ; RETURN
IFTOK               next      :temp               ; IF
INPUTTOK            next      :temp               ; INPUT
STOPTOK             next      :temp               ; STOP
ENDTOK              next      :temp               ; END
WHILETOK            next      :temp               ; WHILE
ENDWHTOK            next      :temp               ; ENDWH
EEPTOK              next      :temp               ; EEP
PORTATOK            next      :temp               ; PORTA
PORTBTOK            next      :temp               ; PORTB
PORTCTOK            next      :temp               ; PORTC
PORTDTOK            next      :temp               ; PORTD
                    #temp     $23
INBYTTOK            next      :temp               ; INBYTE
TIMETOK             next      :temp               ; TIME
ONTIMTOK            next      :temp               ; ONTIME
ONIRQTOK            next      :temp               ; ONIRQ
RETITOK             next      :temp               ; RETI
ONPACTOK            next      :temp               ; ONPACC
SLEEPTOK            next      :temp               ; SLEEP
RTIMETOK            next      :temp               ; RTIME
                    #temp     $36
FUNCTFLG            next      :temp               ; function flag byte
TOTOK               next      :temp               ; TO
THENTOK             next      :temp               ; THEN
ELSETOK             next      :temp               ; ELSE
STEPTOK             next      :temp               ; STEP

;-------------------------------------------------------------------------------
; Function tokens
;-------------------------------------------------------------------------------

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

;-------------------------------------------------------------------------------
; Numerical/variable tokens
;-------------------------------------------------------------------------------

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

;-------------------------------------------------------------------------------
; Misc. tokens
;-------------------------------------------------------------------------------

                    #temp     $78
PNUMTOK             next      :temp               ; '#'
EQUALTOK            next      :temp               ; '='
MEOLTOK             next      :temp               ; :
SEMITOK             next      :temp               ; ;
COMMATOK            next      :temp               ; ,
EOLTOK              next      :temp               ; end of line token
SSCNTOK             next      :temp               ; single space token
MSCNTOK             next      :temp               ; multiple space count token

JMPOP               equ       $7E                 ; OP-CODE FOR "JMP" (USED TO INITIALIZE INTERRUPT TABLE)

;*******************************************************************************
; Variables
;*******************************************************************************

                    #RAM

          ; char

IBUFPTR             rmb       2                   ; input buffer pointer
TBUFPTR             rmb       2                   ; token buffer pointer

          ; the next 5 variables must remain grouped togeather

BASBEG              rmb       2                   ; start of basic program area
BASEND              rmb       2                   ; end of basic program
VARBEGIN            rmb       2                   ; start of variable storage area
VAREND              rmb       2                   ; end of variable storage area
HILINE              rmb       2                   ; highest line number in program buffer

BASMEND             rmb       2                   ; physical end of basic program memory
VARMEND             rmb       2                   ; physical end of variable memory

          ; int

FIRSTLIN            rmb       2                   ; first line to list
LASTLIN             rmb       2                   ; last line to list
INTPTR              rmb       2                   ; integer pointer

          ; short

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
DNAME               rmb       3                   ; Place to put the variable name when doing a dump command
SUBMAX              rmb       2                   ;
SUBCNT              rmb       2                   ;
TOKPTR              rmb       2                   ; token pointer (used for list command)
VarSize             rmb       2                   ; used by the line editor. size of the variable table

;                   org       $009E

CONSTAT             rmb       3                   ; GET CONSOLE STATUS FOR BREAK ROUTINE.
INCONNE             rmb       3                   ; GET BYTE DIRECTLY FROM CONSOLE FOR BREAK ROUTINE.

;                   org       $00A4

INTABLE             rmb       8*2                 ; RESERVE SPACE FOR 8 DIFFERENT INPUT ROUTINES.
OUTABLE             rmb       8*2                 ; RESERVE SPACE FOR 8 DIFFERENT OUTPUT ROUTINES.

;                   org       $00C4               ; START OF RAM INTERRUPT VECTORS.

RAMVECTS            equ       *
SCISS               rmb       3                   ; SCI SERIAL SYSTEM.
SPITC               rmb       3                   ; SPI TRANSFER COMPLETE.
PACCIE              rmb       3                   ; PULSE ACCUMULATOR INPUT EDGE.
PACCOVF             rmb       3                   ; PULSE ACCUMULATOR OVERFLOW.
TIMEROVF            rmb       3                   ; TIMER OVERFLOW.
vTOC5               rmb       3                   ; TIMER OUTPUT COMPARE 5.
vTOC4               rmb       3                   ; TIMER OUTPUT COMPARE 4.
vTOC3               rmb       3                   ; TIMER OUTPUT COMPARE 3.
vTOC2               rmb       3                   ; TIMER OUTPUT COMPARE 2.
vTOC1               rmb       3                   ; TIMER OUTPUT COMPARE 1.
vTIC3               rmb       3                   ; TIMER INPUT CAPTURE 3.
vTIC2               rmb       3                   ; TIMER INPUT CAPTURE 2.
vTIC1               rmb       3                   ; TIMER INPUT CAPTURE 1.
REALTIMI            rmb       3                   ; REAL TIME INTERRUPT.
IRQI                rmb       3                   ; IRQ INTERRUPT.
XIRQ                rmb       3                   ; XIRQ INTERRUPT.
SWII                rmb       3                   ; SOFTWARE INTERRUPT.
ILLOP               rmb       3                   ; ILLEGAL OPCODE TRAP.
COP                 rmb       3                   ; WATCH DOG TIMER FAIL.
CMF                 rmb       3                   ; CLOCK MONITOR FAIL.

                    #ROM
                    title     BASICLB1
;*******************************************************************************
;                      MC68HC11 BASIC INTERPRETER
;                             WRITTEN BY:
;                           GORDON DOUGHMAN
;                        COPYRIGHT 1985-1990 BY
;                           GORDON DOUGHMAN
;*******************************************************************************
;       include "1.DEFINES.C"
;
; *main()
; *{
; initvars();            initalize all variables & pointers
; outheader();           send startup message to console
; outrdy();              output ready message

;*******************************************************************************

MAIN                proc
                    jsr       INITVARS            ; INITALIZE ALL INTERNAL VARIABLES.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; get the auto start flag.
                    cmpa      #$55
                    bne       PrintHeader@@
                    cli                           ; ALLOW ALL INTERRUPTS TO BE SERVICED.
                    jsr       CRUN
PrintHeader@@       bsr       OUTHEADR            ; PRINT HEADER.

TopLoop             ldd       RAMStart            ; RESET STACK VALUE.
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

MainLoop            clr       IMMID               ; CLEAR THE IMMIDIATE MODE FLAG.
                    clr       ERRCODE             ; CLEAR THE ERROR CODE BYTE.
                    clr       RUNFLAG             ; CLEAR THE RUN MODE FLAG.
                    jsr       OUTPRMPT            ; SEND PROMPT TO CONSOLE.
                    jsr       GETLINE             ; GO GET LINE FROM OPERATOR.
                    bsr       SKIPSPCS            ; SKIP ANY LEADING SPACES.
                    jsr       CHCKCMDS            ; GO CHECK FOR ANY COMMANDS.
                    cpd       #0                  ; WERE THERE ANY?
                    bne       TopLoop             ; YES. CONTINUE.
                    ldx       VAREND              ; SAVE CURRENT END OF VARIABLE AREA IN CASE LINE
                    stx       FENCE               ; HAS AN ERROR IN IT. (SO NO SPURIOUS VARIABLES
                                                  ; ARE ALLOCATED)
                    jsr       PARSE
                    tst       IMMID               ; DID WE EXECUTE IN IMMIDATE MODE?
                    bne       TopLoop             ; YES. PRINT READY MESSAGE.
                    bra       MainLoop            ; NO. JUST PRINT PROMPT.

TopCont             ldx       FENCE               ; GET THE VAREND FENCE.
                    clr       ,x                  ; MARK "OLD" END OF VARIABLE AREA IN CASE ANY
                                                  ; VARIABLES WERE ALLOCATED.
                    stx       VAREND              ; RESTORE THE POINTER.
                    bra       TopLoop             ; CONTINUE AFTER ERROR.

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
                    fcc       "BASIC11 v1.55",CR,LF
                    fcc       "Copyright 1985-1990 by",CR,LF
                    fcs       "Gordon Doughman",CR,LF

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
                    bra       PL

Msg@@               fcs       CR,LF,"READY",CR,LF

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

Break?@@            cmpa      #CNTRLC             ; IS IT A "BREAK"?
                    bne       Char@@              ; NO. GO PROCESS THE CHARACTER.
                    inc       CONTFLAG            ; DONT ALLOW A "CONT".
                    jmp       CHCKBRK2            ; GO DO A BREAK.

Char@@              cmpa      #SPC
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
; ibufptr=inbuff;        initalize the input buffer pointer
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

Msg@@               fcs       CR,LF,"#"

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

;===============================================================================

NL                  proc
                    ldx       #Msg@@
                    bsr       PL
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    rts

Msg@@               fcs       $0A,$0D

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
Done@@              equ       :AnRTS

;*******************************************************************************
;        ***** parse() *****

; *parse()
; *{
; int num;
; tbufptr=tknbuf;                initalize the token buffer pointer

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
;  immid=0;                      flag as not immidiate
;  putlinum(num);                put line number in buffer
; }
;  else immid=1;                  if no line # flag as immidiate*/

                    bsr       GETLINUM
                    bcc       Immediate@@
                    psha
                    jsr       GETCHR
                    cmpa      #EOL
                    pula
                    bne       Lined@@
                    jsr       DELLINE
                    ldx       VAREND
                    inx
                    stx       STRASTG
Exit@@              rts

Lined@@             clr       IMMID
ImmLoop@@           bsr       PUTLINUM
                    bra       Done@@

Immediate@@         jsr       GETCHR
                    cmpa      #EOL
                    beq       Exit@@
                    lda       #1
                    sta       IMMID
                    clrd
                    bra       ImmLoop@@

; if(errcode) return;             if line number error, return
; xlate();                        if translation error, return
; if(errcode) return;
; if(immid) runline();            if immidiate mode run 1 line
;  else storlin();                if not store pgm line
; return;                         go get next line
; *}

Done@@              jsr       XLATE
                    tst       IMMID
                    beq       Store@@
                    jmp       RUNLINE             ; GO RUN THE LINE & RETURN.

Store@@             jsr       STORLIN             ; GO STORE LINE & RETURN.
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
                    bcc       Done@@

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
                    bcs       Number@@
                    sec
                    ldd       ,y
                    bne       Done@@
                    lda       #LINENERR
                    bra       Error@@

Done@@              ins:2
                    puly
                    stx       IBUFPTR
                    rts

Number@@            bsr       ADDDIG
                    bpl       Loop@@
                    lda       #LINRANG
Error@@             jmp       RPTERR

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
                    bcc       NUMERIC
                    rts

;*******************************************************************************
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

XLATE               proc
                    jsr       GETCHR              ; GET NEXT CHAR.
                    cmpa      #EOL                ; AT THE END OF THE LINE?
                    beq       Done@@              ; YES.
                    clr       IFWHFLAG            ; NOT XLATING "IF" OR "WHILE"
                    jsr       BLANKS              ; SKIP BLANKS.
                    ldx       #KEYWORDS           ; POINT TO KEYWORD TABLE.
Loop@@              jsr       STREQ               ; IS KEYWORD IS IN THE INPUT BUFFER?
                    bcs       Go@@                ; YES GO PROCESS IT.
ToEnd@@             inx                           ; NO. POINT TO NEXT CHAR.
                    lda       ,x                  ; AT THE END OF THIS KEYWORD?
                    bne       ToEnd@@             ; NO.
                    ldb       #4                  ; NUMBER OF BYTES TO SKIP.
                    abx
                    tst       ,x                  ; AT THE END OF THE TABLE?
                    bne       Loop@@              ; NO. CHCK FOR NEXT KEYWORD.
                    lda       #IMLETTOK           ; ASSUME AN IMPLIED LET.
;                   jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    ldx       #XIMPLET            ; GET ADDR OF XLATION ROUTINE.
;                   jsr       ,x                  ; GO DO IT.
;                   bra       SkipBlanks@@        ; GO FINISH UP.
                    bra       SaveToken@@

Go@@                lda       1,x                 ; GET KEYWORD TOKEN.
                    ldx       2,x                 ; GET ADDR OF XLATION ROUTINE.

SaveToken@@         jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    cmpa      #DATATOK            ; SPECIAL CASE, DONT SKIP BLANKS AFTER KEYWORD.
                    beq       DoToken@@
                    cmpa      #REMTOK             ; SAME SPECIAL CASE AS FOR DATA.
                    beq       DoToken@@
                    jsr       BLANKS              ; SKIP BLANKS BETWEEN KEYWORD & NEXT OBJECT.
DoToken@@           jsr       ,x                  ; GO DO IT.
SkipBlanks@@        jsr       BLANKS              ; SKIP BLANKS.
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
                    rts

;-------------------------------------------------------------------------------
; KEYWORD LOOK UP TABLE
;-------------------------------------------------------------------------------
?                   macro
                    mstr      1
                    fcs       ~1~
                    fcb       ~2~
                    fdb       ~3~
                    endm

KEYWORDS
DATA                @?        DATA,DATATOK,XDATA
LET                 @?        LET,LETTOK,XLET
READ                @?        READ,READTOK,XREAD
RESTORE             @?        RESTORE,RESTRTOK,XRESTORE
GOSUB               @?        GOSUB,GOSUBTOK,XGOSUB
GOTO                @?        GOTO,GOTOTOK,XGOTO
ONTIME              @?        ONTIME,ONTIMTOK,XONTIME
ONIRQ               @?        ONIRQ,ONIRQTOK,XONIRQ
ONPACC              @?        ONPACC,ONPACTOK,XONPACC
ON                  @?        ON,ONTOK,MyXON
RETURN              @?        RETURN,RETNTOK,XRETURN
IIF                 @?        IF,IFTOK,XIF
INPUT               @?        INPUT,INPUTTOK,XINPUT
PRINT               @?        PRINT,PRINTTOK,XPRINT
                    @?        ?,PRINTTOK,XPRINT
FOR                 @?        FOR,FORTOK,XFOR
NEXT                @?        NEXT,NEXTTOK,XNEXT
STOPSS              @?        STOP,STOPTOK,XSTOP
ENDWH               @?        ENDWH,ENDWHTOK,XENDWH
ENDS                @?        END,ENDTOK,XEND
REM                 @?        REM,REMTOK,XREM
TRON                @?        TRON,TRONTOK,XTRON
TROFF               @?        TROFF,TROFFTOK,XTROFF
WHILE               @?        WHILE,WHILETOK,XWHILE
POKE                @?        POKE,POKETOK,XPOKE
DIM                 @?        DIM,DIMTOK,XDIM
EEP                 @?        EEP,EEPTOK,XEEP
MPORTA              @?        PORTA,PORTATOK,XPORTA
MPORTB              @?        PORTB,PORTBTOK,XPORTB
MPORTC              @?        PORTC,PORTCTOK,XPORTC
MPORTD              @?        PORTD,PORTDTOK,XPORTD
INBYTES             @?        INBYTE,INBYTTOK,XINBYTE
TIME                @?        TIME,TIMETOK,XTIME
RETI                @?        RETI,RETITOK,XRETI
PACC                @?        PACC,PACCTOK,XPACC
SLEEP               @?        SLEEP,SLEEPTOK,XSLEEP
RTIMES              @?        RTIME,RTIMETOK,XRTIME
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
                    bne       Spaces?@@
                    incb
                    inx
                    bra       Loop@@

; if(spcnt==0) return;

Spaces?@@           tstb
                    beq       Done@@

; if(spcnt>1)
;  {
;   *tbufptr++=MSCNTOK;
;   *tbufptr++=spcnt;
;  }

                    stx       IBUFPTR
                    ldx       TBUFPTR
                    cmpb      #1
                    beq       Token@@
                    lda       #MSCNTOK
                    sta       ,x
                    inx
Save@@              stb       ,x
                    inx
                    stx       TBUFPTR
Done@@              pulx
                    rts

; else
;  {
;   *tbufptr++=SSCNTOK;
;  }
; return;
; *}

Token@@             ldb       #SSCNTOK
                    bra       Save@@

;*******************************************************************************
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
;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER.

GETCHR              proc
                    pshx
                    ldx       IBUFPTR             ; GET POINTER.
                    lda       ,x                  ; GET A CHARACTER.
                    pulx
                    rts

;*******************************************************************************
;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER
;        AND ADVANCES THE POINTER TO POINT TO THE NEXT CHARACTER.

GETNXCHR            proc
                    bsr       GETCHR

          ; FALL THROUGH TO INCIBP.

;*******************************************************************************
;        THIS ROUTINE JUST INCREMENTS THE INPUT BUFFER POINTER.

INCIBP              proc
                    pshx
                    ldx       IBUFPTR             ; GET POINTER.
                    inx                           ; ADVANCE POINTER.
                    stx       IBUFPTR             ; UPDATE POINTER.
RetX                pulx
                    rts

;*******************************************************************************
;        THIS ROUTINE PUTS THE WORD IN THE D-REG. INTO THE TOKEN BUFFER
;        AND ADVANCES THE TOKEN BUFFER POINTER.

PUTDTOK             proc
                    bsr       PUTTOK              ; PUT THE FIRST BYTE INTO THE TOKEN BUFFER.
                    tba                           ; PUT THE 2ND BYTE INTO A.

          ; FALL THROUGH TO PUTTOK.

;*******************************************************************************
;        THIS ROUTINE PUTS THE CHARACTER IN THE A-REG. INTO THE TOKEN
;        BUFFER AND ADVANCES THE TOKEN BUFFER POINTER.

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
                    bhi       RetX                ; NO. RESTORE X AND RETURN.
                    lda       #EXPCXERR           ; YES. FLAG THE ERROR.
                    jmp       RPTERR              ; GO REPORT IT.

                    title     BASICLB2

;*******************************************************************************
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

XGOSUB              proc
XGOTO               proc
;                   jsr       BLANKS
                    lda       #LCONTOK
                    bsr       PUTTOK
                    jsr       GETLINUM
                    bra       PUTDTOK

;*******************************************************************************
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

XRETURN             equ       :AnRTS
XSTOP               equ       :AnRTS
XEND                equ       :AnRTS
XTRON               equ       :AnRTS
XTROFF              equ       :AnRTS
XRESTORE            equ       :AnRTS
XENDWH              equ       :AnRTS
XRETI               equ       :AnRTS              ; NULL FUNCTIONS BECAUSE TOKEN
XSLEEP              equ       :AnRTS              ; PLACEMENT IS DONE IN
XRTIME              equ       :AnRTS              ; XLATE FUNCTION.

;*******************************************************************************
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
                    clra                          ; SAVE A BYTE FOR THE LENGTH.
                    bsr       PUTTOK
                    ldb       #2                  ; INITALIZE LENGTH TO 2 (INCLUDES LENGTH & EOL.
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
                    bra       ?ASIGNMT            ; GO DO IT LIKE AN ASIGNMENT STATEMENT.

;*******************************************************************************
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
?ASIGNMT            bsr       GETNXCHR
                    cmpa      #'='
                    beq       Equal@@
                    lda       #IVEXPERR
                    jmp       RPTERR

Equal@@             lda       #EQUALTOK
                    bsr       PUTTOK
                    tba

          ; FALL THROUGH TO XEXPRES.

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
MainLoop@@          ldx       #UINARYOP
                    jsr       TBLSRCH
                    bcc       GetChar@@
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

GetChar@@           bsr       GETCHR
                    cmpa      #'('
                    bne       Number?@@
                    bsr       INCIBP
                    lda       #OPARNTOK
                    bsr       PUTTOK
                    lda       ,y
                    bsr       XEXPRES
                    jsr       GETNXCHR
                    cmpa      #')'
                    beq       RightParen@@
                    lda       #UPARNERR
                    jmp       RPTERR

RightParen@@        lda       #CPARNTOK
                    jsr       PUTTOK
                    bra       CHKOPRTR

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

Number?@@           jsr       NUMERIC
                    bcs       XEXPRS4
                    cmpa      #'$'
                    beq       XEXPRS4
                    cmpa      #'"'
                    bne       Func@@
XEXPRS4             jsr       GETCON
                    bra       XEXPRS7

Func@@              jsr       GETFUN
                    tsta
                    bne       XEXPRS7
                    jsr       GETVAR
XEXPRS7             ldb       ,y
                    cmpb      #NULL
                    bne       XEXPRS8
                    sta       ,y
XEXPRS8             cmpa      ,y
                    beq       XEXPRS15
                    lda       #DTMISERR
                    jmp       RPTERR

;  if(type==NUM)
;  {
;   if(c=cknumop()) ;
;   else if(c=ckbolop()) ;
;   else if(ifwhflag) c=cklogop();
;   else c=NULL;
;  }

XEXPRS15            lda       ,y
                    cmpa      #NUM
                    bne       Error@@
                    bsr       CKNUMOP
                    bcs       XEXPRS17
                    bsr       CKBOLOP
                    bcs       XEXPRS17
                    tst       IFWHFLAG
                    beq       Null@@
                    bsr       CKLOGOP
                    bra       XEXPRS17

Null@@              lda       #NULL
                    bra       XEXPRS17

;  else { errcode=IDTYERR; return; }

Error@@             lda       #IDTYERR
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
                    jmp       MainLoop@@

;*******************************************************************************
; *now look for operator or end of expression
;
;  chkoprtr:
;  c=*ibufptr;
;  if(c==EOL | c==MIDEOL | c==SPC | c==COMMA | c==SEMI | c==')')
;  {
;   return(c);
;  }

CHKOPRTR            proc
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
                    bra       XEXPRS15

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
CKOP                bsr       TBLSRCH
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

;*******************************************************************************
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
                    inx                           ; FOUND IT. BUMP POINTER TO NEXT ENTRY.
                    inx
                    lda       ,x                  ; AT THE END OF THE TABLE?
                    bne       TBLSRCH             ; NO. GO CHECK THE NEXT ENTRY.
                    clc                           ; YES. FLAG AS NOT FOUND.
                    rts

Found@@             lda       1,x                 ; GET TOKEN.
                    sec                           ; FLAG AS FOUND.
                    rts

;*******************************************************************************

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
                    sta       1,y
                    jsr       INCIBP
GETVAR2             bsr       CHCKTYP
                    sta       3,y

; if((offset=findvar(vartype,varname))==-1) is var already in table?
; {
;  if(errcode) return;
;  if((offset=putvar(vartype,varname))==-1) return;  no. put it there
; }
; if(errcode) return;

                    bsr       FINDVAR
                    cpd       #-1
                    bne       GETVAR5
GETVAR4             lda       3,y
                    jsr       PUTVAR

; *tbufptr++=vartype;    put variable type byte in token buffer
; putint(offset);        put offset after it
; if((vartype==IVARTOK) | (vartype==FVARTOK)) return(NUM);
; return(STRING);
; *}

GETVAR5             equ       *
                    pshd
                    lda       3,y
                    jsr       PUTTOK
                    puld
                    jsr       PUTDTOK
                    lda       3,y                 ; GET VARIABLE TYPE AGAIN.
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
Done@@              rts

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
                    beq       Error@@

; {
;  if(*varptr==vartype)          is the current var the same type?
;  {                             yes.
;   if(streq(varptr+1,varname))  is the name the same?
;   {                            yes.
;    return(varptr-varbegin);    return the offset from the table start
;   }
;  }

                    cmpa      ,x
                    bne       Array?@@
                    ldb       1,x
                    cmpb      ,y
                    bne       Array?@@
                    ldb       2,x
                    cmpb      1,y
                    bne       Array?@@
                    xgdx
                    subd      VARBEGIN
                    rts

;  if not, advance to the next variable in the table
;  if(*varptr==IVARTOK) varptr=varptr+ISIZ+3;
;  else if(*varptr==SVARTOK) varptr=varptr+SSIZ+3;
;  else if(*varptr==FVARTOK) varptr=varptr+FSIZ+3;
;  else { errcode=ILTOKERR; return(-1); }
; }

Array?@@            ldb       ,x
                    bitb      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       Var?@@              ; NO CONTINUE.
                    ldb       #ASIZ+3             ; YES. GET ARRAY SIZE +3.
                    bra       Cont@@

Var?@@              cmpb      #IVARTOK
                    bne       PrintErr@@
                    ldb       #ISIZ+3
Cont@@              abx
                    bra       Loop@@

PrintErr@@          lda       #ILTOKERR
                    jmp       RPTERR

Error@@             ldd       #-1
                    rts

; return(-1);
; *}

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
                    bls       Done@@
                    lda       #OMEMERR
                    jmp       RPTERR

Done@@              puld
                    subd      VARBEGIN
                    pshd                          ; SAVE THE OFFSET TO THIS VARIABLE.
                    jsr       CCLEAR3             ; CLEAR ALL VARIABLES SINCE WE MAY HAVE TRASHED
                                                  ; ANY ARRAYS THAT HAD BEEN ALLOCATED.
                    puld                          ; RESTORE THE "NEW" VARIABLE OFFSET.
                    rts

;*******************************************************************************

CLRVAR              proc
                    bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       Go@@                ; NO. CONTINUE.
                    ldb       #ASIZ               ; YES. GET THE DICTIONARY SIZE+1.
                    bra       Loop@@              ; PUT THE VARIABLE IN THE DICTIONARY.

Go@@                cmpa      #IVARTOK
                    bne       Error@@
                    ldb       #ISIZ
Loop@@              clr       ,x
                    inx
                    decb
                    bne       Loop@@
                    rts

Error@@             lda       #ILTOKERR
                    jmp       RPTERR

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
                    bsr       GETHEX
                    bra       Cont@@

Decimal@@           bsr       GETDECI

; *tbufptr++=ICONTOK;            put integer constant token in buffer
; putint(const);                 follow it with the constant
; count=ibufptr-litp;    get number of bytes in source form of const.
; *tbufptr++=count;       put it in the token buffer
; while(litp < ibufptr) *tbufptr++=*litp++; copy source form into buffer
; return(NUM);           return the constant type
; }

Cont@@              psha
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
                    psha
                    psha
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    jsr       NUMERIC
                    bcs       Loop@@
                    lda       #SYTXERR
                    bra       CHCKERR

Loop@@              lda       ,x
                    jsr       NUMERIC
                    bcc       GETDECI3
                    jsr       ADDDIG
                    bpl       Loop@@
                    lda       #INTOVERR
                    bra       CHCKERR

GETDECI3            stx       IBUFPTR
                    ldd       ,y
                    ins
                    ins
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
                    psha
                    psha
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    bsr       HEXDIG
                    bcs       Loop@@
                    lda       #IVHEXERR
CHCKERR             tst       RUNFLAG
                    jeq       RPTERR
                    jmp       RPTRERR

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
                    bsr       HEXDIG
                    bcc       GETDECI3
                    ldd       ,y
                    lsld
                    bcs       Error@@
                    lsld
                    bcs       Error@@
                    lsld
                    bcs       Error@@
                    lsld
                    bcs       Error@@
                    std       ,y
                    lda       ,x
                    jsr       ToUpper
                    tab
                    inx
                    subb      #'0'
                    cmpb      #9
                    bls       Skip@@
                    subb      #7
Skip@@              clra
                    addd      ,y
                    std       ,y
                    bra       Loop@@

Error@@             lda       #HEXOVERR
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
                    bcs       :AnRTS

                    jsr       ToUpper
                    cmpa      #'A'
                    blo       No@@
                    cmpa      #'F'
                    bhi       No@@
                    sec
                    rts

No@@                clc
                    rts

;*******************************************************************************
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
                    beq       Done@@
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

Done@@              jsr       PUTTOK
                    pulx
                    stb       ,x
                    rts

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
                    ldx       #FUNCTBL
Loop@@              jsr       STREQ
                    bcs       Done@@
FindEnd@@           inx
                    lda       ,x
                    bne       FindEnd@@
                    ldb       #4
                    abx
                    tst       ,x
                    bne       Loop@@
                    clra
                    rts

Done@@              lda       #FUNCTFLG
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

;*******************************************************************************

XPOKE               proc
                    ldx       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    dex                           ; DEC. TO COMPENSATE FOR PUTTOK DONE IN XLATE.
                    stx       TBUFPTR             ; SAVE NEW POINTER VALUE. FALL THROUGH TO BNUMFUN.
                    lda       ,x                  ; GET TOKEN BACK INTO THE A-REG.

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
                    pshb
                    pshb
                    tsy
                    ldb       #2
                    bsr       DOFUNCT
;                   lda       #NUM
                    pula
                    pula
                    puly
                    rts

;*******************************************************************************
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
                    jmp       ?ASIGNMT            ; GO USE ASSIGNMENT CODE FOR REST OF FUNCTION.

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
                    beq       OpenParen@@
Error@@             lda       #MPARNERR
                    jmp       RPTERR

OpenParen@@         jsr       INCIBP
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
                    beq       Done@@
                    jsr       CHKCOMA
                    bcc       Error@@
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

Done@@              jsr       GETCHR
                    cmpa      #')'
                    bne       Error@@
                    jsr       INCIBP
                    lda       #CPARNTOK
                    jmp       PUTTOK              ; PUT TOKEN IN BUFFER & RETURN.

;*******************************************************************************

FINDPORT            proc
                    jsr       GETNXCHR            ; GO GET PORT "NUMBER".
                    jsr       ToUpper             ; Translate the character to upper case.
                    cmpa      #'A'                ; IS IT AN A OR HIGHER?
                    bhs       UpLim@@             ; YES. GO CHECK UPPER LIMIT.
Error@@             lda       #ILPRTERR           ; NO. ILLEGAL PORT "NUMBER".
                    jmp       RPTERR              ; REPORT ERROR.

UpLim@@             cmpa      #'E'                ; IS IT HIGHER THAN AN "E"?
                    bhi       Error@@             ; YES. ILLEGAL PORT.
                    suba      #'A'                ; SUBTRACT "BASE" PORT OF A
                    adda      #FPRTATOK           ; ADD IN "BASE" TOKEN.

;*******************************************************************************
                                                  ; STEAL SOME CODE.
XPACCF              proc
XTIMEF              jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    lda       #NUM                ; RETURN TYPE "NUM".
                    rts

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

MyXON               proc
;                   jsr       BLANKS
                    lda       #NUM
                    jsr       XEXPRES
                    jsr       BLANKS
                    ldx       #GOTO
                    jsr       STREQ
                    bcc       Go@@
                    lda       #GOTOTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       XGOTO
                    bra       Loop@@

Go@@                ldx       #GOSUB
                    jsr       STREQ
                    bcs       Token@@
                    lda       #IONSYERR
                    jmp       RPTERR

Token@@             lda       #GOSUBTOK
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
                    bcc       :AnRTS

                    jsr       BLANKS
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
                    bcs       Done@@              ; IF PRESENT, IT'S OK.
XONTIME2            lda       #MCOMAERR           ; IF NOT, REPORT AN ERROR.
                    jmp       RPTERR

Done@@              jsr       BLANKS
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
                    bcc       XONTIME2            ; NO COMMA. REPORT ERROR.
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
                    bcs       Go@@
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

Go@@                lda       #THENTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcc       Error@@

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
                    bcc       Exit@@

                    lda       #ELSETOK
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
                    bcs       Done@@
Error@@             lda       #LINENERR
                    jmp       RPTERR

Done@@              lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
                    jmp       PUTDTOK
Exit@@              rts

; return;                in any case, return
; *}

THENS               fcs       "THEN"
ELSES               fcs       "ELSE"

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
Error@@             lda       #IVEXPERR
                    jmp       RPTERR

Go@@                jsr       GETNXCHR
                    cmpa      #'='
                    bne       Error@@

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
                    bcs       Token@@
                    lda       #MTOERR
                    jmp       RPTERR

Token@@             lda       #TOTOK
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
                    bcc       :AnRTS

                    lda       #STEPTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    lda       #NUM
                    jmp       XEXPRES

TO                  fcs       "TO"
STEP                fcs       "STEP"

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
Done@@              equ       :AnRTS

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
;  else if(*ibufptr==SEMI) *tbufptr=SEMITOK;     check for semicolin
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
                    bne       Go@@
Done@@              rts

Go@@                jsr       GETCHR              ; GET THE NEXT CHARACTER IN THE BUFFER.
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
                    beq       Semi@@
                    lda       #MCMSMERR
                    bra       RPTERR

Semi@@              lda       #SEMITOK
                    bsr       PutToken
                    bra       Loop@@

;*******************************************************************************

CHKCOMA             proc
                    jsr       GETCHR              ; GET CHARACTER FROM INPUT BUFFER.
                    cmpa      #COMMA              ; IS IT A COMMA?
                    beq       Token@@             ; YES. PUT IT IN THE TOKEN BUFFER.
                    clc                           ; NO. FLAG NO COMMA FOUND.
                    rts

Token@@             lda       #COMMATOK           ; GET THE COMMA TOKEN.
;                   bra       PutToken

;*******************************************************************************

PutToken            proc
                    jsr       PUTTOK              ; PUT THE TOKEN IN THE BUFFER.
                    jsr       INCIBP              ; BUMP THE INPUT BUFFER POINTER.
                    sec
                    rts

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
XINPUT3             lda       #MCOMAERR
                    bra       RPTERR

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
                    bne       Done@@              ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.

                    lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    lda       #NUM                ; EXPRESSION MUST BE NUMERIC.
                    jsr       XEXPRES             ; GO GET THE EXPRESSION.
                    jsr       BLANKS              ; SKIP SPACES.
                    bsr       CHKCOMA             ; GO GET COMMA THAT MUST FOLLOW THE EXPRESSION.
                    bcc       XINPUT3             ; MISSING COMMA. GO REPORT THE ERROR.
Done@@              rts                           ; IT WAS THERE. GO PROCESS THE REST OF THE STATEMENT.

;*******************************************************************************

XINBYTE             proc
                    bsr       XCHKDEV             ; GO CHECK FOR ALTERNATE DEVICE.
                    jsr       BLANKS              ; SKIP BLANKS AFTER COMMA.
                    jmp       GETVAR              ; GO TRY TO GET A VARIABLE.

;*******************************************************************************
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
                    jmp       ?ASIGNMT            ; GO DO ASSIGNMENT STATEMENT.

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
                    beq       Go@@
                    jsr       OUTBYTE
                    inx
                    bra       Loop@@

; nl();          go to next line
; ptr=inbuff;    point to begining of input buffer
; while(ptr++ < ibufptr-2) outbyte('*');    output '*' to point of error
; pl("^^^");  point to error
; nl();

Go@@                jsr       NL
                    ldx       IBUFPTR
                    dex
                    dex
                    cpx       INBUFFS
                    bls       Print@@
                    stx       IBUFPTR
                    ldx       INBUFFS
                    lda       #'*'
Out@@               jsr       OUTBYTE
                    inx
                    cpx       IBUFPTR
                    bne       Out@@
Print@@             ldx       #Msg@@
                    jsr       PL
                    jsr       NL
                    bsr       PrintErrNum
                    lda       #1
                    sta       CONTFLAG
                    jmp       TopCont

Msg@@               fcs       "^^^"

;*******************************************************************************
; pl("ERROR # ");
; outdeci(errcode);
; return;
; *}

PrintErrNum         proc
                    ldx       #Msg@@
                    jsr       PL
                    ldb       ERRCODE
                    clra
                    bra       OUTDECI

Msg@@               fcs       "ERROR # "

;*******************************************************************************

RPTRERR             proc                          ; REPORT A RUN TIME ERROR.
                    sta       ERRCODE
                    bsr       PrintErrNum
                    ldx       #Msg@@
                    jsr       PL
                    ldd       CURLINE
                    bsr       OUTDECI
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
                    jmp       TopLoop

Msg@@               fcs       " IN LINE # "

OUTDECI             proc
                    cpd       #0
                    bne       Go@@
                    lda       #'0'
                    jmp       OUTBYTE

Go@@                pshy
                    pshd                          ; SAVE THE NUMBER TO PRINT.
                    ldd       #10000              ; NUMBER TO START DIVIDING BY.
                    pshd
                    clrb                          ; SET INITAL VALUE OF LEADING ZERO SUPRESS FLAG.
                    pshb
                    tsy
                    ldd       3,y                 ; IS THE NUMBER NEGATIVE?
                    bpl       Loop@@              ; NO. GO PRINT THE NUMBER.
                    negd                          ; YES. MAKE THE NUMBER POSITIVE.
                    std       3,y                 ; SAVE THE RESULT.
                    lda       #'-'                ; PRINT A MINUS SIGN TO SHOW IT'S NEGATIVE.
                    jsr       OUTBYTE
Loop@@              ldd       3,y                 ; GET THE DIVIDEND.
                    ldx       1,y                 ; GET THE DIVISOR.
                    idiv                          ; DO THE DIVIDE.
                    std       3,y                 ; SAVE THE REMAINDER.
                    xgdx                          ; PUT QUOTIENT IN D.
                    cpd       #0                  ; IS THE QUOTIENT 0?
                    bne       Val@@               ; NO. GO OUTPUT THE NUMBER.
                    tst       ,y                  ; YES. ARE WE STILL SUPRESSING LEADING ZEROS?
                    beq       Done@@              ; YES. DON'T PRINT THE NUMBER.
Val@@               tba                           ; PUT THE NUMBER IN THE A-REG.
                    adda      #$30                ; MAKE THE NUMBER ASCII.
                    ldb       #1                  ; MAKE THE ZERO SUPRESS FLAG NON-ZERO.
                    stb       ,y
                    jsr       OUTBYTE             ; OUTPUT THE NUMBER.
Done@@              ldd       1,y                 ; GET CURRENT DIVISOR.
                    ldx       #10                 ; DIVIDE IT BY 10.
                    idiv
                    stx       1,y                 ; SAVE RESULT. ARE WE DONE?
                    bne       Loop@@              ; NO KEEP GOING.
                    ldb       #5                  ; DEALLOCATE LOCALS.
                    aby
                    tys
                    puly
                    rts

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
                    rts

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
                    rts

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
                    bra       Done@@

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
                    puly
                    rts

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
Done@@              equ       :AnRTS

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
Done@@              equ       :AnRTS

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
                    bhi       Error@@
                    bsr       MoveVarsUp
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

Error@@             lda       #OMEMERR
                    jmp       RPTERR

Done@@              pulx
                    pulx
                    pulx
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

APENDLIN            proc
                    ldx       TKNBUFS
                    ldb       2,x
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       Error@@
;                   ldb       TKNBUF+2
                    bsr       MoveVarsUp
                    ldx       BASEND
                    abx
                    xgdx
                    ldx       BASEND
                    std       BASEND
                    bra       PUTLINE

Error@@             lda       #OMEMERR
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
                    beq       PUTLINE

; else if(lendif<0)              if line in tknbuf is larger
;      {
;       lendif=-lendif;          make it a positive number
;       openspc(lendif,ptr);     tru to open up a space
;       if(errcode) return;      if not enough memory, return
;       putline(ptr);            if ok, copy line to program buffer
;      }

                    bpl       Done@@
                    negb
                    pshx
                    bsr       OPENSPC
                    pulx
                    bra       PUTLINE

; else                           if line in tknbuf is smaller
;      {
;       closespc(lendif,ptr);    close up excess space
;       putline(ptr);            put new line in program buffer
;      }
; return;
; *}

Done@@              pshx
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
                    puly
                    rts

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
                    puly
                    rts

;*******************************************************************************

DecCount            proc
                    ldd       VarSize             ; get the size of the variable table.
                    decd                          ; decrement it.
                    std       VarSize             ; save the new value.
                    rts

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
Clear@@             clr       ,x                  ; CLEAR THE STORAGE TO ZERO.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       BASMEND             ; ARE WE DONE?
                    bls       Clear@@             ; NO. KEEP CLEARING.
                                                  ; YES. POINT TO THE PHYSICAL END OF MEMORY.
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG AGAIN.
                    cmpa      #$55                ; IS THE AUTO START MODE SET?
                    bne       Normal@@             ; NO. DO A NORMAL INIT.

                    jsr       AUTOLOAD            ; GO LOAD the program and VARIABLES INTO RAM.
Normal@@            clrd                          ; MAKE THE HIGHEST LINE IN THE PROGRAM 0.
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
                    rts

;*******************************************************************************

POWERUP             proc
                    @SetChipSelects

                    lda       #$93                ; TURN ON A/D, USE E CLOCK, SET IRQ LEVEL SENSITIVE
                    sta       OPTION              ; DELAY AFTER STOP, DISABLE CLOCK MONITOR, SET COP
                                                  ; TIMOUT PERIOD TO MAX.
                    lda       #$03                ; SET THE TIMER PRESCALER TO /16.
                    sta       TMSK2

                    ldd       RAMStart            ; Get start of RAM.
                    addd      RAMSize             ; Add the size of the RAM to it.
                    xgdx                          ; Put the calculated address into X.
                    txs                           ; Transfer the address to the stack pointer.

                    ldx       #RAMVECTS           ; POINT TO THE RAM INTERRUPT VECTOR TABLE.
                    ldy       #AnRTI              ; GET ADDRESS OF RTI INSTRUCTION.
                    ldb       #20                 ; PUT THE "JMP" OPCODE INTO ALL VECTOR LOCATIONS.
                    lda       #JMPOP              ; GET THE JMP OPCODE.
VecInit@@           sta       ,x                  ; STORE IT.
                    inx                           ; POINT TO THE NEXT VECTOR.
                    sty       ,x                  ; INITALIZE VECTOR TO "RTI".
                    inx
                    inx
                    decb                          ; DONE?
                    bne       VecInit@@           ; NO. INITALIZE ALL VECTORS.
                    ldx       #ILLOP              ; POINT TO THE ILLEGAL OP-CODE VECTOR.
                    ldd       #POWERUP            ; GET THE ADDRESS OF THE POWER UP VECTOR.
                    std       1,x                 ; INITALIZE ILLEGAL OP-CODE VECTOR.
                    std       4,x                 ; INITALIZE WATCH DOG FAIL VECTOR.
                    std       7,x                 ; INITALIZE CLOCK MONITOR FAIL VECTOR.

                    ldx       #INTABLE            ; POINT TO THE START OF THE I/O VECTOR TABLE.
                    ldy       #IOVects            ; point to the default table in ROM.
                    ldb       #32                 ; GET NUMBER OF BYTES IN THE TABLE.
IOInit@@            lda       ,y                  ; Move a byte of the table from ROM into RAM.
                    sta       ,x
                    inx                           ; POINT TO THE NEXT BYTE.
                    iny
                    decb                          ; DECREMENT THE COUNT.
                    bne       IOInit@@            ; GO TILL WE'RE DONE.

                    ldx       #TIMEINT            ; GET THE ADDRESS OF THE OUTPUT COMPARE 1 ROUTINE.
                    stx       vTOC1+1             ; PUT IT IN THE INTERRUPT VECTOR.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    clra
                    clrb
                    std       TIMEREG             ; ZERO THE TIME REGISTER.
                    std       TIMECMP             ; zero the time compare register.
                    bsr       TIMINTS             ; GO SETUP THE TIMER FOR THE FIRST INTERRUPT.
                    lda       #$80                ; ENABLE INTERRUPTS FROM OC1.
                    sta       TMSK1

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
                    jsr       ,x                  ; INITALIZE THE SCI.
                    jmp       MAIN                ; GO TO BASIC.

;*******************************************************************************

TIMEINT             proc
                    bsr       TIMINTS
AnRTI               rti                           ; RETURN FROM ALL INTERRUPT SOURCES.

;*******************************************************************************

TIMINTS             proc
                    ldd       TOC1REG             ; GET THE VALUE OF THE TIMER/COUNTER.

;*******************************************************************************

TIMINTS3            proc
                    addd      TimeVal             ; ADD IN 62500 FOR NEXT COMPARE ( 2 HZ INT.).
                    std       TOC1REG             ; PUT IT IN THE OUTPUT COMPARE REGISTER.
                    lda       #$80                ; SETUP TO CLEAR THE OC1 FLAG.
                    sta       TFLAG1
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
FakeGoto            inc       IMMID               ; FAKE THE GOTO ROUTINE OUT.
                    ldd       CURLINE             ; SAVE THE CURRENT LINE NUMBER IN MAIN PROGRAM.
                    std       SCURLINE
                    ldd       ADRNXLIN            ; SAVE THE ADDRESS OF THE NEXT LINE IN MAIN PROG.
                    std       SADRNXLN
                    jmp       RGOTO3              ; GOTO THE SERVICE ROUTINE.
Done@@              rts

;*******************************************************************************

IRQINT              proc
                    ldy       ONIRQLIN            ; GET POINTER TO LINE NUMBER OF THE IRQ SERVICE.
                    bne       FakeGoto            ; GO DO IT.
                    rti                           ; IF IT'S 0, "ONIRQ" HAS NOT BEEN EXECUTED.

;*******************************************************************************

PACCINT             proc
                    lda       #$30                ; RESET BOTH THE TIMER OVERFLOW & INPUT FLAG.
                    sta       TFLG2
                    ldy       ONPACLIN            ; GET POINTER TO LINE NUMBER OF THE SERVICE ROUT.
                    bne       FakeGoto
                    rti

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
Done@@              clrd                          ; YES. JUST RETURN.
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
                    bra       Done@@              ; YES. RETURN W/ ENTRY NOT FOUND INDICATION.

Found@@             ldx       1,x                 ; GET ADDRESS OF COMMAND.
                    jsr       ,x                  ; GO DO IT.
                    ldd       #1                  ; SHOW WE EXECUTED A COMMAND.
                    rts

?                   macro
                    mdef      2,~1~
                    mstr      1
                    fcs       ~1~
                    dw        C~2~
                    endm

Table@@             @?        LIST
                    @?        RUN
                    @?        NEW
                    @?        CONT
                    @?        CLEAR
                    @?        ESAVE
                    @?        ELOAD
                    @?        LLIST
                    @?        AUTOST
                    @?        NOAUTO
                    @?        FREE
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
                    beq       Done@@

                    jsr       SKIPSPCS

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
                    bcc       Eol?@@
                    jsr       GETLINUM
                    std       FIRSTLIN
                    jsr       GETCHR
                    cmpa      #'-'
                    beq       Dash@@
                    ldd       FIRSTLIN
                    std       LASTLIN
                    cpd       HILINE
                    bls       FirstLine@@
Done@@              rts

Dash@@              jsr       INCIBP
                    jsr       GETLINUM
                    cpd       HILINE
                    bls       LastLine@@
                    ldd       HILINE
LastLine@@          std       LASTLIN
                    bra       FirstLine@@

Eol?@@              cmpa      #EOL
                    bne       Done@@

                    ldx       BASBEG
                    ldd       ,x
                    std       FIRSTLIN
                    ldd       HILINE
                    std       LASTLIN
FirstLine@@         ldd       FIRSTLIN
                    cpd       LASTLIN
                    bhi       Done@@

                    ldd       FIRSTLIN
                    jsr       FINDLINE
                    stx       TOKPTR
                    ldd       LASTLIN
                    jsr       FINDLINE
                    ldd       ,x
                    cpd       LASTLIN
                    bne       Skip@@
                    ldb       2,x
                    abx
Skip@@              stx       LASTLIN

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

MainLoop@@          ldd       TOKPTR
                    cpd       LASTLIN
                    beq       Done@@

                    ldx       TOKPTR
                    ldd       ,x
                    inx
                    inx
                    inx
                    stx       TOKPTR
                    jsr       OUTDECI
Loop@@              ldx       TOKPTR
                    lda       ,x
                    cmpa      #EOLTOK
                    beq       Eol@@
                    tsta
                    bmi       Var@@
                    jsr       LKEYWORD
                    bra       Loop@@

Var@@               bsr       LVARCON
                    bra       Loop@@

Eol@@               jsr       NL
                    ldx       TOKPTR
                    inx
                    stx       TOKPTR
                    bra       MainLoop@@

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
                    beq       Done@@
                    inx
                    inx
                    inx
                    tst       ,x
                    bne       Loop@@
                    lda       #ILTOKERR
                    jmp       RPTERR

Done@@              ldx       1,x
                    jsr       ,x
                    rts

Table@@             fcb       IVARTOK
                    fdb       LIVAR
                    fcb       SCONTOK
                    fdb       LSCON
                    fcb       LCONTOK
                    fdb       LLCON
                    fcb       ICONTOK
                    fdb       LICON
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
                    jmp       OUTBYTE
Done@@              equ       :AnRTS

;*******************************************************************************

LFCON               proc
                    ldd       TOKPTR
                    addd      #FSIZ+1

LFCON2              proc
                    xgdx
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
                    bra       LFCON2

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
                    bra       LFCON2

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
                    inx
                    inx
                    stx       TOKPTR
                    jmp       OUTDECI

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
                    beq       LMSPCS

                    cmpa      #REMTOK
                    beq       LREMLINE

                    cmpa      #DATATOK
                    beq       LDATALIN

                    cmpa      #FUNCTFLG
                    bne       Search@@
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    ldx       #LFUNCTBL
                    bra       Loop@@

Search@@            ldx       #TOKTBL
Loop@@              cmpa      ,x
                    beq       Found@@
                    inx
                    inx
                    inx
                    tst       ,x
                    bne       Loop@@
                    lda       #ILTOKERR
                    jmp       RPTERR

Found@@             ldx       1,x
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
                    bra       LREM3

;*******************************************************************************

LREMLINE            proc
                    ldx       #REM
                    jsr       PL
LREM3               ldx       TOKPTR
                    inx                           ; PUT POINTER PAST LENGTH BYTE.
Loop@@              lda       ,x
                    cmpa      #EOL
                    bne       Out@@
                    inx
                    stx       TOKPTR
                    rts

Out@@               jsr       OUTBYTE
                    inx
                    bra       Loop@@

;*******************************************************************************

TOKTBL              fcb       LETTOK
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
                    fdb       MPORTA
                    fcb       PORTBTOK
                    fdb       MPORTB
                    fcb       PORTCTOK
                    fdb       MPORTC
                    fcb       PORTDTOK
                    fdb       MPORTD
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

LFUNCTBL            fcb       FDIVTOK
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
                    fdb       MPORTA
                    fcb       FPRTBTOK
                    fdb       MPORTB
                    fcb       FPRTCTOK
                    fdb       MPORTC
                    fcb       FPRTDTOK
                    fdb       MPORTD
                    fcb       FPRTETOK
                    fdb       MPORTE
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
MPORTE              fcs       "PORTE"
POUNDSGN            fcs       "#"

;*******************************************************************************

CRUN                proc
                    jsr       NL2                 ; DO 2 CR/LF SEQUENCES.
                    jsr       RUNINIT             ; INITALIZE RUNTIME VARIABLES.
                    lda       #1                  ; SET THE RUN MODE FLAG.
                    sta       RUNFLAG

;        END OF POINTER INITIALIZATIONS

                    ldy       BASBEG              ; POINT TO THE START OF THE PROGRAM.
                    cpy       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    beq       Done@@              ; YES. GO RUN IT.

MainLoop@@          ldd       ,y                  ; GET NUMBER OF FIRST/NEXT LINE OF BASIC PROGRAM.
                    std       CURLINE             ; MAKE IT THE CURRENT LINE.
                    tst       TRFLAG              ; IS THE TRACE MODE TURNED ON?
                    beq       Go@@                ; NO. CONTINUE.

                    lda       #'['                ; YES. PRINT THE CURRENT LINE.
                    jsr       OUTBYTE
                    ldd       CURLINE
                    jsr       OUTDECI
                    lda       #']'
                    jsr       OUTBYTE
                    jsr       NL

Go@@                pshy                          ; SAVE POINTER TO START OF NEW LINE.
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
                    bne       MainLoop@@          ; NO. GO EXECUTE THE NEXT LINE.
                    jmp       REND                ; GO DO AN "END".

Cont@@              iny                           ; MUST BE A MID EOL.
                    bra       Loop@@              ; GO DO NEXT KEYWORD.
Done@@              rts

;*******************************************************************************

RSKIPSPC            proc
                    lda       ,y                  ; GET A CHARACTER.
                    bmi       Done@@
                    cmpa      #SSCNTOK            ; IS IT A SINGLE SPACE?
                    beq       Bump@@              ; YES. BUMP IP BY 1.
                    blo       Done@@
                    iny                           ; BUMP IP BY 2 FOR MULTIPLE SPACES.
Bump@@              iny                           ; BUMP IP.
Done@@              rts

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
                    beq       Done@@
                    iny                           ; MUST BE A MID EOL.
                    bra       Loop@@
Done@@              rts

;*******************************************************************************

CHCKBRK             proc
                    lda       #10                 ; RELOAD THE BREAK CHECK COUNT.
                    sta       BREAKCNT
                    jsr       CONSTAT             ; GET CONSOLE STATUS. CHARACTER TYPED?
                    beq       Done@@              ; YES. GO CHECK IT OUT.

                    jsr       INCONNE             ; GET BYTE FROM CONSOLE BUT DON'T ECHO.
                    cmpa      #$03                ; WAS IT A CONTROL-C?
                    bne       Done@@              ; NO. RETURN.

CHCKBRK2            sty       IPSAVE              ; SAVE THE IP POINTER IN CASE OF A CONTINUE.
                    jsr       NL
                    ldx       #Msg@@              ; POINT TO BREAK STRING.
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    jsr       NL
                    jmp       TopLoop

Done@@              equ       :AnRTS

Msg@@               fcc       "BREAK"

;*******************************************************************************

RUNINIT             proc
                    bsr       CCLEAR              ; GO CLEAR ALL VARIABLE STORAGE.
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
                    clrx                          ; CLEAR THE DATA POINTER.
                    stx       DATAPTR
                    rts

;*******************************************************************************

CCONT               proc
                    jsr       NL2
                    tst       CONTFLAG
                    bne       Error@@
                    ldy       IPSAVE
                    jmp       CRUN7

Error@@             lda       #CNTCNERR
                    sta       ERRCODE
                    jmp       PrintErrNum

;*******************************************************************************

CNEW                proc
                    ldx       EEStart
                    lda       AUTOSTF,x           ; GET THE AUTO START FLAG.
                    cmpa      #$55                ; IS IT SET?
                    bne       Go@@                ; NO. GO INITIALIZE EVERYTHING.
                    lda       #$FF                ; YES. RESET (ERASE) IT.
                    sta       AUTOSTF,x
                    jsr       DLY10MS
Go@@                jmp       INITVARS            ; INITIALIZE EVERYTHING.

;*******************************************************************************

CCLEAR              proc
                    bsr       RUNINIT1            ; GO INITALIZE ALL STACKS ETC.
CCLEAR3             ldx       VARBEGIN
Loop@@              lda       ,x
                    beq       Done@@
                    inx
                    inx
                    inx
                    jsr       CLRVAR
                    bra       Loop@@

Done@@              ldx       VAREND
                    inx
                    stx       STRASTG
                    rts

                    title     COMMAND2

;*******************************************************************************

CESAVE              proc
                    ldd       BASBEG              ; GET POINTER TO THE START OF THE BASIC PROGRAM.
                    cpd       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    beq       Done@@              ; YES. GO SAVE IT.

                    ldd       VAREND
                    subd      BASBEG
                    cpd       EESize
                    bls       Go@@
                    lda       #EETOSMAL
                    jmp       RPTERR

Go@@                ldx       EEStart             ; point to the start of the EEPROM.
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
Program@@           lda       ,y
                    sta       SSTART,x
                    jsr       DLY10MS
                    inx
                    iny
                    cpy       VAREND
                    bls       Program@@
Done@@              rts

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

;*******************************************************************************

LoadEE              proc
Loop@@              lda       SSTART,x            ; get a byte of the program.
                    sta       ,y                  ; put it in the program buffer.
                    inx                           ; point to the next program byte
                    iny                           ; point to the next buffer location.
                    cpy       VAREND              ; have we finished loading the program.
                    bls       Loop@@              ; no. keep loading.
                    sty       STRASTG             ; yes. initialize the array storage area.
                    rts

;*******************************************************************************

CLLIST              proc
                    lda       #1                  ; USE DEVICE #1 FOR HARD COPY LISTING.
                    sta       DEVNUM
                    jsr       CLIST               ; GO DO A STANDARD LIST COMMAND.
                    clr       DEVNUM
                    rts

;*******************************************************************************

CAUTOST             proc                          ; SET AUTO START MODE FOR BASIC PROGRAM.
                    lda       #$55                ; GET FLAG.
;                   bra       SaveAutoFlag

;*******************************************************************************

SaveAutoFlag        proc
                    ldx       EEStart
                    sta       AUTOSTF,x           ; PROGRAM IT INTO THE EEPROM
                    jmp       DLY10MS             ; WAIT WHILE IT PROGRAMS.

;*******************************************************************************

CNOAUTO             proc
                    lda       #$FF
                    bra       SaveAutoFlag

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
                    bra       LoadEE

;*******************************************************************************

CFREE               proc
                    jsr       NL2
                    ldd       VARMEND
                    subd      STRASTG
                    jsr       OUTDECI
                    jmp       NL

;*******************************************************************************

CDUMP               equ       :AnRTS

#ifdef ;------------------------------------------------------------------------
Exit@@              equ       :AnRTS
No@@                equ       :AnRTS

                    jsr       NL2                 ; PRINT TWO BLANK LINES.
                    clr       DNAME+2             ; ZERO THE LAST BYTE OF THE VARIABLE NAME 'ARRAY'
                    ldx       VARBEGIN            ; POINT TO THE START OF THE VARIABLE TABLE.
Loop@@              lda       ,x                  ; GET AN ENTRY. IS IT THE END OF THE TABLE?
                    beq       Exit@@              ; YES. WE'RE DONE.

                    lda       1,x                 ; NO. GET THE FIRST CHARACTER OF THE NAME.
                    sta       DNAME
                    lda       2,x
                    sta       DNAME+1
                    ldx       #DNAME
                    jsr       PL
                    lda       ,x                  ; GET THE VARIABLE TOKEN.
                    cmpa      #IVARTOK            ; IS IT AN INTEGER?
                    beq       Equal@@             ; YES. DUMP ITS VALUE.
                    cmpa      #IAVARTOK           ; NO. IS IT AN INTEGER ARRAY?
                    bne       No@@                ; NO.
                    ldd       3,x                 ; YES. GET THE POINTER TO THE ARRAY STORAGE. HAS IT BEEN DIMENSIONED?
                    bne       Print@@             ; YES. GO PRINT ALL THE VALUES.
                    ldx       #UnDimMsg@@
                    jsr       PL
Cont@@              ldb       #5
                    abx
                    bra       Loop@@

Print@@             pshx                          ; SAVE THE POINTER TO THE VARIABLE TABLE.
                    xgdx                          ; POINT TO THE ARRAY STORAGE AREA.
                    ldd       ,x                  ; GET THE MAXIMUM SUBSCRIPT.
                    std       SUBMAX
                    clrd
                    std       SUBCNT
PLoop@@             lda       #'('
                    jsr       OUTBYTE
                    ldd       SUBCNT
                    jsr       OUTDECI
                    ldx       #EqMsg@@
                    jsr       PL
                    inx
                    inx
                    ldd       ,x
                    jsr       OUTDECI
                    jsr       NL
                    ldd       SUBCNT
                    incd
                    cmpd      SUBMAX
                    bhi       PDone@@
                    std       SUBCNT
                    ldx       #DNAME
                    jsr       PL
                    bra       PLoop@@

PDone@@             pulx
                    bra       Cont@@

Equal@@             lda       #'='
                    jsr       OUTBYTE
                    ldd       3,x
                    jsr       OUTDECI
                    jsr       NL
                    bra       Cont@@

UnDimMsg@@          fcs       '=[?]'
EqMsg@@             fcs       ')='

#endif ;------------------------------------------------------------------------
                    title     RUNTIME1

;*******************************************************************************

RREM                proc                          ; NON-EXECUTABLE STATEMENT JUST SKIP IT.
RDATA               proc
                    ldb       ,y                  ; GET LENGTH OF REMARK OR DATA LINE.
                    aby                           ; POINT TO THE EOLTOK.
                    rts

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
                    bne       Pointer@@           ; YES. GO GET POINTER FOR A STRING DESCRIPTOR.
                    bita      #$10                ; IS IT A NUMERIC ARRAY VARIABLE?
                    bne       Subscr@@            ; YES. GO CALCULATE THE SUBSCRIPT.
Loop@@              ldd       1,y                 ; GET THE OFFSET TO THE DICTIONARY ENTRY.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE DCTIONARY.
                    addd      #3                  ; MAKE POINTER POINT TO THE ACTUAL STORAGE LOCATION
                    pshb                          ; SAVE B.
                    ldb       #3                  ; POINT TO THE FIRST ELEMENT PAST THE VARIABLE.
                    aby
                    pulb
                    rts

Subscr@@            jsr       CALCSUB             ; GO GET BASE ADDR & EVALUATE SUBSCRIPT EXPRESSION.
                    pshx                          ; PUSH BASE ADDRESS ONTO STACK.
                    tsx                           ; POINT TO IT.
                    lsld                          ; MULT THE SUBSCRIPT BY THE # OF BYTES/ELEMENT.
Done@@              addd      ,x                  ; GET ADDRESS OF ELEMENT.
                    pulx
                    rts

Pointer@@           bita      #$10                ; IS IT A STRING ARRAY?
                    beq       Loop@@              ; NO. JUST GO GET POINTER TO DESCRIPTOR.
                    jsr       CALCSUB             ; GET BASE ADDR. & CALC SUBSCRIPT.
                    pshx                          ; SAVE THE BASE ADDRESS.
                    pshd                          ; SAVE THE SUBSCRIPT VALUE.
                    tsx                           ; POINT TO THE VALUES.
                    lsld                          ; MULT BY 2.
                    addd      ,x                  ; MULT BY 3.
                    ins                           ; GET RID OF SUBSCRIPT VALUE.
                    ins
                    tsx                           ; POINT TO BASE ADDRESS.
                    bra       Done@@

;*******************************************************************************

RGOTO               proc
                    tst       IMMID               ; DID WE ENTER HERE FROM THE IMMIDIATE MODE?
                    beq       Go@@                ; NO. JUST GO DO A GOTO.
                    ldd       BASEND              ; YES. SET ADRNXLIN TO END OF PROGRAM SO THE
                    std       ADRNXLIN            ; SEARCH STARTS AT THE FIRST LINE.
Go@@                ldx       ADRNXLIN            ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; IS THIS THE LAST LINE OF THE PROGRAM?
                    bne       Search@@            ; NO. SEARCH STARTING AT THE NEXT LINE.
RGOTO3
ToStart@@           ldx       BASBEG              ; YES. POINT TO THE START OF THE BASIC PROGRAM.
                    bra       Loop@@

Search@@            ldd       ,x                  ; GET THE NEXT LINE NUMBER IN THE PGM.
                    cpd       1,y                 ; IS IT > THE LINE NUMBER WE ARE TO "GOTO"?
                    bhi       ToStart@@           ; YES. START THE SEARCH AT THE BEGINING.
Loop@@              ldd       ,x                  ; GET THE NEXT LINE NUMBER INTO D.
                    cpd       1,y                 ; IS THIS THE CORRECT LINE?
                    beq       Goto@@              ; YES. "GOTO" THE NEW LINE.
                    blo       Lower@@             ; NO. IS IT LESS THAN THE "TARGET LINE"?
Error@@             lda       #LNFERR             ; NO. THE LINE MUST NOT EXIST.
                    jmp       RPTRERR             ; REPORT THE ERROR & RETURN TO MAIN LOOP.

Lower@@             ldb       2,x                 ; GET THE LENGTH OF THIS LINE.
                    abx                           ; POINT TO THE START OF THE NEXT LINE.
                    cpx       BASEND              ; DID WE HIT THE END OF THE PROGRAM?
                    beq       Error@@             ; YES. THE LINE DOESN'T EXIST.
                    bra       Loop@@              ; NO. GO SEE IF THIS IS THE CORRECT LINE.

Goto@@              xgdx                          ; MAKE IT THE NEW IP.
                    xgdy
                    tst       IMMID
                    beq       RGOTO8
                    clr       IMMID
RGOTO9              jmp       CRUN1

RGOTO8              ins
                    ins
                    bra       RGOTO9

;*******************************************************************************

RGOSUB              proc
                    pshy                          ; SAVE THE I.P. TO THE LINE NUMBER.
                    tst       IMMID               ; DID WE GET HERE FROM THE IMMIDIATE MODE?
                    beq       Go@@                ; NO. GO DO A NORMAL GOSUB.
                    ldy       BASEND              ; YES. MAKE RETURN POINT TO THE LAST EOL TOKEN
                    dey                           ; IN THE PROGRAM.
                    bra       RGOSUB2             ; GO PUT IT ON THE ARGUMENT STACK.

Go@@                ldb       #3                  ; BYPASS THE LINE NUMBER.
                    aby
RGOSUB2             jsr       RSKIPSPC            ; SKIP SPACES AFTER THE LINE NUMBER.
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    dex                           ; POINT TO THE NEXT ENTRY ON THE STACK.
                    dex
                    cpx       EGOSTK              ; OUT OF STACK SPACE?
                    bhs       Done@@              ; NO. GO PUSH THE "RETURN ADDRESS" ON THE STACK.
                    lda       #GOSOVERR           ; YES. GET THE ERRCODE.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Done@@              stx       GOSTACK             ; SAVE THE "GOSUB" STACK POINTER.
                    sty       ,x                  ; PUT THE RETURN ADDRESS ON THE STACK.
                    puly                          ; GET THE POINTER TO THE LINE NUMBER.
                    bra       RGOTO               ; GO DO A "GOTO".

;*******************************************************************************

RRETURN             proc
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    cpx       STGOSTK             ; IS THERE A RETURN ADDRESS ON THE GOSUB STACK?
                    bne       Done@@              ; YES. GO RETURN.
                    lda       #RWOGERR            ; NO. RETURN W/O GOSUB ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

Done@@              ldy       ,x                  ; GET THE RETURN ADDRESS IN THE IP.
                    inx                           ; REMOVE THE ADDRESS FROM THE STACK.
                    inx
                    stx       GOSTACK             ; SAVE THE STACK POINTER.
                    rts                           ; BACK TO THE MAIN INTERPRET LOOP.

;*******************************************************************************

RSTOP               proc
                    ldx       #Msg@@
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    sty       IPSAVE
                    bra       ?REND

Msg@@               fcs       LF,CR,"STOPPED AT LINE # "

;*******************************************************************************

REND                proc
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
?REND               clrd
                    std       CURLINE
                    jmp       TopLoop

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
Loop@@              pshb
                    ldy       ADRNXLIN            ; GET THE ADDRESS OF THE NEXT LINE.
                    beq       Done@@

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

Done@@              equ       :AnRTS

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
                    bne       Done@@              ; YES. GO EXECUTE CODE BETWEEN WHILE & ENDWH.
                    puly                          ; NO. GET THE ADDRESS OF THE NEXT LINE/STATEMENT.
                    ldx       WHSTACK             ; GET WHILE STACK POINTER.
                    inx:2                         ; TAKE ADDRESS OFF OF WHILE STACK.
                    stx       WHSTACK             ; SAVE STACK POINTER.
                    bra       Exit@@              ; GO TO INTERPRET LOOP.

Done@@              ins:2                         ; REMOVE POINTER TO STATEMENT AFTER "ENDWH" FROM STACK.
Exit@@              rts                           ; GO EXECUTE LINES TILL "ENDWH".

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
                    bpl       Loop@@              ; IS IT NEGATIVE?
                    bne       Loop@@              ; OR ZERO?
Error@@             lda       #ONARGERR           ; YES. REPORT ERROR.
                    jmp       RPTRERR

Loop@@              ldd       ,x                  ; GET THE EXPRESSION VALUE.
                    subd      #1                  ; SUBTRACT 1. HAVE WE FOUND THE LINE NUMBER?
                    beq       Goto@@              ; YES. GO DO "GOTO" OR "GOSUB".
                    std       ,x                  ; NO. SAVE REMAINDER.
                    ldb       #3                  ; POINT PAST THE LINE NUMBER VALUE.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES PAST THE LINE NUMBER.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #EOLTOK             ; HAVE WE HIT THE END OF THE LINE?
                    beq       Error@@             ; YES. ERROR.
                    iny                           ; NO. MUST BE A COMMA. BYPASS IT.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    bra       Loop@@              ; GO SEE IF THE NEXT LINE NUMBER IS THE ONE.

Goto@@              jsr       PULNUM              ; GET RID OF ARGUMENT.
                    pula                          ; GET "GO" TOKEN.
                    cmpa      #GOTOTOK            ; IS IT A "GOTO" TOKEN?
                    jeq       RGOTO               ; GO DO A "GOTO".

                    pshy                          ; SAVE THE POINTER TO THE LINE NUMBER.
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

Gosub@@             jmp       RGOSUB2             ; GO DO A "GOSUB".

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
?AsgnPort           ldx       #REGS               ; GET ADDRESS OF PORTA I/O REGISTER.
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
                    bra       ?AsgnPort           ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTC              proc
                    ldb       #PORTCIO            ; GET ADDRESS OF PORTC I/O REGISTER.
                    bra       ?AsgnPort           ; GO DO AN ASIGNMENT.

;*******************************************************************************

RPORTD              proc
                    ldb       #PORTDIO            ; GET ADDRESS OF PORTD I/O REGISTER.
                    bra       ?AsgnPort           ; GO DO AN ASIGNMENT.

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
                    rts

;*******************************************************************************

RPRINT              proc
                    jsr       CHCKDEV             ; GO CHECK FOR ALTERNATE OUTPUT DEVICE.
                    lda       ,y                  ; GET FIRST TOKEN.
                    cmpa      #EOLTOK             ; IS IT AN EOL TOKEN?
                    beq       CrLf@@              ; YES. JUST PRINT A CR/LF.
                    cmpa      #MEOLTOK            ; IS IT A MID EOL TOKEN?
                    bne       Loop@@              ; NO. GO PRINT A STRING OR NUMBER.
CrLf@@              jsr       NL                  ; YES. JUST PRINT A CR/LF.
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
                    bra       Cont@@              ; GO DO NEXT EXPRESSION.

Func?@@             cmpa      #FUNCTFLG           ; IS IT A FUNCTION?
                    bne       Hex2?@@             ; NO. GO EVALUATE A NUMERIC EXPRESSION.
                    lda       1,y                 ; GET THE FUNCTION TYPE.
                    cmpa      #TABTOK             ; IS IT A TAB?
                    bne       Chr$?@@             ; NO GO CHECK FOR "CHR$".
                    bsr       RTAB                ; GO DO TAB.
                    bra       Cont@@              ; GO SEE IF THERE'S MORE TO PRINT.

Chr$?@@             cmpa      #CHRTOK             ; IS IT THE CHR$ FUNCTION.
                    bne       Hex?@@              ; NO. GO CHECK FOR HEX().
                    bsr       RCHRS               ; YES. GO DO CHR$.
                    bra       Cont@@              ; GO SEE IF THERE'S MORE TO PRINT.

Hex?@@              cmpa      #HEXTOK             ; IS IT THE HEX() FUNCTION?
                    bne       Hex2?@@             ; NO. GO DO A NUMERIC EXPRESSION.
                    bsr       RHEX                ; YES. GO PRINT THE NUMBER AS HEX.
                    bra       Cont@@              ; GO SEE IF THERE'S MORE TO PRINT.

Hex2?@@             cmpa      #HEX2TOK            ; IS IT THE HEX2() FUNCTION?
                    bne       NumExp@@            ; NO. GO DO A NUMERIC EXPRESSION.
                    bsr       RHEX2               ; YES GO PRINT A NUMBER >=255 AS 2 HEX BYTES.
                    bra       Cont@@              ; GO SEE IF THERE'S MORE TO PRINT.

NumExp@@            jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE NUMERIC STACK.
                    jsr       OUTDECI             ; PRINT IT.
                    lda       #SPC                ; PUT A TRAILING SPACE AFTER ALL NUMBERS.
                    jsr       OUTBYTE             ; PRINT IT.
Cont@@              jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET SEPERATOR CHARACTER.
                    cmpa      #COMMATOK           ; IS IT A COMMA?
                    beq       Comma@@             ; YES.
                    cmpa      #SEMITOK            ; IS IT A SEMICOLIN?
                    bne       Eol@@               ; NO. MUST BE AN EOLTOK.
                    iny                           ; DO NOTHING BUT BUMP THE IP.
                    bra       ToEol@@             ; GO CHECK FOR EOL AFTER COMMA OR SEMICOLIN.

Comma@@             iny                           ; BUMP IP PAST THE COMMATOK.
                    ldb       PRINTPOS            ; YES. "TAB" TO NEXT PRINT FIELD.
                    andb      #$07                ; MASK OFF ALL BUT THE FIELD WIDTH.
                    negb                          ; MAKE IT NEGATIVE.
                    addb      #8                  ; ADD IN THE FIELD WIDTH. ARE WE ON A FIELD BOUND?
                    beq       ToEol@@             ; YES. GO CHECK FOR AN EOL.
                    lda       #SPC                ; NO. GET A SPACE & PRINT TILL WE GET THERE.
Space@@             jsr       OUTBYTE             ; PRINT A SPACE.
                    decb                          ; DECREMENT THE COUNT. ARE WE DONE?
                    bne       Space@@             ; NO. KEEP GOING.
ToEol@@             jsr       RSKIPSPC            ; SKIP ANY SPACES.
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
?RPTRERR            jmp       RPTRERR             ; REPORT ERROR.

Loop@@              cmpb      PRINTPOS            ; ARE WE ALREADY PAST THE "TAB" POSITION?
                    bls       Done@@              ; YES. DONE.
                    lda       #SPC                ; GET A SPACE.
                    jsr       OUTBYTE             ; PRINT IT.
                    bra       Loop@@

Done@@              equ       :AnRTS

;*******************************************************************************

RCHRS               proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       Done@@              ; YES. GO DO TAB.
                    lda       #CHRARGER           ; NO. ERROR.
                    bra       ?RPTRERR            ; REPORT ERROR.
Done@@              tba                           ; PUT BYTE INTO A
                    jmp       OUTBYTE             ; PRINT THE BYTE & RETURN.

;*******************************************************************************

RHEX2               proc
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       RHEX1               ; YES. GO PRINT 2 HEX CHARACTERS & RETURN.
                    lda       #HEX2AERR           ; NO. ARG. MUST BE >=0 & <=255.
                    bra       ?RPTRERR            ; GO REPORT ERROR.

;*******************************************************************************

RHEX                proc
                    bsr       PFUNCOM             ; GO DO COMMON CODE FOR PRINT FUNCTIONS
                    bsr       PRNT2HEX            ; GO PRINT 2 HEX CHARACTERS.
RHEX1               tba                           ; PUT LOWER BYTE IN A.
;                   bra       PRNT2HEX            ; PRINT 2 HEX CHARACTERS & RETURN.

;*******************************************************************************

PRNT2HEX            proc
                    psha                          ; SAVE THE CHARACTER.
                    bsr       Left@@              ; PRINT THE LEFT HEX NYBBLE.
                    pula                          ; GET BYTE BACK.
                    bra       Right@@             ; PRINT RIGHT NYBBLE & RETURN.

Left@@              lsra:4                        ; GET UPPER NYBBLE INTO LOWER ONE.

Right@@             anda      #$0F                ; MASK OFF UPPER NYBBLE.
                    adda      #'0'                ; MAKE IT A HEX NUMBER.
                    cmpa      #'9'                ; IS IT?
                    bls       Print@@             ; YES. PRINT IT.
                    adda      #7                  ; NO. MAKE IT A HEX LETTER.
Print@@             jmp       OUTBYTE             ; PRINT IT & RETURN.

;*******************************************************************************

PFUNCOM             proc
                    ldb       #3                  ; POINT PAST FUNCTION FLAG, FUNCTION TOKEN, &
                    aby                           ; OPEN PAREN.
                    jsr       DONEXP              ; GO GET POSITION TO TAB TO.
                    iny                           ; BUMP IP PAST CLOSING PAREN.
                    jsr       PULNUM              ; GET OPERAND OFF STACK.
                    tsta                          ; CHECK THAT OPERAND IS >0 & <=255 FOR FUNCTIONS THAT REQUIRE IT.
                    rts

;*******************************************************************************

RDIM                proc
                    lda       ,y                  ; GET VARIABLE FLAG/TYPE.
                    bita      #$10                ; IS IT A SUBSCRIPTED VARIABLE?
                    bne       Go@@                ; YES. GO DIMENSION IT.
                    lda       #NOSUBERR           ; NO. GET ERROR.
Error@@             jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                ldd       1,y                 ; GET THE OFFSET INTO THE DICTIONARY.
                    addd      VARBEGIN            ; ADD IN THE START OF THE DICTIONARY.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    ldd       3,x                 ; GET THE POINTER TO THE STORAGE. BEEN DIMENSIONED?
                    beq       Dim@@               ; NO. GO DIMENSION IT.
                    lda       #REDIMERR           ; YES. ERROR.
                    bra       Error@@

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
                    bpl       Pos@@               ; ONLY POSITIVE SUBSCRIPTS ALLOWED.
                    lda       #NEGSUBER           ; NEGATIVE NUMBER.
                    bra       Error@@             ; REPORT ERROR.

Pos@@               pshx
                    ldx       3,x                 ; GET POINTER TO STORAGE.
                    std       ,x                  ; PUT MAX SUBSCRIPT IN POOL STORAGE.
                    addd      #1                  ; COMPENSATE FOR "0" SUBSCRIPT.
                    pulx                          ; RESTORE POINTER TO DICTIONARY ENTRY.
                    lsld                          ; MULT. BY 2 (2 BYTES/INTEGER).
                    addd      STRASTG             ; ADD IN CURRENT POINTER TO POOL.
                    cpd       STRASTG             ; WAS THE SUBSCRIPT SO BIG WE WRAPPED AROUND?
                    bls       MemErr@@            ; YES. ERROR.
                    cpd       VARMEND             ; DO WE HAVE ENOUGH MEMORY?
                    bls       Save@@              ; YES.
MemErr@@            lda       #OMEMERR            ; NO. ERROR.
                    bra       Error@@             ; GO REPORT THE ERROR.

Save@@              std       STRASTG             ; SAVE POINTER.
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
Done@@              equ       :AnRTS              ; BACK TO MAIN INTERPRET LOOP.

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
Loop@@              std       2,x                 ; PUT IT IN THE STACK.
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
                    bra       Loop@@              ; GO PUT VALUE IN STACK.

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
                    bvs       Loop@@              ; Bug fix (2000.03.28 <tonyp@acm.org>). On overflow, exit

                    std       ,y                  ; SAVE THE RESULT.
                    tst       2,x                 ; IS THE STEP VALUE NEGATIVE?
                    bmi       Cont@@              ; YES. GO DO TEST.
                    cpd       4,x                 ; NO. ARE WE DONE?
                    ble       Done@@              ; NO. GO DO THE LOOP AGAIN.
Loop@@              puly                          ; RESTORE THE CURRENT IP.
                    xgdx                          ; PUT "FOR - NEXT" STACK POINTER IN D.
                    addd      #10                 ; REMOVE DESCRIPTOR FROM STACK.
                    std       FORSTACK            ; SAVE NEW STACK VALUE.
                    jmp       RSKIPSPC            ; SKIP SPACES AFTER CONTROL VARIABLE.

Cont@@              cpd       4,x                 ; ARE WE DONE?
                    blt       Loop@@              ; YES. CONTINUE.
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
                    inx                           ; POINT TO STRING.
                    inx
                    jsr       OUTSTR              ; GO PRINT THE STRING.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER COMMA.
                    bra       Done@@

Loop@@              jsr       NL
Done@@              ldx       #Msg@@              ; POINT TO PROMPT.
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
                    bne       Done@@              ; NO. RETURN.

                    iny                           ; YES. PASS THE '#' TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; GO EVALUATE THE NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    bpl       Go@@                ; NEGATIVE NUMBERS NOT ALLOWED.
Error@@             lda       #ILLIOERR           ; REPORT THE ERROR.
                    jmp       RPTRERR

Go@@                cpd       #7                  ; IS IT LARGER THAN 7?
                    bhi       Error@@
                    stb       DEVNUM              ; MAKE IT THE NEW DEVICE NUMBER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    cmpa      #EOLTOK             ; IF THIS IS A PRINT STATEMENT, IS IT EOL?
                    beq       Done@@              ; YES. DON'T BUMP THE IP.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
Done@@              rts

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
                    beq       Eol@@
                    cmpa      #MEOLTOK
                    beq       Eol@@
                    iny                           ; BUMP PAST THE COMMA.
                    jsr       RSKIPSPC
                    bra       RINRDC

Eol@@               clc
                    rts

;*******************************************************************************

INNUMD              proc
                    cmpa      #'$'
                    bne       Decimal@@
                    jsr       INCIBP
                    jsr       GETHEX
                    bra       Go@@

Decimal@@           bsr       INDECI
Go@@                pshd
                    jsr       SKIPSPCS
                    cmpa      #COMMA
                    beq       Comma@@
                    cmpa      #EOL
                    beq       Done@@
                    lda       #MCOMAERR
                    jmp       RPTRERR

Comma@@             jsr       INCIBP
Done@@              jsr       RVARPTR
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
                    bne       Pos@@               ; NO. GO GET POSITIVE NUMBER.
                    jsr       INCIBP              ; YES. BUMP INPUT BUFFER PAST IT.
                    jsr       GETDECI             ; GET THE NUMBER.
                    negd                          ; NEGATE IT.
                    rts

Pos@@               jmp       GETDECI

;*******************************************************************************

RREAD               proc
                    ldx       DATAPTR             ; GET POINTER TO DATA. IS IT POINTING TO DATA?
                    bne       Go@@                ; YES. CONTINUE TO READ DATA.
                    bsr       RRESTOR             ; NO. GO GET POINTER TO FIRST DATA STATEMENT.
                    ldx       DATAPTR             ; GET POINTER TO DATA.
Go@@                stx       IBUFPTR             ; PUT IT IN THE INPUT BUFFER POINTER.
                    bsr       RINRDC              ; GO USE INPUT/READ COMMON CODE.
                    bcs       Cont@@              ; IF CARRY SET, MORE DATA TO READ.
                    ldx       IBUFPTR             ; GET POINTER TO DATA LINE.
                    stx       DATAPTR             ; SAVE DATA POINTER FOR NEXT READ.
                    rts

Cont@@              pshy
                    ldy       IBUFPTR
                    iny:2
                    bsr       RESTOR4             ; GO FIND NEXT "DATA" STATEMENT.
                    puly
                    bra       RREAD               ; KEEP READING DATA.

;*******************************************************************************

RRESTOR             proc
                    pshy
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
                    puly
                    rts

RESTOR4             pshy                          ; CALL TO COMPENSATE FOR PULL OF Y ON RETURN.
                    bra       Cont@@

;*******************************************************************************

RIF                 proc
                    jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; SKIP PAST "THEN" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THEN.
                    jsr       PULNUM              ; GET RESULT OF EXPRESSION FROM OPERAND STACK.
                    beq       Skip@@              ; NOT TRUE. SEE IF ELSE CLAUSE PRESENT.
Goto@@              jmp       RGOTO               ; RESULT WAS TRUE. GOTO PROPER LINE NUMBER.

Skip@@              ldb       #3                  ; BUMP IP PAST LINE NUMBER.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES IF PRESENT.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #ELSETOK            ; IS IT THE "ELSE" CLAUSE.
                    bne       Done@@              ; NO RETURN.
                    iny                           ; PASS ELSE TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    bra       Goto@@              ; DO A GOTO.

Done@@              equ       :AnRTS

;*******************************************************************************

REEP                proc                          ; PROGRAM A WORD OF EEPROM.
                    iny                           ; PASS UP THE OPEN PAREN.
                    jsr       RSKIPSPC            ; PASS UP ANY SPACES.
                    jsr       DONEXP              ; GO GET THE "SUBSCRIPT" OF THE EEPROM LOCATION.
                    iny:2                         ; PASS UP THE CLOSING PAREN AND THE EQUALS TOKEN.
                    jsr       DONEXP              ; GET VALUE TO FROGRAM INTO EEPROM.
                    pshy                          ; SAVE THE Y REG.
                    ldy       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    ldd       2,y                 ; GET THE SUBSCRIPT FOR THE EEPROM LOCATION.
                    bmi       Error@@             ; NEGATIVE SUBSCRIPTS NOT ALLOWED.
                    cpd       #MAXEESUB           ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       Go@@                ; YES. CONTINUE.
Error@@             lda       #EESUBERR           ; EEPROM SUBSCRIPT ERROR.
                    jmp       RPTRERR             ; REPORT IT.

Go@@                lsld                          ; MULT THE SUBSCRIPT BY 2.
                    addd      #EEPBASAD           ; ADD IN THE EEPROM BASE ADDRESS.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    lda       ,x                  ; GET THE MOST SIGNIFIGANT BYTE OF THE CURRENT NUM.
                    cmpa      #$FF                ; DOES IT NEED ERASING?
                    beq       Cont@@              ; NO. SEE IF NEXT BYTE NEEDS ERASING.
                    bsr       ERASEBYT            ; YES. GO ERASE IT.
Cont@@              inx                           ; POINT TO NEXT BYTE.
                    lda       ,x                  ; GET NEXT BYTE.
                    cmpa      #$FF                ; DOES THIS BYTE NEED TO BE ERASED?
                    beq       Write@@             ; NO. GO WRITE DATA TO EEPROM.
                    bsr       ERASEBYT            ; YES. GO ERASE THE BYTE.
Write@@             lda       1,y                 ; GET LS BYTE OF WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    dex                           ; POINT TO THE MOST SIGNIFIGANT EEPROM LOCATION.
                    lda       ,y                  ; GET THE MS BYTE OF THE WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    puly
                    jsr       PULNUM              ; FIX UP NUM STACK.
                    jmp       PULNUM

;*******************************************************************************

ERASEBYT            proc
                    ldb       #$16                ; SET UP BYTE ERASE MODE, ADDR LATCH, ERASE
                    stb       PPROG               ; VOLTAGE OFF.
                    sta       ,x                  ; LATCH ADDRESS.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE ERASING.
                    inc       PPROG               ; TURN ON ERASE VOLTAGE
                    bsr       DLY10MS             ; DELAY ABOUT 10 MS.
                    dec       PPROG               ; TURN PROGRAMING VOLTAGE OFF.
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG
                    rts

;*******************************************************************************

PROGBYTE            proc
Loop@@              ldb       #$02                ; SET UP NORMAL PROGRAMING MODE, ADDRESS/DATA
                    stb       PPROG               ; LATCHED, PROGRAMING VOLTAGE OFF.
                    sta       ,x                  ; LATCH DATA & ADDRESS.
                    psha                          ; SAVE THE DATA FOR COMPARE AFTER PROGRAMING.
                    tpa                           ; GET CURRENT I-BIT STATUS.
                    psha                          ; SAVE IT.
                    sei                           ; INHIBIT INTERRUPTS WHILE PROGRAMING.
                    inc       PPROG               ; TURN ON PROGRAMING VOLTAGE.
                    bsr       DLY10MS             ; LEAVE IT ON FOR 10 MS.
                    dec       PPROG               ; NOW, TURN THE PROGRAMMING VOLTAGE OFF.
                    pula                          ; GET ORIGINAL I-BIT STATUS.
                    tap                           ; RESTORE IT.
                    clr       PPROG               ; PUT THE EEPROM BACK IN THE READ MODE.
                    pula                          ; RESTORE THE DATA TO SEE IF IT WAS PROGRAMMED.
                    cmpa      ,x                  ; WAS THE DATA WRITTEN PROPERLY?
                    bne       Loop@@              ; NO. TRY AGAIN.
                    rts

;*******************************************************************************
                              #Cycles
DLY10MS             proc
                    pshx
                    ldx       #DELAY@@            ; GET DELAY CONSTANT.
                              #Cycles
Loop@@              dex                           ; DECREMENT THE COUNT. DONE?
                    bne       Loop@@              ; NO. DELAY SOME MORE.
                              #temp :cycles
                    pulx
                    rts

BUS_KHZ             def       2000
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
                    rts

;*******************************************************************************

RTIME               proc
                    iny                           ; POINT PAST THE EQUALS TOKEN.
                    jsr       DONEXP              ; GO EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    rts

;*******************************************************************************

RRTIME              proc
                    sei                           ; disable interrupts.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    ldd       TCNT                ; get the current value of the timer counter.
                    jsr       TIMINTS3            ; go initialize the TOC using the timer interrupt code.
                    clrd
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    cli
                    rts

;*******************************************************************************

RPACC               proc
                    iny                           ; POINT PAST EQUALS TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    tsta                          ; IS THE NUMBER WITHIN RANGE?
                    beq       Go@@                ; YES. GO SETUP THE PACC REGISTER.
                    lda       #PACCARGE           ; NO. REPORT AN ERROR.
                    jmp       RPTRERR

Go@@                stb       PACNT               ; PUT NUMBER IN PULSE ACC.
                    rts

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
                    bra       ?RonExit            ; GO FINISH UP.

;*******************************************************************************

RONIRQ              proc
                    bsr       CHCKIMID
                    jsr       DONEXP              ; GO CHECK TO SEE IF WE ARE TO ENABLE OR DISABLE.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO COMMA.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO LINE NUMBER.
                    jsr       PULNUM              ; GET MODE. SHOULD WE ENABLE THE FUNCTION?
                    bne       Go@@                ; YES.
                    std       ONIRQLIN            ; NO. MAKE THE LINE NUMBER 0.
                    bra       ?RonExit            ; GO FINISH UP.

Go@@                sty       ONIRQLIN            ; SAVE THE POINTER TO THE LINE NUMBER,
?RonExit            ldb       #3                  ; MOVE IP PAST THE LINE NUMBER.
                    aby
                    rts

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
                    ins:2                         ; TAKE THE RETURN ADDRESS OFF THE STACK.
                    rti                           ; GO BACK TO WHERE WE LEFT OFF.

;*******************************************************************************

CHCKIMID            proc
                    tst       IMMID               ; ARE WE IN THE IMMIDIATE MODE?
                    beq       Done@@              ; NO. JUST RETURN.
                    lda       #NOTALERR           ; YES. THIS COMMAND NOT ALLOWED.
                    jmp       RPTRERR             ; REPORT THE ERROR.
Done@@              equ       :AnRTS

;*******************************************************************************

RONPACC             proc
                    bsr       CHCKIMID            ; THIS INSTRUCTION NOT ALLOWED IN IMMID MODE.
                    bsr       DONEXP              ; GO EVALUATE THE COUNT MODE EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER COMMA.
                    bsr       DONEXP              ; GO EVALUATE THE INTERRUPT MODE EXPRESSION.
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
Error@@             lda       #INTMODER           ; NO. GET ERROR CODE.
                    jmp       RPTRERR

Go@@                lda       #$10                ; GET BIT TO ENABLE INTERRUPT.
                    tstb                          ; WAS THE ARGUMENT 0?
                    beq       Skip@@              ; YES. GO ENABLE INTS. ON EACH COUNT.
                    lsla                          ; NO. ENABLE INTS. ON PACC OVERFLOW ONLY.
Skip@@              sta       TMSK2
                    jsr       PULNUM              ; GET THE COUNT MODE OFF THE STACK.
                    bne       Range?@@            ; GO SET THE MODE IF NOT 0.
                    clr       PACTL               ; TURN OFF THE PULSE ACCUMULATOR.
                    std       ONPACLIN            ; CLEAR POINTER TO LINE NUMBER.
                    bra       Done@@              ; GO CLEAN UP & RETURN.

Range?@@            cpd       #4                  ; IS THE ARGUMENT IN RANGE?
                    bhi       Error@@             ; YES. REPORT AN ERROR.
                    addb      #3                  ; GET BIT TO ENABLE PACC.
                    lslb:4
                    stb       PACTL               ; ENABLE THE PACC & SET MODE.
Done@@              pula                          ; GET OLD I-BIT STATUS OFF STACK.
                    tap                           ; RESTORE OLD STATUS.
                    ldb       #3
                    aby                           ; PASS UP LINE NUMBER.
                    rts

                    title     REXPRES

;*******************************************************************************
;               RUNTIME EXPRESSION EVALUATION SUBROUTINE                       *
;*******************************************************************************

DONEXP              proc
                    lda       #OPARNTOK           ; USE AN OPEN PAREN AS AN END OF EXPRESSION MARKER.
                    jsr       PSHOP               ; PUSH OPEN PAREN ON THE STACK.
Loop@@              lda       ,y                  ; GET THE NEXT CHARACTER IN THE EXPRESSION.
                    cmpa      #OPARNTOK           ; IS IT AN OPEN PAREN?
                    bne       Op?@@               ; NO. CONTINUE.
                    iny                           ; POINT TO NEXT TOKEN.
                    bsr       DONEXP              ; GO DO A SUBEXPRESSION.
                    iny                           ; MOVE THE IP PAST THE CLOSING PAREN.
                    bra       Loop@@              ; GO GET THE NEXT CHARACTER.

Op?@@               tsta                          ; CHECK FOR OPERATOR OR OPERAND.
                    bpl       Func?@@             ; IF NOT VARIABLE OR CONSTANT, GO CHECK FOR FUNCT.
                    bsr       PSHNUM              ; GO PUSH OPERAND ONTO STACK.
                    bra       Loop@@              ; GO GET NEXT TOKEN.

Func?@@             jsr       CHKNFUN             ; GO CHECK FOR FUNCTION THAT RETURNS A NUMBER.
                    jsr       CHCKEE              ; GO CHECK FOR END OF EXPRESSION.
                    bcs       Done@@              ; IF NOT END OF EXPRESSION, GO PUSH OPERATOR.

                    iny                           ; POINT TO THE NEXT TOKEN.
                    jsr       PSHOP               ; PUSH OPERATOR ONTO STACK.
                    bra       Loop@@              ; GO GET NEXT TOKEN.
Done@@              equ       :AnRTS

;*******************************************************************************
;        PSHNUM SUBROUTINE
;
;        PUSHES A NUMERIC OPERAND (CONSTANT OR VARIABLE) VALUE ONTO THE
;        OPERAND STACK.

PSHNUM              proc
                    cmpa      #IVARTOK            ; IS IT AN INTEGER SCALER VARIABLE?
                    bne       Const?@@            ; NO. GO CHECK FOR CONSTANT.
                    ldd       1,y                 ; YES. GET THE "OFFSET" ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE VARIABLE TABLE.
                    xgdx                          ; GET THE ADDRESS INTO X.
                    ldb       #3                  ; BUMP INTERPRETER POINTER PAST "VARIABLE".
                    aby
                    ldd       3,x                 ; GET THE VARIABLE VALUE.
                    bra       PushOperand         ; GO PUT IT ON THE STACK.

Const?@@            cmpa      #ICONTOK            ; IS IT AN INTEGER CONSTANT?
                    bne       Array?@@            ; NO. GO CHECK FOR AN INTEGER ARRAY VARIABLE.
                    ldx       1,y                 ; GET THE CONSTANT VALUE INTO X.
                    ldb       #4
                    addb      3,y
                    aby
                    xgdx                          ; PUT THE CONSTANT VALUE INTO D.
                    bra       PushOperand         ; GO PUT IT ON THE STACK.

Array?@@            cmpa      #IAVARTOK           ; IS IT AN INTEGER ARRAY?
                    bne       Error@@             ; NO. GO CHECK FOR A STRING VARIABLE.
                    bsr       CALCSUB             ; GO GET BASE ADDR. & SUBSCRIPT OF ARRAY.
                    pshy                          ; SAVE THE INTERPRETER POINTER.
                    pshx                          ; PUT THE BASE ADDRESS OF THE ARRAY ON THE STACK.
                    asld                          ; MULTIPLY THE SUBSCRIPT BY THE # OF BYTES/ELEMENT.
                    tsy                           ; POINT TO THE BASE ADDRESS.
                    addd      ,y                  ; GET ADDRESS OF THE ELEMENT.
                    pulx
                    puly
                    xgdx                          ; PUT ELEMENT ADDRESS INTO X.
                    ldd       ,x                  ; GET VALUE OF ELEMENT IN D.
                    bra       PushOperand

Error@@             lda       #ILTOKERR
                    jmp       RPTRERR

;*******************************************************************************

PushOperand         proc
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    dex:2                         ; MAKE ROOM ON THE STACK FOR NEW OPERAND.
                    cpx       ENUMSTK             ; HAS THE STACK OVERFLOWED?
                    bhs       Done@@              ; NO. GO STACK THE VALUE.
                    lda       #MSTKOERR           ; YES.
                    sta       ERRCODE
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Done@@              stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    std       ,x                  ; PUT THE VALUE ON THE STACK.
                    rts

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
Error@@             jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                ldb       #4                  ; SET POINTER TO START OF SUBSCRIPT EXPRESSION.
                    aby
                    pshx                          ; SAVE THE POINTER TO THE ARRAY STORAGE AREA.
                    jsr       DONEXP              ; GO GET THE SUBSCRIPT.
                    iny                           ; BUMP IP PAST THE CLOSING PAREN OF THE SUBSCRIPT.
                    pulx
                    bsr       PULNUM              ; GET SUBSCRIPT FROM THE OPERAND STACK.
                    cpd       ,x                  ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       Done@@              ; YES. CONTINUE.
                    lda       #SUBORERR           ; NO. SUBSCRIPT OUT OF RANGE ERROR.
                    bra       Error@@             ; GO REPORT IT.

Done@@              inx:2                         ; BYPASS THE SUBSCRIPT LIMIT.
                    rts

;*******************************************************************************

PULNUM              proc
                    pshx
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    ldd       ,x                  ; GET THE OPERAND.
                    inx:2                         ; BUMP THE STACK POINTER.
                    stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    pulx
                    cpd       #0                  ; "TEST" THE OPERAND BEFORE WE RETURN.
                    rts

;*******************************************************************************
;        ***** chcknfun() *****
;
;        checks for a numeric function and performs it if present

CHKNFUN             proc
                    cmpa      #FUNCTFLG           ; IS THIS A FUNCTION CALL?
                    bne       Done@@              ; NO. JUST RETURN.

                    lda       1,y                 ; GET THE FUNCTION CODE BYTE IN B.
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
Done@@              rts

Table@@             fdb       RFDIV
                    fdb       ICHRS               ; "ICHRS" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    fdb       RADC
                    fdb       RABS
                    fdb       RRND
                    fdb       RSGN
                    fdb       ITAB                ; "ITAB" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    fdb       RCALL
                    fdb       RPEEK
                    fdb       RFEEP               ; "EEP" AS A FUNCTION.
                    fdb       IHEX                ; "IHEX" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    fdb       RFPORTA
                    fdb       RFPORTB
                    fdb       RFPORTC
                    fdb       RFPORTD
                    fdb       RFPORTE
                    fdb       RFTIME
                    fdb       IHEX2               ; "IHEX2" BECAUSE IT'S ILLEGAL IN AN EXPRESSION.
                    fdb       RFPACC

;*******************************************************************************
;        ***** chckee() *****
;
;        if the current token is a semicolin, comma, colin, or space
;        all pending operations on the math stack are performed and
;        we return with the carry set

CHCKEE              proc
                    cmpa      #CPARNTOK           ; IS IT A CLOSED PAREN?
                    beq       Go@@                ; YES.
                    cmpa      #MEOLTOK            ; IS IT ONE OF THE "EXPRESSION END" TOKENS?
                    bhs       Token@@             ; YES.
                    clc                           ; FLAG "NOT AT THE END OF EXPRESSION".
                    rts

Token@@             lda       #CPARNTOK           ; END OF EXPRESSION FOUND. PERFORM ALL PENDING
Go@@                bsr       PSHOP               ; OPERATIONS.
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
                    bne       Cont@@              ; NO. CONTINUE.
                    cmpa      #OPARNTOK           ; YES. IS THE PREVIOUS OPERATOR AN OPEN PAREN?
                    bne       Cont@@              ; NO. CONTINUE.
                    inx:2                         ; YES. KNOCK BOTH OPERATORS OFF THE STACK.
                    stx       OPSTACK             ; SAVE THE STACK POINTER.
Done@@              rts

Cont@@              stb       1,x                 ; PUT IT ON THE STACK.
                    inx                           ; UPDATE THE STACK POINTER.
                    stx       OPSTACK
                    bsr       DOOP                ; GO DO THE OPERATION.
                    bra       Loop@@              ; GO TRY FOR ANOTHER OPERATION.

;*******************************************************************************

DOOP                proc
                    cmpa      #$70                ; IS IT A UINARY OPERATOR?
                    blo       Caret?@@            ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$70                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR7              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

Caret?@@            cmpa      #$60                ; IS IT THE "^" OPERATOR?
                    blo       MulDiv?@@           ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$60                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR6              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

MulDiv?@@           cmpa      #$50                ; IS IT MULTIPLY, DIVIDE, OR MOD?
                    blo       AddSub?@@           ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$50                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR5              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

AddSub?@@           cmpa      #$40                ; IS IT ADD OR SUBTRACT?
                    blo       Logical?@@          ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$40                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR4              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

Logical?@@          cmpa      #$30                ; IS IT A LOGICAL OPERATOR?
                    blo       AndOrXor?@@         ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$30                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR3              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

AndOrXor?@@         cmpa      #$20                ; IS IT AND, OR, OR EOR?
                    blo       Error@@             ; NO. ERROR.
                    suba      #$20                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR2              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       Go@@                ; GO DO THE OPERATION.

Error@@             lda       #ILTOKERR           ; ILLEGAL OPERATOR TOKEN ENCOUNTERED.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                tab                           ; PUT THE OFFSET IN B.
                    aslb                          ; MULTIPLY THE OFFSET BY 2.
                    abx                           ; POINT TO THE ROUTINE ADDRESS.
                    ldx       ,x                  ; GET THE ADDRESS.
                    jmp       ,x                  ; GO DO THE OPERATION & RETURN.

;*******************************************************************************

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
                    std       ,x
                    rts

;*******************************************************************************

RORV                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    ora       ,x
                    orb       1,x
                    std       ,x
                    rts

;*******************************************************************************

RAND                proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    anda      ,x
                    andb      1,x
                    std       ,x
                    rts

;*******************************************************************************

RPLUS               proc
                    jsr       PULNUM
                    ldx       NUMSTACK
                    addd      ,x
                    std       ,x
                    rts

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
                    rts

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
                    bsr       RNEG                ; IF NOT MAKE IT POSITIVE.
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
                    bsr       RNEG                ; MAKE THE QUOTIENT NEGATIVE.
                    ldd       2,x                 ; GET THE REMAINDER.
                    negd                          ; MAKE IT NEGATIVE.
                    std       2,x                 ; SAVE THE RESULT.
Done@@              rts

;*******************************************************************************

RMOD                proc
                    bsr       RDIVS               ; GO GET QUOTIENT & REMAINDER.
                    jmp       PULNUM              ; REMOVE INTEGER RESULT & LEAVE REMAINDER.

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

RINDIR              equ       :AnRTS

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
                    bge       ?RLT1
?RLT2               inc       3,x
?RLT1               inx:2
                    stx       NUMSTACK
                    rts

;*******************************************************************************

RGT                 proc
                    bsr       CMPNUM
                    ble       ?RLT1
                    bra       ?RLT2

;*******************************************************************************

RLTEQ               proc
                    bsr       CMPNUM
                    bgt       ?RLT1
                    bra       ?RLT2

;*******************************************************************************

RGTEQ               proc
                    bsr       CMPNUM
                    blt       ?RLT1
                    bra       ?RLT2

;*******************************************************************************

REQ                 proc
                    bsr       CMPNUM
                    bne       ?RLT1
                    bra       ?RLT2

;*******************************************************************************

RNOTEQ              proc
                    bsr       CMPNUM
                    beq       ?RLT1
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

RPWR                equ       :AnRTS

;*******************************************************************************

RABS                proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    bpl       ?RABS1
?RABS2              negd
?RABS1              std       ,x
                    rts

;*******************************************************************************

RSGN                proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    beq       ?RABS1
                    ldd       #1
                    tst       ,x
                    bpl       ?RABS1
                    bra       ?RABS2

;*******************************************************************************

RCALL               proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    jsr       ,x
                    bra       ?RPEEK

;*******************************************************************************

RPEEK               proc
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    ldb       ,x
                    clra
?RPEEK              ldx       NUMSTACK
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
                    jmp       RPTRERR             ; REPORT THE ERROR.

Go@@                lsld                          ; MULT THE SUBSCRIPT BY 2.
                    addd      #EEPBASAD           ; ADD IN THE BASE ADDRESS OF THE EEPROM ADDRESS.
                    xgdx                          ; PUT THE ADDRESS IN X.
                    ldd       ,x                  ; GET THE DATA.
                    bra       ?RPEEK              ; GO STEAL SOME CODE.

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
                    jmp       RPTRERR             ; GO REPORT IT.

Go@@                xgdx                          ; PUT QUOTIENT IN D.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    inx:2                         ; REMOVE DIVISOR FROM STACK.
                    std       ,x                  ; PUT QUITIENT ON OPERAND STACK.
                    stx       NUMSTACK            ; SAVE NEW VALUE OF STACK POINTER.
                    rts

;*******************************************************************************

RADC                proc
                    jsr       DONEXP              ; GO GET THE CHANNEL NUMBER TO CONVERT.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    ldd       ,x                  ; GET THE CHANNEL NUMBER.
                    bmi       Error@@             ; NEGATIVE CHANNEL NUMBERS ARE ILLEGAL.
                    cpd       #7                  ; IS IT A VALID CHANNEL NUMBER?
                    bls       Go@@                ; YES. GO CONVERT IT.
Error@@             lda       #INVCHERR           ; NO. INVALID CHANNEL NUMBER.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

Go@@                stb       ADCTL               ; START THE CONVERSION ON THE SELECTED.
Ready?@@            tst       ADCTL               ; IS THE CONVERSION COMPLETE?
                    bpl       Ready?@@            ; NO. WAIT FOR 4 CONVERSIONS ON 1 CHANNEL.
                    clra                          ; YES. NOW AVERAGE THE 4 CONVERSIONS.
                    ldb       ADR1                ; GET 1ST RESULT.
                    addb      ADR2                ; ADD IN THE SECOND.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR3                ; ADD IN THE THIRD.
                    adca      #0                  ; ADD IN CARRY.
                    addb      ADR4                ; ADD IN THE FOURTH.
                    adca      #0                  ; ADD IN CARRY.
                    lsrd:2                        ; DIVIDE RESULT BY 4.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    std       ,x                  ; PUT THE RESULT ON THE OPERAND STACK.
                    rts

;*******************************************************************************

RRND                proc
                    jsr       DONEXP              ; GO GET FUNCTION ARGUMENT.
                    ldx       NUMSTACK            ; GET ARGUMENT OFF STACK. GET NEW RANDOM NUMBER?
                    ldd       ,x
                    beq       GetNext@@           ; YES. GO GET NEXT RANDOM NUMBER IN THE SERIES.
                    bmi       New@@               ; IF NEG., START A NEW SERIES.
                    ldd       RANDOM              ; IF POSITIVE, GET LAST RANDOM NUMBER.
                    bra       Done@@              ; RETURN.

New@@               ldd       TCNT                ; USE THE TIMER VALUE AS THE NEW SEED.
                    std       RANDOM              ; SAVE IT.

GetNext@@           ldd       RANDOM              ; GET PREVIOUS RANDOM NUMBER (USE AS SEED).
                    aslb                          ; DO SOME OPERATIONS.
                    aba
                    ldb       RANDOM+1
                    asld
                    asld
                    addd      RANDOM
                    addd      #$3619
                    std       RANDOM

Done@@              lsrd                          ; MAKE THE NUMBER POSITIVE.
                    std       ,x                  ; PUT THE NUMBER ON THE STACK.
                    rts

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
                    bra       ?RFPORTA2           ; GO PUT NUMBER ON THE STACK.

;*******************************************************************************

RFPACC              proc
                    ldx       #REGS
                    ldb       PACNT               ; GET THE CURRENT VALUE OF THE PULSE ACCUMULATOR.
                    clra
                    bra       ?RFPORTA2           ; GO PUT THE NUMBER ON THE STACK.

;*******************************************************************************

RFPORTA             proc
                    ldb       #PORTAIO            ; GET DATA FROM PORTA.
?RFPORTA1           ldx       #REGS
                    abx
                    ldb       ,x
                    clra                          ; CLEAR UPPER BYTE OF WORD.
?RFPORTA2           dey                           ; DECREMENT IP BECAUSE CALLING ROUTINE WILL TRY
                    dey                           ; TO BUMP IT PAST AN OPENING & CLOSING PAREN
                                                  ; WHICH ISN'T THERE.
                    jmp       PushOperand         ; GO PUSH VALUE ON OPERAND STACK & RETURN.

;*******************************************************************************

RFPORTB             proc
                    ldb       #PORTBIO
                    bra       ?RFPORTA1

;*******************************************************************************

RFPORTC             proc
                    ldb       #PORTCIO
                    bra       ?RFPORTA1

;*******************************************************************************

RFPORTD             proc
                    ldb       #PORTDIO
                    bra       ?RFPORTA1

;*******************************************************************************

RFPORTE             proc
                    ldb       #PORTEIO
                    bra       ?RFPORTA1

                    title     IOPKG

;*******************************************************************************

OUTBYTE             proc
                    inc       PRINTPOS            ; INCREMENT THE CURRENT PRINT POSITION.
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #OUTABLE            ; POINT TO THE OUTPUT VECTOR TABLE.
?OUTBYTE            ldb       DEVNUM              ; GET THE CURRENT DEVICE NUMBER.
                    aslb                          ; MULT BY 2.
                    abx                           ; POINT TO THE ADDRESS OF THE OUTPUT ROUTINE.
                    ldx       ,x                  ; GET THE ADDRESS. HAS THE VECTOR BEEN INITALIZED?
                    bne       Done@@              ; YES. GO OUTPUT THE CHARACTER.
                    clr       DEVNUM              ; NO. RESET TO DEVICE #0.
                    lda       #UNINIERR           ; GO REPORT AN UNINITALIZED I/O VECTOR ERROR.
                    jmp       RPTRERR
Done@@              jsr       ,x                  ; GO OUTPUT THE CHARACTER.
                    pulx
                    pulb
                    rts

;*******************************************************************************

INBYTE              proc
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #INTABLE            ; POINT TO THE INPUT VECTOR TABLE.
                    bra       ?OUTBYTE            ; GO USE THE SAME CODE AS OUTBYTE.

;*******************************************************************************

SCIIN               proc
Loop@@              bsr       SCISTAT             ; GET SCI STATUS. HAS A CHARACTER BEEN RECEIVED?
                    beq       Loop@@              ; NO. WAIT FOR CHARACTER TO BE RECEIVED.
                    lda       SCDR                ; GET THE CHARACTER.
                    rts

;*******************************************************************************

SCIOUT              proc
                    psha                          ; SAVE THE CHARACTER TO SEND.
Loop@@              lda       SCSR                ; GET THE SCI STATUS.
                    bita      #$80                ; HAS THE LAST CHARACTER BEEN SHIFTED OUT?
                    beq       Loop@@              ; NO. WAIT TILL IT HAS.
                    pula                          ; RESTORE CHARACTER TO SEND.
                    sta       SCDR                ; SEND THE CHARACTER.
                    rts

;*******************************************************************************

SCISTAT             proc
                    psha                          ; SAVE THE A-REG.
                    lda       SCSR                ; GET THE SCI STATUS.
                    bita      #$20                ; CHECK TO SEE IF A CHARACTER HAS BEEN RECEIVED.
                    pula                          ; RESTORE STATUS.
                    rts                           ; RETURN W/ STATUS.

;*******************************************************************************

IODevInit           proc
                    bsr       InitSCI
                    lda       #JMPOP
                    sta       CONSTAT             ; INITIALIZE THE CONSOLE STATUS VECTOR.
                    sta       INCONNE             ; INITIALIZE THE INPUT FROM CONSOLE NO ECHO VECT.
                    rts

;*******************************************************************************

InitSCI             proc
                    lda       #$30                ; SET BAUD RATE TO 9600.
                    sta       BAUD
                    clr       SCCR1               ; SET FOR 8 BIT OPERATION, DISABLE WAKEUP.
                    lda       #$0C                ; ENABLE THE TRANSMITER & RECEIVER.
                    sta       SCCR2
                    lda       #$11                ; GET THE XON CHARACTER (CONTROL-Q).
                    sta       XONCH               ; INITALIZE THE XON REGISTER.
                    lda       #$13                ; GET THE XOFF CHARACTER (CONTROL-S).
                    sta       XOFFCH              ; INITALIZE THE XOFF CHARACTER.
                    rts

;*******************************************************************************

PROUT               proc                          ; SEND A CHARACTER TO THE PRINTER.
                    bsr       SCISTAT             ; WAS AN "X-OFF" RECEIVED?
                    beq       Send@@              ; NO. GO SEND THE CHARACTER.
                    psha                          ; SAVE THE CHARACTER TO SEND.
                    bsr       SCIIN               ; YES. GO RESET THE SCI RECEIVER STATUS.
                    cmpa      XOFFCH              ; WAS IT AN XOFF?
                    bne       Cont@@              ; NO. SO GO SEND THE CHARACTER.
Loop@@              bsr       SCIIN               ; GO WAIT FOR AN "X-ON" CHARACTER.
                    cmpa      XONCH               ; IS IT AN X-ON CHARACTER?
                    bne       Loop@@              ; NO. GO WAIT FOR AN X-ON CHARACTER.
Cont@@              pula                          ; GET THE CHARACTER TO SEND.
Send@@              bra       SCIOUT              ; SEND THE CHARACTER TO THE PRINTER & RETURN.

;*******************************************************************************
                    title     Config/Reset/Interrupt Vectors
;*******************************************************************************

IOVects             dw        0                   ; Inputs
                    dw        SCIIN
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0
          ;--------------------------------------
                    dw        0                   ; Outputs
                    dw        PROUT
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0
                    dw        0

;===============================================================================

RAMStart            dw        $c000               ; starting address of system RAM.
RAMSize             dw        $2000               ; size of BASIC11 RAM Buffer.
EEStart             dw        $6000               ; starting address of program storage EEPROM
EESize              dw        $2000               ; size of the program storage EEPROM
TimeVal             dw        62500               ; value used for generating 'Time' Interrupt
UserInit            dw        IODevInit           ; Used to initialize console/other hardware.
DFLOPADR            dw        $4000               ; Address of flip-flop used to connect the HC11 SCI
                                                  ; to the host port connector.

;*******************************************************************************

                    @vector   Vsci,SCISS          ; SCI SERIAL SYSTEM
                    @vector   Vspi,SPITC          ; SPI TRANSFER COMPLETE
                    @vector   Vpai,PACCIE         ; PULSE ACCUMULATOR INPUT EDGE
                    @vector   Vpao,PACCOVF        ; PULSE ACCUMULATOR OVERFLOW
                    @vector   Vrto,TIMEROVF       ; TIMER OVERFLOW
                    @vector   Vtoc5,vTOC5         ; TIMER OUTPUT COMPARE 5
                    @vector   Vtoc4,vTOC4         ; TIMER OUTPUT COMPARE 4
                    @vector   Vtoc3,vTOC3         ; TIMER OUTPUT COMPARE 3
                    @vector   Vtoc2,vTOC2         ; TIMER OUTPUT COMPARE 2
                    @vector   Vtoc1,vTOC1         ; TIMER OUTPUT COMPARE 1
                    @vector   Vtic3,vTIC3         ; TIMER INPUT CAPTURE 3
                    @vector   Vtic2,vTIC2         ; TIMER INPUT CAPTURE 2
                    @vector   Vtic1,vTIC1         ; TIMER INPUT CAPTURE 1
                    @vector   Vrti,REALTIMI       ; REAL TIME INTERRUPT
                    @vector   Virq,IRQI           ; IRQ INTERRUPT
                    @vector   Vxirq,XIRQ          ; XIRQ INTERRUPT
                    @vector   Vswi,SWII           ; SOFTWARE INTERRUPT
                    @vector   Villop,ILLOP        ; ILLEGAL OPCODE TRAP
                    @vector   Vcop,COP            ; WATCH DOG FAIL
                    @vector   Vcmf,CMF            ; CLOCK MONITOR FAIL
                    @vector   Vreset,POWERUP      ; RESET

                    end       :s19crc

;*******************************************************************************
                    #Message  +---------------------------------------------------
                    #Message  | Verification    7990 bytes, RAM:   206, CRC: $375B
                    #Message  +---------------------------------------------------
;*******************************************************************************
