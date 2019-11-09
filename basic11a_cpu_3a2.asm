;*******************************************************************************
;                         BASIC11 FOR CPU_3A2                                  *
;                                                                              *
;               MODIFIED BY PAUL BEALING, APRIL-JUNE 2000                      *
;*******************************************************************************
;                                                                              *
;                      MC68HC11 BASIC INTERPRETER                              *
;                                                                              *
;                      WRITTEN BY GORDON DOUGHMAN                              *
;                                                                              *
;                COPYRIGHT 1985-1990 BY GORDON DOUGHMAN                        *
;                                                                              *
;*******************************************************************************
;
; 17-june-2000
;       Ready for first release for the CPU_1A1/1B board.
; 02-may-2003
;       Modified for the CPU_3A2 board.
;
;*******************************************************************************

;_DEBUG                                           ; include if testing in RAM
                                                  ; exclude for Flash version
_EEPROM                                           ; include for Flash version
                                                  ; exclude if testing in RAM

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "DEFINES.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

;************************************************************
;       SYSTEM DEFINES
;************************************************************

; These defines specify how BASIC11 is loaded into the CPU_3A2 module
; In debug mode it is loaded into and run from RAM. This allows for
;  easy testing without erasing and loading the Flash
; When running from RAM, the vectors must already be loaded into EEPROM.

; Normally it will be loaded into Flash.

ESC                 equ       27

          #ifdef _DEBUG
ROMBEG              equ       $5000               ; starting address of code space 5000-7FFF
ROMSIZE             equ       $3000               ; 12K of code space
RAMSTART            equ       $2000               ; starting address of system RAM 2000-2FFF
RAMSIZE             equ       $1000               ; 4K of system RAM
PSSTART             equ       $8000               ; first add. of program storage FLASH 3000 or 8000
PSSIZE              equ       $1000               ; 4K of program storage FLASH
EESTART             equ       $FE00               ; starting address of CPU EEPROM FE00-FF7F (384B)
EESIZE              equ       $017F               ; size of CPU EEPROM
EEPBASAD            equ       $FE00               ; /* EEPROM base address */
MAXEESUB            equ       384                 ; /* maximum EEP subscript */
          #else
ROMBEG              equ       $C000               ; starting address of code space C000-AFFF
ROMSIZE             equ       $3DFF               ; ~16K of code space
RAMSTART            equ       $2000               ; starting address of system RAM 2000-5FFF
RAMSIZE             equ       $4000               ; approx. 16K of system RAM
PSSTART             equ       $8000               ; first add. of program storage FLASH 8000-BFFF
PSSIZE              equ       $4000               ; approx. 12K of program storage FLASH
EESTART             equ       $FE00               ; starting address of CPU EEPROM FE00-FF7F (384B)
EESIZE              equ       $017F               ; size of CPU EEPROM
EEPBASAD            equ       $FE00               ; /* EEPROM base address */
MAXEESUB            equ       384                 ; /* maximum EEP subscript */
          #endif
RAM_SA              equ       $0000               ; start of RAM
RAM_EA              equ       $7FFF               ; end of RAM
FLS_SA              equ       $8000               ; start of Flash
FLS_EA              equ       $FDFF               ; end of Flash
EEP_SA              equ       $FE00               ; start of EEPROM
EEP_EA              equ       $FFFF               ; end of EEPROM

TIMEVAL             equ       62500               ; value used for generating 'Time' Interrupt
IOBASE              equ       $1000               ; Base Address of the I/O Registers
SWPRE               equ       02                  ; SOFTWARE PRESCALER VALUE.

SBASBEG             equ       0
SBASEND             equ       2
SVARBEG             equ       4
SVAREND             equ       6
SHILINE             equ       8
AUTOSTF             equ       10
SSTART              equ       11

;************************************************************
;    I/O Register Offsets From The Base Address
;************************************************************

PORTAIO             equ       $00                 ; PORT A
DDRA                equ       $01                 ; PORT A DDR
PORTGIO             equ       $02                 ; PORT G
DDRG                equ       $03                 ; PORT G DDR
PORTBIO             equ       $04                 ; PORT B
PORTCIO             equ       $05                 ; PROT C LATCHED DATA REG.
DDRC                equ       $07                 ; DATA DIRECTION REGISTER C
PORTDIO             equ       $08                 ; PORT D
DDRD                equ       $09                 ; DATA DIRECTION REGISTER D
PORTEIO             equ       $0A                 ; PORT E
CFORC               equ       $0B                 ; TIMER COMPARE FORCE REG.
OC1M                equ       $0C                 ; O/P COMPARE 1 MASK REG.
OC1D                equ       $0D                 ; O/P COMPARE 1 DATA REG
TCNT                equ       $0E                 ; TIMER COUNT H
TCNTL               equ       $0F                 ; L
TIC1                equ       $10                 ; TIMER O/P COMPARE 1 H
TIC1L               equ       $11                 ; 1 L
TIC2                equ       $12                 ; 2 H
TIC2L               equ       $13                 ; 2 L
TIC3                equ       $14                 ; 3 H
TIC3L               equ       $15                 ; 3 L
TOC1                equ       $16                 ; TIMER O/P COMPARE 1 H
TOC1L               equ       $17                 ; 1 L
TOC2                equ       $18                 ; 2 H
TOC2L               equ       $19                 ; 2 L
TOC3                equ       $1A                 ; 3 H
TOC3L               equ       $1B                 ; 3 L
TOC4                equ       $1C                 ; 4 H
TOC4L               equ       $1D                 ; 4 L
TOC5                equ       $1E                 ; 5 H
TOC5L               equ       $1F                 ; 5 L
TCTL1               equ       $20                 ; TIMER CONTROL REG 1
TCTL2               equ       $21                 ; TIMER CONTROL REG 2
TMSK1               equ       $22                 ; TIMER MASK 1
TFLAG1              equ       $23                 ; TIMER FLAG 1
TMSK2               equ       $24                 ; TIMER MASK 2
TFLG2               equ       $25                 ; TIMER FLAG 2
PACTL               equ       $26                 ; PULSE ACCUMULATOR CONT. REG.
PACNT               equ       $27                 ; PULSE ACCUMULATOR COUNT REG.
SPCR                equ       $28
SPSR                equ       $29
SPDR                equ       $2A
BAUD                equ       $2B                 ; SCI BAUD REG
SCCR1               equ       $2C                 ; SCI CONTROL 1 REG
SCCR2               equ       $2D                 ; SCI CONTROL 2 REG
SCSR                equ       $2E                 ; SCI STATUS REG
SCDR                equ       $2F                 ; SCI DATA REG
ADCTL               equ       $30
ADR1                equ       $31
ADR2                equ       $32
ADR3                equ       $33
ADR4                equ       $34
BPROT               equ       $35
OPTION              equ       $39                 ; OPTION REG
COPRST              equ       $3A                 ; COP RESET REG
PPROG               equ       $3B                 ; EEPROM PROG REG
HPRIO               equ       $3C                 ; HPRIO REG
INITSU              equ       $103D               ; INIT AT RESET
INIT                equ       $3D                 ; INIT
CONFIG              equ       $3F                 ; CONFIG REG
CSSTRH              equ       $5C                 ; CS CYCLE STRETCH
CSCTL               equ       $5D                 ; CS CONTROL
CSGADR              equ       $5E                 ; GENERAL PURPOSE CS (RAM)
CSGSIZ              equ       $5F                 ; GENERAL PURPOSE CS SIZE
BANKSR              equ       $1600               ; BANK SELECT REGISTER

;************************************************************
;       misc. defines

EOL                 equ       13                  ; /* end of line marker */
CR                  equ       13                  ; /* same as EOL */
LF                  equ       10                  ; /* linefeed character */
BS                  equ       8                   ; /* backspace character */
SPC                 equ       32                  ; /* space character */
MIDEOL              equ       ':'                 ; /* mid EOL character */
COMMA               equ       ','                 ; /* comma */
SEMI                equ       59                  ; ';' /* semicolin */
NUM                 equ       1                   ; /* getvar return flag */
STRING              equ       2                   ; /* getvar return flag */
NULL                equ       0                   ; /* null value */
CNTRLC              equ       3                   ; /* control-c (break character) */

IBUFLEN             equ       80                  ; /* input buffer max length */
TBUFLEN             equ       128                 ; /* token buffer max length */
SWSTKSize           equ       592                 ; /* software stack size /*

OPSLEN              equ       30                  ; /* operator stack length */
NUMSLEN             equ       60                  ; /* operand stack length */
FORSLEN             equ       80                  ; /* FOR..NEXT stack length */
WHSLEN              equ       16                  ; /* WHILE..ENDWH stack length */
GOSLEN              equ       16                  ; /* GOSUB stack length */

LED1                equ       %00000100           ; led output
LED2                equ       %00001000           ; led output
LED3                equ       %00010000           ; led output
LED4                equ       %00100000           ; led output
LED5                equ       %10000000           ; led output on port A
SPIMS               equ       %10000000           ; SPI master/slave select
A15                 equ       %00000010           ; flash upper address bit
A16                 equ       %00000001           ; flash upper address bit
A17                 equ       %00010000           ; flash upper address bit
A18                 equ       %00001000           ; flash upper address bit

;************************************************************
;       define error codes

LINRANG             equ       1                   ; /* line number range error */
SYTXERR             equ       2                   ; /* syntax error */
IVEXPERR            equ       3                   ; /* invalid expression error */
UPARNERR            equ       4                   ; /* unbalanced parentheses error */
DTMISERR            equ       5                   ; /* data type mismatch error */
OPRTRERR            equ       6                   ; /* illegal operator error */
ILVARERR            equ       7                   ; /* illegal variable error */
ILTOKERR            equ       8                   ; /* illegal token error */
OMEMERR             equ       9                   ; /* out of memory error */
INTOVERR            equ       10                  ; /* integer overflow error */
IVHEXERR            equ       11                  ; /* invalid hex digit error */
HEXOVERR            equ       12                  ; /* hex number overflow */
MISQUERR            equ       13                  ; /* missing quote error */
MPARNERR            equ       14                  ; /* missing open or closing parenthisis */
IONSYERR            equ       15                  ; /* "ON" syntax error */
MTHENERR            equ       16                  ; /* missing "THEN" in "IF" statement */
MTOERR              equ       17                  ; /* missing "TO" in "FOR" statement */
LINENERR            equ       18                  ; /* line number error */
IDTYERR             equ       19                  ; /* illegal data type error */
EXPCXERR            equ       20                  ; /* expression too complex (xlator token buff ovf.) */
MCOMAERR            equ       21                  ; /* missing comma */
MCMSMERR            equ       22                  ; /* missing comma or semicolin */
MSTKOERR            equ       23                  ; /* math stack overflow error */
UNDIMERR            equ       24                  ; /* undimentioned array error */
SUBORERR            equ       25                  ; /* subscript out of range error */
ZDIVERR             equ       26                  ; /* divide by zero error */
LNFERR              equ       27                  ; /* line not found error */
GOSOVERR            equ       28                  ; /* too many nested GOSUB's */
RWOGERR             equ       29                  ; /* RETURN w/o GOSUB error */
WHSOVERR            equ       30                  ; /* too many active WHILE's */
ENDWHERR            equ       31                  ; /* ENDWH statement w/o WHILE */
ONARGERR            equ       32                  ; /* ON argument is negative, zero, or too large */
NOSUBERR            equ       33                  ; /* non-subscriptable variable found in DIM statem. */
REDIMERR            equ       34                  ; /* variable has already been DIMensioned */
FORNXERR            equ       35                  ; /* too many active FOR -- NEXT loops active */
MFRNXERR            equ       36                  ; /* mismatched FOR -- NEXT statements. */
CNTCNERR            equ       37                  ; /* can't continue */
ODRDERR             equ       38                  ; /* out of data in read or restore statement */
NEGSUBER            equ       39                  ; /* negative subscripts not allowed */
EESUBERR            equ       40                  ; /* EEP() subscript negative or > 200 */
PRFUNERR            equ       41                  ; /* function only allowed in print statement */
TABARGER            equ       42                  ; /* argument <0 or >255 in TAB() function */
CHRARGER            equ       43                  ; /* argument <0 or >255 in CHR$() function */
OVDV0ERR            equ       44                  ; /* overflow or /0 error in FDIV() function */
INVCHERR            equ       45                  ; /* invalid channel number in ADC() function */
PRTASERR            equ       46                  ; /* tried to assign a value <0 or >255 to PORT(X) */
ILPRTERR            equ       47                  ; /* illegal port error */
ILLIOERR            equ       48                  ; /* illegal I/O vector number <0 or >7 */
UNINIERR            equ       49                  ; /* uninitalized I/O vector */
HEX2AERR            equ       50                  ; /* argument <0 or >255 in HEX2 function */
NOTALERR            equ       51                  ; /* statement not allowed in direct mode */
NOTINTER            equ       52                  ; /* an RETI statement executed when not in interrupt */
PACCARGE            equ       53                  ; /* tried to assign a value of <0 or >255 to PACC */
INTMODER            equ       54                  ; /* interrupt or count mode error in ONPACC */
EETOSMAL            equ       55                  ; /* program storage EEPROM is Too Small */
FLERERR             equ       56                  ; /* Flash memory erase failure */
NOFLPROG            equ       57                  ; /* No Flash memory program to load */
EEWRERR             equ       58                  ; /* eeprom write failure

;************************************************************
;       mathematical operator tokens

OPARNTOK            equ       $10                 ; /* '(' */
CPARNTOK            equ       $11                 ; /* ')' */
ANDTOK              equ       $20                 ; /* 'AND' */
ORTOK               equ       $21                 ; /* 'OR' */
EORTOK              equ       $22                 ; /* 'EOR' */
LTTOK               equ       $30                 ; /* '<' */
GTTOK               equ       $31                 ; /* '> */
LTEQTOK             equ       $32                 ; /* '<=' */
GTEQTOK             equ       $33                 ; /* '>=' */
EQTOK               equ       $34                 ; /* '=' */
NOTEQTOK            equ       $35                 ; /* '<>' */
PLUSTOK             equ       $40                 ; /* '+' */
MINUSTOK            equ       $41                 ; /* '-' */
SPLUSTOK            equ       $42                 ; /* '+' */
MULTTOK             equ       $50                 ; /* '*' */
DIVTOK              equ       $51                 ; /* '/' */
MODTOK              equ       $52                 ; /* '%' */
PWRTOK              equ       $60                 ; /* '^' */
INDIRTOK            equ       $70                 ; /* '@' */
NOTTOK              equ       $71                 ; /* 'NOT' */
NEGTOK              equ       $72                 ; /* '-' (uniary minus) */

;************************************************************
;       keyword tokens

LETTOK              equ       $01                 ; /* LET */
IMLETTOK            equ       $02                 ; /* implied LET */
PRINTTOK            equ       $03                 ; /* PRINT */
FORTOK              equ       $04                 ; /* FOR */
NEXTTOK             equ       $05                 ; /* NEXT */
TRONTOK             equ       $06                 ; /* TRON */
TROFFTOK            equ       $07                 ; /* TROFF */
POKETOK             equ       $08                 ; /* POKE */
DIMTOK              equ       $09                 ; /* DIM */
REMTOK              equ       $0A                 ; /* REM */
PACCTOK             equ       $0B                 ; /* PACC */
DATATOK             equ       $0C                 ; /* DATA */
READTOK             equ       $0D                 ; /* READ */
RESTRTOK            equ       $0E                 ; /* RESTORE */
GOSUBTOK            equ       $0F                 ; /* GOSUB */
GOTOTOK             equ       $12                 ; /* GOTO */
ONTOK               equ       $13                 ; /* ON */
RETNTOK             equ       $14                 ; /* RETURN */
IFTOK               equ       $15                 ; /* IF */
INPUTTOK            equ       $16                 ; /* INPUT */
STOPTOK             equ       $17                 ; /* STOP */
ENDTOK              equ       $18                 ; /* END */
WHILETOK            equ       $19                 ; /* WHILE */
ENDWHTOK            equ       $1A                 ; /* ENDWH */
EEPTOK              equ       $1B                 ; /* EEP */
PORTATOK            equ       $1C                 ; /* PORTA */
PORTBTOK            equ       $1D                 ; /* PORTB */
PORTCTOK            equ       $1E                 ; /* PORTC */
PORTDTOK            equ       $1F                 ; /* PORTD */
INBYTTOK            equ       $23                 ; /* INBYTE */
TIMETOK             equ       $24                 ; /* TIME */
ONTIMTOK            equ       $25                 ; /* ONTIME */
ONIRQTOK            equ       $26                 ; /* ONIRQ */
RETITOK             equ       $27                 ; /* RETI */
ONPACTOK            equ       $28                 ; /* ONPACC */
SLEEPTOK            equ       $29                 ; /* SLEEP */
RTIMETOK            equ       $2A                 ; /* RTIME */
CLSTOK              equ       $2B                 ; /* CLS */
FUNCTFLG            equ       $36                 ; /* function flag byte */
TOTOK               equ       $37                 ; /* TO */
THENTOK             equ       $38                 ; /* THEN */
ELSETOK             equ       $39                 ; /* ELSE */
STEPTOK             equ       $3A                 ; /* STEP */

;************************************************************
;       function tokens

FDIVTOK             equ       $01                 ; /* FDIV */
CHRTOK              equ       $02                 ; /* CHR$ */
ADCTOK              equ       $03                 ; /* ADC */
ABSTOK              equ       $04                 ; /* ABS */
RNDTOK              equ       $05                 ; /* RND */
SGNTOK              equ       $06                 ; /* SGN */
TABTOK              equ       $07                 ; /* TAB */
CALLTOK             equ       $08                 ; /* CALL */
PEEKTOK             equ       $09                 ; /* PEEK */
FEEPTOK             equ       $0A                 ; /* EEP */
HEXTOK              equ       $0B                 ; /* HEX */
FPRTATOK            equ       $0C                 ; /* PORTA */
FPRTBTOK            equ       $0D                 ; /* PORTB */
FPRTCTOK            equ       $0E                 ; /* PORTC */
FPRTDTOK            equ       $0F                 ; /* PORTD */
FPRTETOK            equ       $10                 ; /* PORTE */
FTIMETOK            equ       $11                 ; /* TIME */
HEX2TOK             equ       $12                 ; /* HEX2 */
FPACCTOK            equ       $13                 ; /* PACC */

;************************************************************
;       numerical/variable tokens

FVARTOK             equ       $81                 ; /* floating point variable address */
SVARTOK             equ       $82                 ; /* string variable address */
IVARTOK             equ       $84                 ; /* integer variable address */

FAVARTOK            equ       $91                 ; /* floating point array */
SAVARTOK            equ       $92                 ; /* string array */
IAVARTOK            equ       $94                 ; /* integer array */

FCONTOK             equ       $A1                 ; /* floating point constant */
SCONTOK             equ       $A2                 ; /* string constant */
LCONTOK             equ       $A8                 ; /* line # constant */
ICONTOK             equ       $A4                 ; /* integer constant */

ISIZ                equ       2                   ; /* number of bytes in integer variable */
SSIZ                equ       3                   ; /* number of bytes in string variable */
FSIZ                equ       5                   ; /* number of bytes in f.p. variable */
ASIZ                equ       2                   ; /* number of bytes for array variable in dictionary */

;************************************************************
;       misc. tokens

MSCNTOK             equ       $7F                 ; /* multiple space count token */
SSCNTOK             equ       $7E                 ; /* single space token */
EOLTOK              equ       $7D                 ; /* end of line token */
COMMATOK            equ       $7C                 ; /* , */
SEMITOK             equ       $7B                 ; /* ; */
MEOLTOK             equ       $7A                 ; /* : */
EQUALTOK            equ       $79                 ; /* '=' */
PNUMTOK             equ       $78                 ; /* '#' */

JMPOP               equ       $7E                 ; OP-CODE FOR "JMP" (USED TO INITALIZE INTERRUPT TABLE)

;************************************************************
;************************************************************
;       define variables

                    org       $0000

;       char

IBUFPTR             rmb       2                   ; /* input buffer pointer */
TBUFPTR             rmb       2                   ; /* token buffer pointer */

;       the next 5 variables must remain grouped togeather

BASBEG              rmb       2                   ; /* start of basic program area */
BASEND              rmb       2                   ; /* end of basic program */
VARBEGIN            rmb       2                   ; /* start of variable storage area */
VAREND              rmb       2                   ; /* end of variable storage area */
HILINE              rmb       2                   ; /* highest line number in program buffer */

BASMEND             rmb       2                   ; /* physical end of basic program memory */
VARMEND             rmb       2                   ; /* physical end of variable memory */

;       int

FIRSTLIN            rmb       2                   ; /* first line to list */
LASTLIN             rmb       2                   ; /* last line to list */
INTPTR              rmb       2                   ; /* integer pointer */

;       short

ERRCODE             rmb       1                   ; /* error code status byte */
IMMID               rmb       1                   ; /* immidiate mode flag */
BREAKCNT            equ       *                   ; /* also use for break check count */
COUNT               equ       *                   ; /* count used in ESAVE & ELOAD routines */
IFWHFLAG            rmb       1                   ; /* translating IF flag */
TRFLAG              rmb       1                   ; /* trace mode flag */
CONTFLAG            rmb       1                   ; /* continue flag */
RUNFLAG             rmb       1                   ; /* indicates we are in the run mode */
PRINTPOS            rmb       1                   ; /* current print position */
NUMSTACK            rmb       2                   ; /* numeric operand stack pointer */
OPSTACK             rmb       2                   ; /* operator stack pointer */
FORSTACK            rmb       2                   ; /* FOR stack pointer */
WHSTACK             rmb       2                   ; /* WHILE stack pointer */
GOSTACK             rmb       2                   ; /* GOSUB stack pointer */
CURLINE             rmb       2                   ; /* line # that we are currently interpreting */
ADRNXLIN            rmb       2                   ; /* address of the next line */
STRASTG             rmb       2                   ; /* dynamic string/array pool pointer */
FENCE               rmb       2                   ; /* varend fence in case of an error in xlation */
IPSAVE              rmb       2                   ; /* interpretive pointer save for "BREAK" */
DATAPTR             rmb       2                   ; /* pointer to data for read statement */
RANDOM              rmb       2                   ; /* random number/seed */
DEVNUM              rmb       1                   ; /* I/O device number */
TIMEREG             rmb       2                   ; /* TIME register */
TIMECMP             rmb       2                   ; /* TIME compare register */
TIMEPRE             rmb       1                   ; /* software prescaler for TIME */
ONTIMLIN            rmb       2                   ; /* ONTIME line number to goto */
ONIRQLIN            rmb       2                   ; /* ONIRQ line number to goto */
ONPACLIN            rmb       2                   ; /* ONPACC line number to goto */
XONCH               rmb       1                   ; /* XON character for printer */
XOFFCH              rmb       1                   ; /* XOFF character for printer */
SCURLINE            rmb       2                   ; /* used to save CURLINE during int. processing */
SADRNXLN            rmb       2                   ; /* used to save ADRNXLIN during int. processing */
INBUFFS             rmb       2                   ; /* pointer to the start of the input buffer */
TKNBUFS             rmb       2                   ; /* pointer to the start of the token buffer */

EOPSTK              rmb       2                   ; /* end of operator stack */
STOPS               rmb       2                   ; /* start of operator stack */
ENUMSTK             rmb       2                   ; /* end of operand stack */
STNUMS              rmb       2                   ; /* start of operand stack */
EFORSTK             rmb       2                   ; /* end of FOR - NEXT stack */
STFORSTK            rmb       2                   ; /* start of FOR - NEXT stack */
EWHSTK              rmb       2                   ; /* end of WHILE stack */
STWHSTK             rmb       2                   ; /* start of WHILE stack */
EGOSTK              rmb       2                   ; /* end of GOSUB stack */
STGOSTK             rmb       2                   ; /* start of GOSUB stack */
IOBASEV             rmb       2                   ; /* Address vector for I/O Registers */
DNAME               rmb       3                   ; /* Place to put the variable name when doing a dump command */
SUBMAX              rmb       2                   ; /* */
SUBCNT              rmb       2                   ; /* */
TOKPTR              rmb       2                   ; /* token pointer (used for list command) */
VarSize             rmb       2                   ; /* used by the line editor. size of the variable table */


TADD                rmb       1                   ; Talkaddress
ADD                 rmb       1                   ; Address
RLEN                rmb       1                   ; Record Length
CSUM                rmb       1                   ; Checksum
BADDR               rmb       1                   ; Board Address
CCOM                rmb       1                   ; Current Command
COUNT1              rmb       1                   ; Counter for dump routine
TEMP1               rmb       2                   ; 2 ; Temp for dump routine
DMPGO               rmb       2                   ; 2 ; dump start address
DMPEND              rmb       2                   ; 2 ; dump end address
GWIZ1               rmb       1                   ; unexpected interrupt ID code
FLAGS1              rmb       1                   ; general purpose flags
BANKSS              rmb       1                   ; bank select register main copy
BANKST              rmb       1                   ; bank select register temp copy

HERE1               equ       *

; if  *>$9E
; fail     "Ran out of Page 0 RAM"
; endif

                    org       $009E

CONSTAT             rmb       3                   ; GET CONSOLE STATUS FOR BREAK ROUTINE.
INCONNE             rmb       3                   ; GET BYTE DIRECTLY FROM CONSOLE FOR BREAK ROUTINE.

                    org       $00A4

INTABLE             rmb       16                  ; RESERVE SPACE FOR 8 DIFFERENT INPUT ROUTINES.
OUTABLE             rmb       16                  ; RESERVE SPACE FOR 8 DIFFERENT OUTPUT ROUTINES.

;************

                    org       $00C4               ; START OF RAM INTERRUPT VECTORS.

RAMVECTS            equ       *
SCISS               rmb       3                   ; 00C4 ; SCI SERIAL SYSTEM.
SPITC               rmb       3                   ; 00C7 ; SPI TRANSFER COMPLETE.
PACCIE              rmb       3                   ; 00CA ; PULSE ACCUMULATOR INPUT EDGE.
PACCOVF             rmb       3                   ; 00CD ; PULSE ACCUMULATOR OVERFLOW.
TIMEROVF            rmb       3                   ; 00D0 ; TIMER OVERFLOW.
TIMOC5              rmb       3                   ; 00D3 ; TIMER OUTPUT COMPARE 5.
TIMOC4              rmb       3                   ; 00D6 ; TIMER OUTPUT COMPARE 4.
TIMOC3              rmb       3                   ; 00D9 ; TIMER OUTPUT COMPARE 3.
TIMOC2              rmb       3                   ; 00DC ; TIMER OUTPUT COMPARE 2.
TIMOC1              rmb       3                   ; 00DF ; TIMER OUTPUT COMPARE 1.
TIMIC3              rmb       3                   ; 00E2 ; TIMER INPUT CAPTURE 3.
TIMIC2              rmb       3                   ; 00E5 ; TIMER INPUT CAPTURE 2.
TIMIC1              rmb       3                   ; 00E8 ; TIMER INPUT CAPTURE 1.
REALTIMI            rmb       3                   ; 00EB ; REAL TIME INTERRUPT.
IRQI                rmb       3                   ; 00EE ; IRQ INTERRUPT.
XIRQ                rmb       3                   ; 00F1 ; XIRQ INTERRUPT.
SWII                rmb       3                   ; 00F4 ; SOFTWARE INTERRUPT.
ILLOP               rmb       3                   ; 00F7 ; ILLEGAL OPCODE TRAP.
COP                 rmb       3                   ; 00FA ; WATCH DOG TIMER FAIL.
CMF                 rmb       3                   ; 00FD ; CLOCK MONITOR FAIL.

;************

                    org       $FF7F               ; EEPROM variable storage

ASFLAG              rmb       1                   ; auto-start flag

;************************************************************************
;************************************************************************
;************************************************************************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************; P R O G R A M   S T A R T S   H E R E ********************
;************************************************************************
;************************************************************************
;************************************************************************

;******************************************************************************
;******************************************************************************
;       SYSTEM INITALISATION
;******************************************************************************
;******************************************************************************

                    org       ROMBEG

POWERUP             equ       *

                    ldd       #IOBASE             ; register base address (default, $1000)
                    std       IOBASEV             ; "
                    lsra                          ; adjust for INIT
                    lsra                          ; "
                    lsra                          ; "
                    lsra                          ; "
                    sta       INITSU              ; remap the CPU registers
                    lds       #$0FFF              ; load stack pointer

                    ldx       IOBASEV             ; get reg offset pointer
                    lda       #%10010011          ; adpu, irqe, dly, cop = longest
                    sta       OPTION,x
                    lda       #%00000000
                    sta       CSSTRH,x            ; CS cycle stretch
                    lda       #%10000101
                    sta       CSCTL,x             ; enable program CS for 32K and bank
                    lda       #%00000000
                    sta       CSGADR,x            ; RAM starts at address 0000H
                    lda       #%10000001
                    sta       CSGSIZ,x            ; RAM block size is 32K

                    lda       #%11111111          ; enable SRAM (CE2) and select banks
                    sta       BANKSR              ; select banks (Flash and RAM)
                    sta       BANKSS              ; make working copy

                    lda       #$03                ; set timer prescale to "/16"
                    sta       TMSK2,x             ; save

; clear RAM
; clear 0100h to 0FFFh
                    ldx       #$0100              ; first address
RAM1                clr       ,x                  ; clear it
                    inx                           ; next
                    cpx       #$1000              ; last address +1
                    bne       RAM1                ; loop

; load interrupt vector table

                    ldx       #INTVECT_SA         ; source, INT VECTORS IN ROM
                    ldy       #RAMVECTS           ; target, START OF RAM TABLE
ST_20               lda       #$7E                ; JMP OPCODE
                    sta       ,y                  ; store jmp opcode
                    iny                           ; NEXT target ADDRESS
                    ldd       ,x                  ; GET VECTOR
                    std       ,y                  ; store vector
                    inx                           ; NEXT DESTINATION ADDRESS
                    inx                           ; "
                    iny                           ; NEXT SOURCE ADDRESS
                    iny                           ; "
                    cpx       #INTVECT_EA+2       ; finished ?
                    blo       ST_20               ; no, loop

;************
; Loads INTABLE and OUTABLE with the addresses of the input and output routines.
; DEVNUM specifies which routine to use.
;  There are only 2 at this tme, SCIIN and PROUT (DEVNUM =1).

                    ldy       #IOVects            ; source, point to the default table in ROM.
                    ldx       #INTABLE            ; destination, START OF THE I/O VECTOR TABLE.
                    ldb       #32                 ; byte count, GET NUMBER OF BYTES IN THE TABLE.
PWRUP2              lda       ,y                  ; Move a byte of the table from ROM into RAM.
                    sta       ,x
                    inx                           ; POINT TO THE NEXT BYTE.
                    iny
                    decb                          ; DECREMENT THE COUNT.
                    bne       PWRUP2              ; GO TILL WE'RE DONE.
                    ldx       IOBASEV             ; get reg offset pointer

;************
; misc

                    jsr       FL_RAM              ; check/copy to RAM (for Flash)

                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    clra
                    clrb
                    std       TIMEREG             ; ZERO THE TIME REGISTER.
                    std       TIMECMP             ; zero the time compare register.

                    ldx       IOBASEV
                    jsr       TIMINTS             ; GO SETUP THE TIMER FOR THE FIRST INTERRUPT.
                    lda       #$80                ; ENABLE INTERRUPTS FROM OC1.
                    sta       TMSK1,x

                    clra
                    clrb
                    std       ONTIMLIN            ; INITALIZE THE LINE POINTERS.
                    std       ONIRQLIN
                    std       ONPACLIN

                    ldx       #IODevInit          ; USERINIT
                    jsr       ,x                  ; INITALIZE THE SCI.

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
; include "BASICLB1.ASM"
;******************************************************************************
;******************************************************************************

; *main()
; *{
; initvars();            /* initalize all variables & pointers */
; outheader();           /* send startup message to console */
; outrdy();              /* output ready message */

MAIN

MAINC               jsr       INITVARS            ; INITALIZE ALL INTERNAL VARIABLES.
                    ldx       #PSSTART
                    lda       ASFLAG              ; get the auto start flag.
                    cmpa      #$55
                    bne       MAIN9
                    cli                           ; ALLOW ALL INTERRUPTS TO BE SERVICED.
                    jsr       CRUN

MAIN9               lda       #20                 ; \
MAIN9A              jsr       DELAY               ; | short delay
                    deca                          ; |
                    bne       MAIN9A              ; /
                    bsr       OUTHEADR            ; PRINT HEADER.
MAINW
MAIN2               lds       #$0FFF              ; load stack pointer

                    cli                           ; ALLOW ALL INTERRUPTS TO BE SERVICED

                    clra                          ; CLEAR D.
                    clrb
                    std       TIMECMP             ; DON'T ALLOW "ONTIME" INTERRUPTS TO OCCUR.
                    std       ONIRQLIN            ; DON'T ALLOW "ONIRQ" INTERRUPTS TO OCCUR.
                    std       ONPACLIN            ; DON'T ALLOW "PACC" INTERRUPTS TO OCCUR.
                    jsr       OUTRDY              ; PRINT READY MESSAGE.

;************
; while(1)               /* do forever */
; {
;  outprmpt();           /* output prompt */
;  getline();            /* getline from console */
;  skipspcs();           /* ignore leading spaces in input buffer */
;  if(chckcmds()) continue;           /* check for commands */
;  parse();              /* translate/execute line */
;  if(errcode) rpterr(); /* if an error occured somewhere, report it. */
;  errcode=0;            /* clear error status */
; }
; *}
;
MAIN1               equ       *
                    clr       IMMID               ; CLEAR THE IMMIDIATE MODE FLAG.
                    clr       ERRCODE             ; CLEAR THE ERROR CODE BYTE.
                    clr       RUNFLAG             ; CLEAR THE RUN MODE FLAG.
                    jsr       OUTPRMPT            ; SEND PROMPT TO CONSOLE.
                    jsr       GETLINE             ; GO GET LINE FROM OPERATOR.
                    bsr       SKIPSPCS            ; SKIP ANY LEADING SPACES.
                    jsr       CHCKCMDS            ; GO CHECK FOR ANY COMMANDS.
                    cpd       #0                  ; WERE THERE ANY?
                    bne       MAIN2               ; YES. CONTINUE.
                    ldx       VAREND              ; SAVE CURRENT END OF VARIABLE AREA IN CASE LINE
                    stx       FENCE               ; HAS AN ERROR IN IT. (SO NO SPURIOUS VARIABLES
;                             ; ARE ALLOCATED)
                    jsr       PARSE
                    tst       IMMID               ; DID WE EXECUTE IN IMMIDATE MODE?
                    bne       MAIN2               ; YES. PRINT READY MESSAGE.
                    bra       MAIN1               ; NO. JUST PRINT PROMPT.

;
;
MAIN3               ldx       FENCE               ; GET THE VAREND FENCE.
                    clr       ,x                  ; MARK "OLD" END OF VARIABLE AREA IN CASE ANY
;                             ; VARIABLES WERE ALLOCATED.
                    stx       VAREND              ; RESTORE THE POINTER.
                    bra       MAIN2               ; CONTINUE AFTER ERROR.

;
;        /***** skipspcs() *****/

; *skipspcs()
; *{
; while(*ibufptr==SPC) ++ibufptr;
; return;
; *}
;
SKIPSPCS            equ       *
SKIPSPC1            jsr       GETCHR
                    cmpa      #SPC
                    bne       SKIPSPC2
                    jsr       INCIBP
                    bra       SKIPSPC1

SKIPSPC2            rts

;

;        /***** outheader *****/

; *outheader()
; *{
; ("BASIC11 v1.1");
; nl();
; nl();
; pl("Written by Gordon Doughman");
; nl();
; *}
;
OUTHEADR            ldx       #HEADER
                    jmp       PL

;
HEADER              fcc       ESC,"[2J"           ; CLR SCREEN
                    fcc       ESC,"[0;0H"         ; TOP LEFT CORNER FCC "BASIC11 V1.55"
                    fcb       $0D,$0A
                    fcc       "Copyright 1985-1990 Gordon Doughman"
                    fcb       $0D,$0A
                    fcc       "Modified for the PMB Electronics CPU_3A2 HC11F1 board, May 2003"
                    fcb       $0D,$0A,0
;

;        /***** outrdy() *****/

; *outrdy()
; *{
; nl();
; pl("READY");
; return;
; *}
;
OUTRDY              ldx       #READY
                    bra       PL

;
READY               fcb       $0D,$0A
                    fcc       "READY"
                    fcb       $00
;
;
;        /***** getline() *****/

; *getline()
; *{
; short chrcnt;
; char c;
; chrcnt=IBUFLEN;
; ibufptr=inbuff;
;
GETLINE             equ       *
                    ldb       #IBUFLEN-1
                    ldx       INBUFFS
;
; while((c=inbyte())!=EOL && (chrcnt>0))
; {
;
GETLINE3            jsr       INBYTE
                    cmpa      #EOL
                    beq       GETLINE1
                    tstb
                    beq       GETLINE1
;
;  if(c==BS)
;  {
;   if(ibufptr!=inbuff)
;   {
;    ibufptr-=1;         /* point to previous buffer location */
;    outbyte(SPC);        /* erase character on screen */
;    outbyte(BS);        /* back cursor up */
;    chrcnt+=1;
;   }
;   else
;    continue;
;   }
;
;
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
;
;  if(c < SPC) continue;  /* ignore all other control characters */
;
GETLINE2            equ       *
                    cmpa      #CNTRLC             ; IS IT A "BREAK"?
                    bne       GETLINE4            ; NO. GO PROCESS THE CHARACTER.
                    inc       CONTFLAG            ; DONT ALLOW A "CONT".
                    jmp       CHCKBRK2            ; GO DO A BREAK.

GETLINE4            cmpa      #SPC
                    blo       GETLINE3
;
;  *ibufptr=c;            /* save character in input buffer */
;  ibufptr+=1;           /* point to next location */
;  chrcnt-=1;
; }
                    sta       ,x
                    inx
                    decb
                    bne       GETLINE3
;
; *ibufptr=EOL;          /* put EOL in input buffer */
; ibufptr=inbuff;        /* initalize the input buffer pointer */
; return;
; *}
;
GETLINE1            equ       *
                    lda       #EOL
                    sta       ,x
                    ldx       INBUFFS
                    stx       IBUFPTR
                    rts

;
ToUpper             equ       *
                    cmpa      #'a'                ; is the character less than a lower case 'a'?
                    blo       ToUpper1
                    cmpa      #'z'
                    bhi       ToUpper1
                    anda      #$df
ToUpper1            rts

;        /***** outprmpt() *****/

; *outprmpt()
; *{
; nl();          /* go to new line */
; outbyte('#');  /* send prompt to console */
; return;
; *}
;
OUTPRMPT            equ       *
                    ldx       #PRMPTSTR
                    bra       PL

;
PRMPTSTR            equ       *
                    fcb       $0D,$0A
                    fcc       "#"
                    fcb       0
;
;        /***** nl() *****/

; nl()                    /* send CR/LF to console */
; *{
; outbyte(CR);
; outbyte(LF);
; return;
; *}
;
;
NL2                 bsr       NL
NL                  equ       *
                    ldx       #CRLFSTR
                    bsr       PL
                    clr       PRINTPOS            ; SET THE CURRENT PRINT POSITION TO 0.
                    rts

;
CRLFSTR             fcb       $0A,$0D,$00
;
;        /***** pl() *****/

; *pl(ptr)                 /* send line to console */
; *char *ptr;
; *{
; int k; char c;
; k=0;
; while(c=ptr[k++]) outbyte(c);
; return;
; *}
;
PL                  equ       *
PL1                 lda       ,x
                    beq       PL2
                    jsr       OUTBYTE
                    inx
                    bra       PL1

PL2                 rts

;

;        /***** parse() *****/

; *parse()
; *{
; int num;
; tbufptr=tknbuf;                /* initalize the token buffer pointer */
;
PARSE               equ       *
                    ldx       TKNBUFS             ; Get the start of the token buffer
                    stx       TBUFPTR
;
; if(num=getlinum())             /* get line number if present */
; {
;  if(*ibufptr==EOL)             /* was line # followed by CR? */
;  {                             /* yes. */
;   delline(num);                /* go delete the line from the prog buffer */
;   return;
;  }
;  immid=0;                      /* flag as not immidiate */
;  putlinum(num);                /* put line number in buffer */
; }
;  else immid=1;                  /* if no line # flag as immidiate*/
;
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

;
; if(errcode) return;             /* if line number error, return */
; xlate();                        /* if translation error, return */
; if(errcode) return;
; if(immid) runline();            /* if immidiate mode run 1 line */
;  else storlin();                /* if not store pgm line */
; return;                         /* go get next line */
; *}
;
PARSE3              jsr       XLATE
                    tst       IMMID
                    beq       PARSE5
                    jmp       RUNLINE             ; GO RUN THE LINE & RETURN.

PARSE5              jsr       STORLIN             ; GO STORE LINE & RETURN.
                    ldx       VAREND
                    inx
                    stx       STRASTG
                    rts

;

;        /***** getlinum *****/

; *getlinum()
; *{
; int num;
; num=0;
;
GETLINUM            equ       *
;
;
                    pshy
                    clra
                    psha
                    psha
                    tsy
;
; if(numeric(*ibufptr)==0) return(0);    /* if 1st char not numeric, rtn 0 */
;
                    ldx       IBUFPTR
                    lda       ,x
                    bsr       NUMERIC
                    bcc       GTLNUM4
;
; while(numeric(*ibufptr))       /* while *ibufptr is numeric */
; {
;  num=num*10+(*ibufptr-'0');    /* get a digit */
;  ibufptr++;                    /* advance input buffer pointer */
;  if(num<=0) { errcode=LINRANG; return(0); }
; }
; return(num);
; *}
;
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

;
;
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

;
;
;        /***** putlinum *****/

; *putlinum(lnum)
; *int lnum;
; *{
; putint(lnum);          /* put line # in token buffer */
; *tbufptr++=0;          /* hold place for length of line */
; return;
; *}
;
PUTLINUM            equ       *
                    jsr       PUTDTOK
                    clra
                    jmp       PUTTOK

;
;        /***** numeric() *****/

; *numeric(c)
; *char c;
; *{
; c=c&127;
; return((c>='0')&(c<='9'));
; *}
;
NUMERIC             equ       *
                    cmpa      #'0'
                    blo       NUMERIC1
                    cmpa      #'9'
                    bhi       NUMERIC1
                    sec
                    rts

NUMERIC1            clc
                    rts

;        /***** alpha() *****/

; *alpha(c)
; *char c;
; *{
; c=c&127;
; return((c>='A')&(c<='Z'));
; *}
;
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

;        /***** alphanum *****/

; *alphanum(c)
; *char c;
; *{ return ((alpha(c)) | (numeric(c))); }
;
ALPHANUM            equ       *
                    bsr       ALPHA
                    bcc       ALPHANU1
                    rts

ALPHANU1            bra       NUMERIC

;
; */*****************************************
;              xlate()
; translate the input buffer into tokenized
; form placing the results into tknbuf
;******************************************/
;
; *xlate()
; *{
; while(*ibufptr!=EOL)
; {
;  ifwhflag=0;              /* set IF flag to zero */
;  blanks();              /* skip all blanks */
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
; */*else if(match("SWAP")) xswap();*/
;  else if(match("TRON")) xtron();
;  else if(match("TROFF")) xtroff();
;  else if(match("WHILE")) xwhile();
; */*else if(match("ONIRQ")) xonirq();*/
;  else ximplet();                 /* if no keyword, assume implied LET */
;  if(errcode) return;
;  blanks();
;  if(*ibufptr==MIDEOL) { xmideol(); continue; }
;  if(*ibufptr!=EOL) { errcode=SYTXERR; return; }
;  }
; *tbufptr=EOLTOK;               /* put token eol in token buffer */
; tknbuf[2]=tbufptr-tknbuf+1;    /* put line length into tokenized line */
; return;
; *}
;
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

;
;
;        KEYWORD LOOK UP TABLE
;
;
KEYWORDS            equ       *
DATA                fcc       "DATA"
                    fcb       0
                    fcb       DATATOK
                    fdb       XDATA
LET                 fcc       "LET"
                    fcb       0
                    fcb       LETTOK
                    fdb       XLET
READ                fcc       "READ"
                    fcb       0
                    fcb       READTOK
                    fdb       XREAD
RESTORE             fcc       "RESTORE"
                    fcb       0
                    fcb       RESTRTOK
                    fdb       XRESTORE
GOSUB               fcc       "GOSUB"
                    fcb       0
                    fcb       GOSUBTOK
                    fdb       XGOSUB
GOTO                fcc       "GOTO"
                    fcb       0
                    fcb       GOTOTOK
                    fdb       XGOTO
ONTIME              fcc       "ONTIME"
                    fcb       0
                    fcb       ONTIMTOK
                    fdb       XONTIME
ONIRQ               fcc       "ONIRQ"
                    fcb       0
                    fcb       ONIRQTOK
                    fdb       XONIRQ
ONPACC              fcc       "ONPACC"
                    fcb       0
                    fcb       ONPACTOK
                    fdb       XONPACC
ON                  fcc       "ON"
                    fcb       0
                    fcb       ONTOK
                    fdb       XON
RETURN              fcc       "RETURN"
                    fcb       0
                    fcb       RETNTOK
                    fdb       XRETURN
IIF                 fcc       "IF"
                    fcb       0
                    fcb       IFTOK
                    fdb       XIF
INPUT               fcc       "INPUT"
                    fcb       0
                    fcb       INPUTTOK
                    fdb       XINPUT
PRINT               fcc       "PRINT"
                    fcb       0
                    fcb       PRINTTOK
                    fdb       XPRINT
                    fcc       "?"
                    fcb       0
                    fcb       PRINTTOK
                    fdb       XPRINT
FOR                 fcc       "FOR"
                    fcb       0
                    fcb       FORTOK
                    fdb       XFOR
NEXT                fcc       "NEXT"
                    fcb       0
                    fcb       NEXTTOK
                    fdb       XNEXT
STOPSS              fcc       "STOP"
                    fcb       0
                    fcb       STOPTOK
                    fdb       XSTOP
ENDWH               fcc       "ENDWH"
                    fcb       0
                    fcb       ENDWHTOK
                    fdb       XENDWH
ENDS                fcc       "END"
                    fcb       0
                    fcb       ENDTOK
                    fdb       XEND
REM                 fcc       "REM"
                    fcb       0
                    fcb       REMTOK
                    fdb       XREM
TRON                fcc       "TRON"
                    fcb       0
                    fcb       TRONTOK
                    fdb       XTRON
TROFF               fcc       "TROFF"
                    fcb       0
                    fcb       TROFFTOK
                    fdb       XTROFF
WHILE               fcc       "WHILE"
                    fcb       0
                    fcb       WHILETOK
                    fdb       XWHILE
POKE                fcc       "POKE"
                    fcb       0
                    fcb       POKETOK
                    fdb       XPOKE
DIM                 fcc       "DIM"
                    fcb       0
                    fcb       DIMTOK
                    fdb       XDIM
EEP                 fcc       "EEP"
                    fcb       0
                    fcb       EEPTOK
                    fdb       XEEP
PORTA               fcc       "PORTA"
                    fcb       0
                    fcb       PORTATOK
                    fdb       XPORTA
PORTB               fcc       "PORTB"
                    fcb       0
                    fcb       PORTBTOK
                    fdb       XPORTB
PORTC               fcc       "PORTC"
                    fcb       0
                    fcb       PORTCTOK
                    fdb       XPORTC
PORTD               fcc       "PORTD"
                    fcb       0
                    fcb       PORTDTOK
                    fdb       XPORTD
INBYTES             fcc       "INBYTE"
                    fcb       0
                    fcb       INBYTTOK
                    fdb       XINBYTE
TIME                fcc       "TIME"
                    fcb       0
                    fcb       TIMETOK
                    fdb       XTIME
RETI                fcc       "RETI"
                    fcb       0
                    fcb       RETITOK
                    fdb       XRETI
PACC                fcc       "PACC"
                    fcb       0
                    fcb       PACCTOK
                    fdb       XPACC
SLEEP               fcc       "SLEEP"
                    fcb       0
                    fcb       SLEEPTOK
                    fdb       XSLEEP
RTIMES              fcc       "RTIME"
                    fcb       0
                    fcb       RTIMETOK
                    fdb       XRTIME
CLS                 fcc       "CLS"
                    fcb       0
                    fcb       CLSTOK
                    fdb       XCLS
                    fcb       0                   ; END OF TABLE MARKER.

; *blanks()
; *{
; short spcnt;
; spcnt=0;
; while(*ibufptr==SPC) { ibufptr++; spcnt++; }
;
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

;
; if(spcnt==0) return;
;
BLANKS2             tstb
                    bne       BLANKS3
                    pulx
                    rts

;
; if(spcnt>1)
;  {
;   *tbufptr++=MSCNTOK;
;   *tbufptr++=spcnt;
;  }
;
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

;
; else
;  {
;   *tbufptr++=SSCNTOK;
;  }
; return;
; *}
;
BLANKS4             ldb       #SSCNTOK
                    bra       BLANKS5

;
;
; *<><><><><><><> NOTE: THIS FUNCTION NOT NEEDED <><><><><><><>
;
; */**************************************
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

; */****************************************
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
; while(str2[k])         /* we're not at the end of string2 */
; {
;  if((str1[k])!=(str2[k])) return(0);
;  k++;
; }
; return(k);
; *}
;
STREQ               equ       *
                    ldd       IBUFPTR             ; SAVE VALUE OF POINTER.
;        PSHD
                    pshb
                    psha
STREQU4             lda       ,x
                    beq       STREQU2
STREQU1             bsr       GETNXCHR
                    jsr       ToUpper             ; Make the character upper case.
                    cmpa      ,x
                    beq       STREQU3
;        PULD
                    pula
                    pulb
                    std       IBUFPTR
                    clc
                    rts

STREQU3             inx
                    bra       STREQU4

STREQU2             pula
                    pulb
                    sec
                    rts

;
;
;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER.
;
;
GETCHR              pshx                          ; SAVE THE X REGISTER.
                    ldx       IBUFPTR             ; GET POINTER.
                    lda       ,x                  ; GET A CHARACTER.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

;
;
;        THIS ROUTINE GETS THE NEXT CHARACTER FROM THE INPUT BUFFER
;        AND ADVANCES THE POINTER TO POINT TO THE NEXT CHARACTER.
;
;
GETNXCHR            bsr       GETCHR
;                       FALL THROUGH TO INCIBP.
;
;
;        THIS ROUTINE JUST INCREMENTS THE INPUT BUFFER POINTER.
;
;
INCIBP              pshx                          ; SAVE X.
                    ldx       IBUFPTR             ; GET POINTER.
INCIBP1             inx                           ; ADVANCE POINTER.
                    stx       IBUFPTR             ; UPDATE POINTER.
INCIBP2             pulx                          ; RESTORE X
                    rts                           ; RETURN.

;
;
;        THIS ROUTINE PUTS THE WORD IN THE D-REG. INTO THE TOKEN BUFFER
;        AND ADVANCES THE TOKEN BUFFER POINTER.
;
;
PUTDTOK             bsr       PUTTOK              ; PUT THE FIRST BYTE INTO THE TOKEN BUFFER.
                    tba                           ; PUT THE 2ND BYTE INTO A.
;                            ; FALL THROUGH TO PUTTOK.
;
;
;        THIS ROUTINE PUTS THE CHARACTER IN THE A-REG. INTO THE TOKEN
;        BUFFER AND ADVANCES THE TOKEN BUFFER POINTER.
;
;
PUTTOK              pshx                          ; SAVE X.
                    pshb
                    psha                          ; (9/12/89).
                    ldx       TBUFPTR             ; GET POINTER.
                    sta       ,x                  ; PUT CHARACTER.
PUTTOK1             inx                           ; ADVANCE POINTER.
                    stx       TBUFPTR             ; SAVE POINTER.

                    ldd       TKNBUFS             ; get the starting address of the token buffer.
                    addd      #TBUFLEN            ; add the length of the buffer to it.
                    cpd       TBUFPTR             ; IS THE TOKEN BUFFER FULL?
                    pula                          ; (9/12/89).
                    pulb                          ; restore the b reg.
                    bhi       INCIBP2             ; NO. RESTORE X AND RETURN.
                    lda       #EXPCXERR           ; YES. FLAG THE ERROR.
                    jmp       RPTERR              ; GO REPORT IT.

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "BASICLB2.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; *<><><><><><> NOTE: FUNCTION PERFORMED IN "XLATE" <><><><><><>
;
; */***** xmideol() *****/
;
; *xmideol()
; *{
; *tbufptr++=MEOLTOK;
; ++ibufptr;
; return;
; *}
;
;
;
; */****** common code for GOSUB and GOTO *****/
;
; *xgo(gotok)
; *char gotok;
; *{
; int num;
; *tbufptr++=gotok;      /* put GOTO or GOSUB token in buffer */
; blanks();              /* skip blanks before line number */
; *tbufptr++=LCONTOK;    /* put line number constant token in buffer */
; num=getlinum();        /* get line number */
; if(num==0) errcode=LINENERR;   /* if 0, line number error */
; if(errcode) return;    /* if error, return */
; putint(num);           /* put line number in buffer */
; return;
; *}
;
XGOSUB              equ       *
XGOTO               equ       *
;        JSR    BLANKS
                    lda       #LCONTOK
                    bsr       PUTTOK
                    jsr       GETLINUM
XGOTO2              bra       PUTDTOK

;
;
; *<><><><><><> ROUTINE NOT NEEDED <><><><><><>
;
; */***** GOSUB *****/
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
; */***** GOTO *****/
;
; *xgoto()
; *{
; xgo(GOTOTOK);
; return;
; *}

; */***** RETURN *****/
;
; *xreturn()
; *{
; *tbufptr++=RETNTOK;    /* put RETURN token in buffer */
; return;
; *}
;
; */***** STOP *****/
;
; *xstop()
; *{
; *tbufptr++=STOPTOK;    /* put STOP token in buffer */
; return;
; *}
;
; */***** END *****/
;
; *xend()
; *{
; *tbufptr++=ENDTOK;     /* put end token in buffer */
; return;
; *}
;
; */***** TRON *****/
;
; *xtron()
; *{
; *tbufptr++=TRONTOK;    /* put TRON token in buffer */
; return;
; *}
;
; */***** TROFF *****/
;
; *xtroff()
; *{
; *tbufptr++=TROFFTOK;   /* put TROFF token in buffer */
; return;
; *}
;
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
XCLS                equ       *
                    rts                           ; NULL FUNCTIONS BECAUSE TOKEN PLACEMENT IS DONE IN

;                            ; XLATE FUNCTION.
;

; */***** REM *****/
;
; *xrem()
; *{
; char c;
; *tbufptr++=REMTOK;     /* put rem token in buffer */
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
; */***** xdata() *****/
;
; *xdata()
; *{
; char c;
; *tbufptr++=DATATOK;            /* put DATA token in buffer */
; while((c=*ibufptr)!=EOL)
; {
;  if(c==',') *tbufptr++=COMMATOK;
;  else *tbufptr++=c;
;  ++ibufptr;
; }
; return;
; *}
;
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

;
;
XPORTA              equ       *
XPORTB              equ       *
XPORTC              equ       *
XPORTD              equ       *
                    ldb       #NUM                ; WE'RE XLATING A NUMERICAL STATEMENT.
                    bra       ASIGNMT1            ; GO DO IT LIKE AN ASIGNMENT STATEMENT.

;
;
;
;

; */***** LET *****/
;
; *xlet()
; *{
; letcom(LETTOK);                /* pass LET token to common code */
; return;
; *}
;
; */***** implied LET *****/
;
; *ximplet()
; *{
; letcom(IMLETTOK);
; return;
; *}
;
; */***** common code for explicit & implicit LET *****/
;
; *letcom(letok)
; *short letok;
; *{
; *tbufptr++=letok;              /* put LET token in buffer */
; blanks();              /* skip blanks before assignment statement */
; if(ibufptr=='@') { *tbufptr++=INDIRTOK; ++ibufptr; }
; asignmt();                     /* evaluate expression */
; return;
; *}
;
XLET                equ       *
XIMPLET             equ       *
;        JSR    BLANKS
; *XLET1    JMP    ASIGNMT
;
;

; */***** asignmt() *****/
;
; *asignmt()
; *{
; *short type;
; if((type=getvar())==0) return; /* get variable & return type */
; if(errcode) return;
; if(*ibufptr++!='=') { errcode=IVEXPERR; return; } /* invalid expression */
; *tbufptr++=EQUALTOK;           /* put equals token in buffer */
; xexpres(type);                 /* build expression in token buffer */
; return;
; *}
;
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
;                               FALL THROUGH TO XEXPRES.
;
;

; */***** xexpres() *****/
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
;
;  if(*ibufptr=='(')     /* open paren? */
;  {
;   *tbufptr++=OPARNTOK; /* put in token buffer */
;   ++ibufptr;           /* point to next char in input buffer */
;   xexpres(type);       /* go get sub expression */
;   if(errcode) return;
;   if(*ibufptr!=')') { errcode=UPARNERR; return; }
;   *tbufptr++=CPARNTOK; /* put it in the token buffer */
;   ++ibufptr;           /* point to the next char in the input buffer */
;   goto chkoprtr;
;  }
;
XEXPRS30            bsr       GETCHR
                    cmpa      #'('
                    bne       XEXPRS1
                    bsr       INCIBP
                    lda       #OPARNTOK
                    bsr       PUTTOK
                    lda       ,y
                    bsr       XEXPRES
XEXPRS2             jsr       GETNXCHR
                    cmpa      #')'
                    beq       XEXPRS3
                    lda       #UPARNERR
                    jmp       RPTERR

XEXPRS3             lda       #CPARNTOK
                    jsr       PUTTOK
                    bra       CHKOPRTR

;
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
;
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
;
;
; */* now look for operator or end of expression */
;
;  chkoprtr:
;  c=*ibufptr;
;  if(c==EOL | c==MIDEOL | c==SPC | c==COMMA | c==SEMI | c==')')
;  {
;   return(c);
;  }
;
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
;
;
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
                    bsr       CKNUMOP
                    bcs       XEXPRS17
                    bsr       CKBOLOP
                    bcs       XEXPRS17
                    tst       IFWHFLAG
                    beq       XEXPRS18
                    bsr       CKLOGOP
                    bra       XEXPRS17

XEXPRS18            lda       #NULL
                    bra       XEXPRS17

;
;
;  else { errcode=IDTYERR; return; }
;
XEXPRS21            equ       *
                    lda       #IDTYERR
                    jmp       RPTERR

;
;
;  if(c==NULL) { errcode=OPRTRERR; return; }
;  *tbufptr++=c;
; }
; return;
; *}
;
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

;
;
; */***** cknumop() *****/
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
;
CKNUMOP             equ       *
                    ldx       #NUMOPTBL
;
CKOP                bsr       TBLSRCH
                    bcs       CKOP1
                    lda       #NULL
CKOP1               rts

;
;

; */***** ckbolop() *****/
;
; *ckbolop()
; *{
; if(match("AND")) return(ANDTOK);
; else if(match("OR")) return(ORTOK);
; else if(match("EOR")) return(EORTOK);
; else return(NULL);
; *}
;
CKBOLOP             equ       *
                    ldx       #BOLOPTBL
                    bra       CKOP

;
;

; */***** cklogop() *****/
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
;
CKLOGOP             equ       *
                    ldx       #LOGOPTBL
                    bra       CKOP

;
;
;
;
; *<><><><><> NOTE: THIS ROUTINE HAS NO 'C' COUNTER PART <><><><><><>
;
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

;
;
NUMOPTBL            equ       *
PLUS                fcc       "+"
                    fcb       0
                    fcb       PLUSTOK
MINUS               fcc       "-"
                    fcb       0
                    fcb       MINUSTOK
MULT                fcc       "*"
                    fcb       0
                    fcb       MULTTOK
DIV                 fcc       "/"
                    fcb       0
                    fcb       DIVTOK
MODS                fcb       $5C,$00
                    fcb       MODTOK
                    fcb       0                   ; END OF TABLE FLAG.
;
BOLOPTBL            equ       *
ANDS                fcc       ".AND."
                    fcb       0
                    fcb       ANDTOK
ORS                 fcc       ".OR."
                    fcb       0
                    fcb       ORTOK
EORS                fcc       ".EOR."
                    fcb       0
                    fcb       EORTOK
                    fcb       0                   ; END OF TABLE FLAG.
;
LOGOPTBL            equ       *
LTEQ                fcc       "<="
                    fcb       0
                    fcb       LTEQTOK
GTEQ                fcc       ">="
                    fcb       0
                    fcb       GTEQTOK
NOTEQ               fcc       "<>"
                    fcb       0
                    fcb       NOTEQTOK
LT                  fcc       "<"
                    fcb       0
                    fcb       LTTOK
GT                  fcc       ">"
                    fcb       0
                    fcb       GTTOK
EQ                  fcc       "="
                    fcb       0
                    fcb       EQTOK
                    fcb       0                   ; END OF TABLE FLAG.
;
;
UINARYOP            equ       *
NEGS                fcc       "-"
                    fcb       0
                    fcb       NEGTOK
NOTS                fcc       "NOT"
                    fcb       0
                    fcb       NOTTOK
                    fcb       0                   ; END OF TABLE MARKER.


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "BASICLB3.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; */***** getvar *****/
;
; */* tries to make a variable out of what is currently being pointed to by */
; */* 'ibufptr' and places it into the variable symbol table if it is not   */
; */* already there */
;
; *getvar()
; *{
; short vartype,cnt;
; char varname[3];
; int offset;
; for(cnt=0; cnt<=2; cnt++) { varname[cnt]=0; } /* clr out var name */
; if(alpha(*ibufptr)) { varname[0]=*ibufptr++; } /* is 1st char an alpha? */
;  else { errcode=ILVARERR; return(0); } /* no. error */
;
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
;
;
; if(alphanum(*ibufptr)) { varname[1]=*ibufptr++; }
; if((vartype=chcktyp())==0) { vartype=FVARTOK; }
;  else { ++ibufptr; }
;
                    jsr       GETCHR
                    jsr       ALPHANUM
                    bcc       GETVAR2
                    jsr       ToUpper
                    sta       1,y
                    jsr       INCIBP
GETVAR2             bsr       CHCKTYP
                    sta       3,y
;
;
; if((offset=findvar(vartype,varname))==-1) /* is var already in table? */
; {
;  if(errcode) return;
;  if((offset=putvar(vartype,varname))==-1) return;  /* no. put it there */
; }
; if(errcode) return;
;
                    bsr       FINDVAR
                    cpd       #-1
                    bne       GETVAR5
GETVAR4             lda       3,y
                    jsr       PUTVAR
;
;
; *tbufptr++=vartype;    /* put variable type byte in token buffer */
; putint(offset);        /* put offset after it */
; if((vartype==IVARTOK) | (vartype==FVARTOK)) return(NUM);
; return(STRING);
; *}
;
GETVAR5             equ       *
;        PSHD
                    pshb
                    psha
                    lda       3,y
                    jsr       PUTTOK
;        PULD
                    pula
                    pulb
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
                    beq       GETVAR6
                    lda       #STRING
GETVAR6             ins
                    ins
                    ins
                    ins
                    puly
                    rts

;
;
;
; */***** chcktype *****/
;
; *chcktyp()
; *{
; if(*ibufptr=='%') return(IVARTOK);
;  else if(*ibufptr=='$') return(SVARTOK);
;  else return(0);
; *}
;
CHCKTYP             equ       *
                    lda       #IVARTOK            ; IN V1.0 ONLY INTEGER VARIABLES ARE SUPPORTED.
                    psha                          ; IN V2.0 FLOATING POINT VARIABLES WILL BE
                    jsr       GETCHR              ; SUPPORTED.
                    cmpa      #'('                ; IS A SUBSCRIPT FOLLOWING THE NAME?
                    pula                          ; RESTORE THE TOKEN TYPE.
                    bne       CHCKTYP4            ; NO. RETURN.
                    adda      #$10                ; YES. MAKE IT AN ARRAY VARIABLE.
CHCKTYP4            rts                           ; RETURN.

;
; */***** findvar *****/
;
; *findvar(vartype,varname)
; *short vartype;
; *char *varname;
; *{
; char *varptr;
; varptr=varbegin;               /* point to the start of the var table */
; while(*varptr)                 /* we're not to the end of the table */
;
FINDVAR             equ       *
                    ldx       VARBEGIN
FINDVAR1            tst       ,x
                    beq       FINDVAR2
;
; {
;  if(*varptr==vartype)          /* is the current var the same type? */
;  {                             /* yes. */
;   if(streq(varptr+1,varname))  /* is the name the same? */
;   {                            /* yes. */
;    return(varptr-varbegin);    /* return the offset from the table start */
;   }
;  }
;
                    cmpa      ,x
                    bne       FINDVAR3
                    ldb       1,x
                    cmpb      ,y
                    bne       FINDVAR3
                    ldb       2,x
                    cmpb      1,y
                    bne       FINDVAR3
                    xgdx
                    subd      VARBEGIN
                    rts

;
;  /* if not, advance to the next variable in the table */
;  if(*varptr==IVARTOK) varptr=varptr+ISIZ+3;
;  else if(*varptr==SVARTOK) varptr=varptr+SSIZ+3;
;  else if(*varptr==FVARTOK) varptr=varptr+FSIZ+3;
;  else { errcode=ILTOKERR; return(-1); }
; }
;
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

;
; return(-1);
; *}
;
;
;
; */***** putvar *****/
;
; *putvar(vartype,varname)
; *short vartype;
; *char *varname;
; *{
; *short count,n;
; *char *varadd;
; varadd=varend;         /* save begining addr of var we are storing */
; *varend++=vartype;     /* put token/type in variable symbol table */
; *varend++=*varname++;  /* put variable name in */
; *varend++=*varname++;
;
PUTVAR              equ       *
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
;
; if(vartype==IVARTOK) count=ISIZ+1;     /* determine # of bytes for this */
; else if(vartype==SVARTOK) count=SSIZ+1;/* variable */
; else if(vartype==FVARTOK) count=FSIZ+1;
; else { errcode=ILTOKERR; return(-1); }
; for(n=1;n<=count;n++) *varend++=0;      /* zero the storage */
; --varend;
; if(varend > varmend) { errcode=OMEMERR; return(-1); } /* memory overflow? */
; vmemavil-=count;       /* decrement the amt of avail memory */
; return(varadd-varbegin);       /* return offset */
; *}
;
                    bsr       CLRVAR
                    clr       ,x                  ; CLEAR 1 BYTE BEYOND THE END OF THE VAR AREA.
                    stx       VAREND
                    cpx       VARMEND
                    bls       PUTVAR5
                    lda       #OMEMERR
                    bra       CLRVAR6

PUTVAR5             equ       *
;        PULD
                    pula
                    pulb
                    subd      VARBEGIN
;        PSHD                ; SAVE THE OFFSET TO THIS VARIABLE.
                    pshb
                    psha
                    jsr       CCLEAR3             ; CLEAR ALL VARIABLES SINCE WE MAY HAVE TRASHED
;                            ANY ARRAYS THAT HAD BEEN ALLOCATED.
;        PULD                RESTORE THE "NEW" VARIABLE OFFSET.
                    pula
                    pulb
                    rts

;
;
CLRVAR              bita      #$10                ; IS IT AN ARRAY VARIABLE?
                    beq       CLRVAR8             ; NO. CONTINUE.
                    ldb       #ASIZ               ; YES. GET THE DICTIONARY SIZE+1.
                    bra       CLRVAR1             ; PUT THE VARIABLE IN THE DICTIONARY.

CLRVAR8             cmpa      #IVARTOK
                    bne       CLRVAR4
                    ldb       #ISIZ
CLRVAR1             clr       ,x
                    inx
                    decb
                    bne       CLRVAR1
                    rts

CLRVAR4             lda       #ILTOKERR
CLRVAR6             jmp       RPTERR

;
;
; */***** getcon() *****/
;
; *getcon()
; *{
; int const;
; char *litp;
; short count;
; litp=ibufptr;          /* save a pointer to start of constant */
; if(*ibufptr=='"') { getscon(); return(STRING); } /* if " get strng */
;
GETCON              equ       *
                    jsr       GETCHR
;
;
;  else if(*ibufptr=='$') { ++ibufptr; const=gethex(); } /* if '$' get hex */
;  else const=getdeci();         /* else assume its a decimal constant */
; if(errcode) return(0);         /* if error abort */
;
GETCON2             equ       *
                    ldx       IBUFPTR
                    pshx
                    cmpa      #'$'
                    bne       GETCON3
                    jsr       INCIBP
                    bsr       GETHEX
                    bra       GETCON4

GETCON3             bsr       GETDECI
;
;
; *tbufptr++=ICONTOK;            /* put integer constant token in buffer */
; putint(const);                 /* follow it with the constant */
; count=ibufptr-litp;    /* get number of bytes in source form of const. */
; *tbufptr++=count;       /* put it in the token buffer */
; while(litp < ibufptr) *tbufptr++=*litp++; /* copy source form into buffer */
; return(NUM);           /* return the constant type */
; }
;
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

;
;
; */***** getdeci() *****/
;
; *getdeci()
; *{
; char c;
; int num;
; num=0;
; if(numeric(*ibufptr)==0)       /* is 1st char numeric? */
;  { errcode=SYTXERR; return; }  /* no. flag error & return */
; while(numeric(c=*ibufptr))     /* yes. while *ibufptr is numeric */
; {
;  num=num*10+(c-'0');           /* build number */
;  if(num < 0) { errcode=INTOVERR; return; }     /* if <0 flag error & ret */
;  ++ibufptr;
; }
; return(num);           /* return the value */
; *}
;
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

;
;
; */***** gethex() *****/
;
; *gethex()
; *{
; char c;
; short count;
; int num;
; num=count=0;
; if(hexdig(*ibufptr)==0)        /* is the char a hex digit? */
;  { errcode=IVHEXERR; return; } /* no. flag error & return */
;
GETHEX              equ       *
                    pshy
                    clra
                    psha
                    psha
                    tsy
                    ldx       IBUFPTR
                    lda       ,x
                    bsr       HEXDIG
                    bcs       GETHEX1
                    lda       #IVHEXERR
CHCKERR             tst       RUNFLAG
                    beq       GETHEX5
                    jmp       RPTRERR

GETHEX5             jmp       RPTERR

;
;
; while(hexdig(c=*ibufptr))      /* while a hex digit is in the buffer */
; {
;  if(numeric(c)) num=num*16+(c-'0');  /* build the number */
;   else num=num*16+(c-55);
;  if(count++ > 4)
;   { errcode=HEXOVERR; return; }  /* if over 4 digits flag overflow & ret */
;  ++ibufptr;
; }
; return(num);   /* return constant value */
; *}
;
GETHEX1             equ       *
                    lda       ,x
                    bsr       HEXDIG
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

;
;
; */***** hexdig() *****/
;
; *hexdig(c)
; *char c;
; *{
; return(numeric(c) | (c>='A' & c<='F')); /* return true if c is hex */
; *}
;
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

;
;
; */***** getscon *****/
;
; *getscon()
; *{
; short count;
; char *bufptr,c;
; count=2;       /* initalize byte count to 2 */
; *tbufptr++=SCONTOK;   /* put string constant token in buffer */
; bufptr=tbufptr++;   /* save value of tbufptr, advance to next byte, */
;                     /* and reserve a byte for string length */
; *tbufptr++=*ibufptr++;   /* put 1st quote in token buffer */
;
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
;
;
; while(((c=*ibufptr) != '"'))
; {
;  if(c==EOL)             /* if we hit EOL */
;   { errcode=MISQUERR; return; } /* flag error & return */
;  *tbufptr++=c;         /* if not, put next char in buffer */
;  ++ibufptr;            /* advance input buffer pointer */
;  ++count;              /* up byte count */
; }
;
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

;
; *tbufptr++=c;          /* put closing quote in token buffer */
; ++ibufptr;             /* advance input buffer pointer */
; *bufptr=count;         /* put string byte count in token buffer */
; return;
; *}
;
GETSCON2            equ       *
                    jsr       PUTTOK
GETSCON4            pulx
                    stb       ,x
                    rts

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "BASICLB4.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; */***** xon *****/
;
; *xon()
; *{
; int num;
; *tbufptr++=ONTOK;      /* put ON token in buffer */
; blanks();              /* skip any blanks */
; xexpres(NUM);          /* get the expression */
; if(errcode) return;    /* if error, return */
; blanks();              /* skip any blanks */
; if(match("GOTO")) xgoto();     /* check for GOTO */
; else if(match("GOSUB")) xgosub();      /* check for GOSUB */
; else errcode=IONSYERR;         /* if neither, flag an error */
; if(errcode) return;            /* if error, return */
; blanks();              /* skip blanks */
;
;
XON                 equ       *
;        JSR    BLANKS
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
;
;
; while(*ibufptr==',')   /* do until EOL */
; {
;  *tbufptr++=COMMATOK;  /* put COMMA token in buffer */
;  ++ibufptr;            /* advance the input buffer pointer */
;  blanks();             /* skip blanks */
;  *tbufptr++=LCONTOK;   /* put line number constant token in buffer */
;  num=getlinum();       /* get line number */
;  if(num==0) errcode=LINENERR;   /* if 0, line number error */
;  if(errcode) return;    /* if error, return */
;  putint(num);           /* put line number in buffer */
;  blanks();              /* skip blanks */
; }
; return;
; *}
;
                    jsr       CHKCOMA
                    bcs       XON6
                    rts

XON6                jsr       BLANKS
                    lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XON8                jsr       PUTDTOK
                    bra       XON5

;
;
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

;
;
XONPACC             equ       *
                    bsr       GETARG              ; GET AN ARGUMENT AND A COMMA.
                    bra       XONTIME             ; GO USE SOME OTHER CODE.

;
;
GETARG              lda       #NUM                ; GO GET THE "OPERATING MODE" EXPRESSION.
                    jsr       XEXPRES
                    jsr       BLANKS              ; SKIP BLANKS.
                    jsr       CHKCOMA             ; GO CHECK FOR COMMA.
                    bcc       XONTIME2            ; NO COMMA. REPORT ERROR.
                    jmp       BLANKS              ; SKIP BLANKS AFTER COMMA AND RETURN.

;
;
;
; */***** xif() *****/
;
; *xif()
; *{
; *int num;
; *tbufptr++=IFTOK;              /* put if token in the buffer */
; blanks();                      /* skip any blanks */
; ifwhflag=1;                    /* let xexpres() know we are doing an IF */
; xexpres(NULL);                 /* get relational expression */
; if(errcode) return;            /* if error, return */
; blanks();                      /* if not, skip blanks */
;
;
XIF                 equ       *
;        JSR    BLANKS
                    inc       IFWHFLAG
                    lda       #NUM
                    jsr       XEXPRES
XIF1                jsr       BLANKS
;
;
; if(match("THEN"))              /* check for "THEN" clause */
; {
;  *tbufptr++=THENTOK;           /* put THEN token in the buffer */
;  blanks();                     /* skip any blanks after "THEN" */
;
;
                    ldx       #THENS
                    jsr       STREQ
                    bcs       XIF2
                    lda       #MTHENERR
                    jmp       RPTERR

;
;
;  if(numeric(*ibufptr))         /* is a line number present after THEN? */
;  {                             /* yes */
;   *tbufptr++=LCONTOK;          /* put line # const. token in buffer */
;   num=getlinum();              /* get the line # */
;   if(num==0) errcode=LINENERR;
;   if(errcode) return;          /* if error, return */
;   putint(num);                 /* put number in buffer */
;  }
;  else                          /* not a line #, check for statement */
;  {
;   xlate();             /* try to make a statement out of what follows */
;   if(errcode) return;          /* if error, return */
;  }
; }
;
;
XIF2                equ       *
                    lda       #THENTOK
                    jsr       PUTTOK
                    jsr       BLANKS
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcc       XIF9
;
XIF3                lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XIF6                jsr       PUTDTOK
;
;
; else                   /* if "THEN" not present */
; {
;  errcode=MTHENERR;             /* flag a missing THEN error */
;  return;
; }
;
;
; blanks();                      /* skip any blanks after object of THEN */
; if(match("ELSE"))              /* is "ELSE" clause present? */
; {                              /* yes */
;  *tbufptr++=ELSETOK;           /* put ELSE token in the buffer */
;  blanks();                     /* skip any blanks after ELSE */
;
;
XIF4                equ       *
                    jsr       BLANKS
                    ldx       #ELSES
                    jsr       STREQ
                    bcs       XIF7
                    rts

XIF7                lda       #ELSETOK
                    jsr       PUTTOK
                    jsr       BLANKS
;
;
;  if(numeric(*ibufptr))         /* is a line # present after ELSE */
;  {                     /* yes */
;   *tbufptr++=LCONTOK;          /* put line # const. token in buffer */
;   num=getlinum();              /* get the line # */
;   if(num==0) errcode=LINENERR;
;   if(errcode) return;          /* if error, return */
;   putint(num);                 /* put number in buffer */
;  }
;  else          /* line # not present, try to xlate a statement */
;  {
;   xlate();
;  }
; }
;
;
                    jsr       GETCHR
                    jsr       NUMERIC
                    bcs       XIF8
XIF9                lda       #LINENERR
                    jmp       RPTERR

XIF8                lda       #LCONTOK
                    jsr       PUTTOK
                    jsr       GETLINUM
XIF10               jmp       PUTDTOK

;
; return;                /* in any case, return */
; *}
;
;
THENS               fcc       "THEN"
                    fcb       0
ELSES               fcc       "ELSE"
                    fcb       0
;
;
; */***** xfor() *****/
;
; *xfor()
; *{
; short type;
; *tbufptr++=FORTOK;             /* put for token in buffer */
; blanks();              /* skip blanks between FOR & assignment statement */
; type=getvar();         /* get variable */
; if((type!=NUM)|(*ibufptr++!='='))      /* is it a numerical variable? */
; { errcode=IVEXPERR; return; }  /* no. flag error & return */
;
;
XFOR                equ       *
;        JSR    BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       XFOR1
XFOR2               lda       #IVEXPERR
                    jmp       RPTERR

XFOR1               jsr       GETNXCHR
                    cmpa      #'='
                    bne       XFOR2
;
;
; *tbufptr++=EQUALTOK;   /* put equals token in buffer */
; xexpres(NUM);          /* go get a numerical expression */
; if(errcode) return;    /* if error, return */
; blanks();              /* skip blanks */
;
;
                    lda       #EQUALTOK
                    jsr       PUTTOK
                    lda       #NUM
                    jsr       XEXPRES
                    jsr       BLANKS
;
;
; if(match("TO"))        /* is TO present? */
; {                      /* yes */
;  *tbufptr++=TOTOK;     /* put TO token in buffer */
;  blanks();             /* skip blanks */
;  xexpres(NUM);         /* get the "TO" expression */
;  if(errcode) return;   /* return if error */
; }
; else                   /* "TO" not present */
; {
;  errcode=MTOERR;       /* set error flag & return */
;  return;
; }
;
;
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
;
;
; blanks();              /* skip blanks */
; if(match("STEP"))      /* is optional "STEP" clause present? */
; {                      /* yes */
;  *tbufptr++=STEPTOK;   /* put STEP token in buffer */
;  blanks();             /* skip blanks */
;  xexpres(NUM);         /* get expression */
; }
; return;
; *}
;
;
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

;
;
TO                  fcc       "TO"
                    fcb       0
STEP                fcc       "STEP"
                    fcb       0
;
;
; */***** xnext() *****/
;
; *xnext()
; *{
; *tbufptr++=NEXTTOK;    /* put NEXT token in buffer */
; blanks();              /* skip blanks */
; if(getvar()!=NUM) errcode=SYTXERR;     /* get variable, must be numeric */
; return;
; *}
;
;
XNEXT               equ       *
;        JSR    BLANKS
                    jsr       GETVAR
                    cmpa      #NUM
                    beq       XNEXT1
                    lda       #SYTXERR
                    jmp       RPTERR

XNEXT1              rts

;
;
; */***** xprint() *****/
;
; *xprint()
; *{
; *tbufptr++=PRINTTOK;   /* put PRINT token in buffer */
; blanks();             /* skip blanks */
;
;
XPRINT              equ       *
;        JSR    BLANKS
                    jsr       GETCHR
                    cmpa      #'#'                ; HAS AN ALTERNATE PORT BEEN SPECIFIED?
                    bne       XPRINT9             ; NO. GO PROCESS THE REST OF THE PRINT STATEMENT.
                    lda       #PNUMTOK            ; YES. PUT THE TOKEN INTO THE BUFFER.
                    jsr       PUTTOK              ; DO IT.
                    jsr       INCIBP              ; POINT PAST THE "#".
                    jsr       BLANKS              ; SKIP SPACES BETWEEN '#' AND EXPRESION.
                    bra       XPRINT7             ; GO GET EXPRESSION & CONTINUE.

;
;

; while((*ibufptr!=EOL)|(*ibufptr!=MIDEOL))    /* do until end of line */
; {
;  xexpres(NULL);        /* get expression */
;  if(errcode) return;   /* if error, return */
;  blanks();             /* skip blanks */
;  if(*ibufptr==COMMA) *tbufptr=COMMATOK;        /* check for comma */
;  else if(*ibufptr==SEMI) *tbufptr=SEMITOK;     /* check for semicolin */
;  else return;          /* if neither, return */
;  ++ibufptr;            /* advance input buffer pointer */
;  ++tbufptr;            /* advance token buffer pointer */
;  blanks();             /* skip blanks after delimeter */
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
                    bra       RPTERR

XPRINT6             lda       #SEMITOK
                    bsr       CHKCOMA2
                    bra       XPRINT9

;
;
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

;
;
; */***** xinput() *****/
;
; *xinput()
; *{
; *tbufptr++=INPUTTOK;           /* put INPUT token in buffer */
; blanks();                      /* skip blanks */
;
;
XINPUT              equ       *
;        JSR    BLANKS
                    bsr       XCHKDEV             ; GO CHECK FOR AN ALTERNATE DEVICE NUMBER.
;
;
; if(*ibufptr=='"')              /* is a prompt included? */
; {                      /* yes */
;  getscon();            /* get the string constant */
;  if(errcode) return;   /* if error, return */
;  if(*ibufptr!=COMMA) { errcode=SYTXERR; return; }
;  *tbufptr++=COMMATOK;
;  ++ibufptr;
; }
; inreadcm();            /* get the input variable list */
; return;
; *}
;
;
XINPUT2             jsr       BLANKS
                    jsr       GETCHR
                    cmpa      #'"'
                    bne       INREADCM
                    jsr       GETSCON
                    bsr       CHKCOMA             ; IF COMMA PRESENT, PUT IN TOKEN BUFFER.
                    bcs       INREADCM
XINPUT3             lda       #MCOMAERR
                    bra       RPTERR

;
;
; */***** inreadcm() *****/
;
; *inreadcm()
; *{
; while(1)               /* do forever */
; {
;  blanks();             /* skip blanks */
;  getvar();             /* get a variable */
;  if(errcode) return;   /* if error, return */
;  blanks();             /* skip blanks */
;  if(*ibufptr==COMMA)
;  {
;   *tbufptr++=COMMATOK;      /* put delimiter in buffer */
;   ++ibufptr;                /* and point to the next char in the buffer */
;  }
;  else return;          /* if no delimiter return */
; }
; *}
;
XDIM                equ       *
INREADCM            equ       *
XREAD               equ       *
;        JSR    BLANKS
                    jsr       GETVAR
XREAD1              jsr       BLANKS
                    bsr       CHKCOMA
                    bcs       XREAD
                    rts

;
;
;
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

;
;
;
XINBYTE             equ       *
                    bsr       XCHKDEV             ; GO CHECK FOR ALTERNATE DEVICE.
                    jsr       BLANKS              ; SKIP BLANKS AFTER COMMA.
                    jmp       GETVAR              ; GO TRY TO GET A VARIABLE.

;
;
;
;
; */***** xread *****/
;
; *xread()
; *{
; *tbufptr++=READTOK;    /* put read token in buffer */
; inreadcm();            /* get the variable list */
; return;
; *}
;
; */***** xrestore() *****/
;
; *xrestore()
; *{
; *tbufptr++=RESTRTOK;   /* put RESTORE token in buffer */
; return;
; *}
;
; */***** xwhile() *****/
;
; *xwhile()
; *{
; *tbufptr++=WHILETOK;   /* put WHILE token in buffer */
; blanks();              /* skip blanks */
; ifwhflag=1;            /* indicate we are going to get a WHILE expression */
; xexpres(NULL);         /* get expression */
; return;
; *}
;
;
XWHILE              equ       *
;        JSR    BLANKS
                    inc       IFWHFLAG
                    lda       #NULL
                    jmp       XEXPRES

;
;
; */***** xendwh() *****/
;
; *xendwh()
; *{
; *tbufptr++=ENDWHTOK;   /* put ENDWH token in buffer */
; return;
; *}
;
;
XPACC               equ       *
XTIME               equ       *
                    ldb       #NUM                ; SETUP TO USE CODE IN "ASIGNMT".
                    jmp       ASIGNMT1            ; GO DO ASSIGNMENT STATEMENT.

;
;
;
; */***** rpterr() *****/
;
; *rpterr()
; *{
; *char *ptr,c;
; ptr=inbuff;    /* point to start of input buffer */
; nl();
; nl();
; while((c=*ptr++)!=EOL) outbyte(c);     /* print the input buffer */
;
;
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

;
;
; nl();          /* go to next line */
; ptr=inbuff;    /* point to begining of input buffer */
; while(ptr++ < ibufptr-2) outbyte('*');    /* output '*' to point of error */
; pl("^^^");  /* point to error */
; nl();
;
;
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

;
;
; pl("ERROR # ");
; outdeci(errcode);
; return;
; *}
;
;
RPTERR5             ldx       #ERRORS
                    jsr       PL
                    ldb       ERRCODE
                    clra
                    bra       OUTDECI

;
;
ARROWS              fcc       "^^^"
                    fcb       0
ERRORS              fcc       "ERROR # "
                    fcb       0
;
;
RPTRERR             equ       *                   ; REPORT A RUN TIME ERROR.
                    sta       ERRCODE
                    bsr       RPTERR5
RPTRERR1            ldx       #INLINE
                    jsr       PL
                    ldd       CURLINE
                    bsr       OUTDECI
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
                    jmp       MAINW

;
;
BREAKS              fcc       "BREAK"
INLINE              fcc       " IN LINE # "
                    fcb       0
;
OUTDECI             equ       *
                    cpd       #0
                    bne       OUTDECI1
                    lda       #'0'
                    jmp       OUTBYTE

OUTDECI1            equ       *
                    pshy
;        PSHD                ; SAVE THE NUMBER TO PRINT.
                    pshb
                    psha
                    ldd       #10000              ; NUMBER TO START DIVIDING BY.
;        PSHD
                    pshb
                    psha
                    clrb                          ; SET INITAL VALUE OF LEADING ZERO SUPRESS FLAG.
                    pshb
                    tsy
                    ldd       3,y                 ; IS THE NUMBER NEGATIVE?
                    bpl       OUTDECI2            ; NO. GO PRINT THE NUMBER.
                    coma                          ; YES. MAKE THE NUMBER POSITIVE.
                    comb                          ; (TWO'S COMPLEMENT)
                    addd      #1
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


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "BASICLB5.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; */***** getfun() *****/
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
;
;
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

;
;
FUNCTBL             equ       *
FDIVS               fcc       "FDIV"
                    fcb       0
                    fcb       FDIVTOK
                    fdb       BNUMFUN
CHRS                fcc       "CHR$"
                    fcb       0
                    fcb       CHRTOK
                    fdb       UNUMFUN
ABS                 fcc       "ABS"
                    fcb       0
                    fcb       ABSTOK
                    fdb       UNUMFUN
RND                 fcc       "RND"
                    fcb       0
                    fcb       RNDTOK
                    fdb       UNUMFUN
SGN                 fcc       "SGN"
                    fcb       0
                    fcb       SGNTOK
                    fdb       UNUMFUN
TABS                fcc       "TAB"
                    fcb       0
                    fcb       TABTOK
                    fdb       UNUMFUN
ADCS                fcc       "ADC"
                    fcb       0
                    fcb       ADCTOK
                    fdb       UNUMFUN
CALL                fcc       "CALL"
                    fcb       0
                    fcb       CALLTOK
                    fdb       UNUMFUN
PEEK                fcc       "PEEK"
                    fcb       0
                    fcb       PEEKTOK
                    fdb       UNUMFUN
                    fcc       "EEP"
                    fcb       0
                    fcb       FEEPTOK
                    fdb       UNUMFUN
HEX2                fcc       "HEX2"
                    fcb       0
                    fcb       HEX2TOK
                    fdb       UNUMFUN
HEX                 fcc       "HEX"
                    fcb       0
                    fcb       HEXTOK
                    fdb       UNUMFUN
                    fcc       "PORT"
                    fcb       0
                    fcb       FPRTATOK
                    fdb       FINDPORT
                    fcc       "TIME"
                    fcb       0
                    fcb       FTIMETOK
                    fdb       XTIMEF
                    fcc       "PACC"
                    fcb       0
                    fcb       FPACCTOK
                    fdb       XPACCF
                    fcb       0                   ; END OF TABLE MARKER.
;
;
XPOKE               equ       *
                    ldx       TBUFPTR             ; GET TOKEN BUFFER POINTER.
                    dex                           ; DEC. TO COMPENSATE FOR PUTTOK DONE IN XLATE.
                    stx       TBUFPTR             ; SAVE NEW POINTER VALUE. FALL THROUGH TO BNUMFUN.
                    lda       ,x                  ; GET TOKEN BACK INTO THE A-REG.
;
;
; */***** xfdiv() *****/
;
; *xfdiv()
; *{
; short type[2];
; type[0]=type[1]=NUM;            /* both arguments must be type NUM */
; dofunct(FDIVTOK,2,type);
; return(NUM);
; *}
;
BNUMFUN             equ       *
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

;
;
; */***** xchrs *****/
;
; *xchrs()
; *{
; return(unumfun(CHRTOK));
; *}
;
; */***** xabs() *****/
;
; *xabs()
; *{
; return(unumfun(ABSTOK));
; *}
;
; */***** xrnd() *****/
;
; *xrnd()
; *{
; return(unumfun(RNDTOK));
; *}
;
; */***** xsgn() *****/
;
; *xsgn()
; *{
; return(unumfun(SGNTOK));
; *}
;
; */***** xtab() *****/
;
; *xtab()
; *{
; return(unumfun(TABTOK));
; *}
;
; */***** xadc() *****/
;
; *xadc()
; *{
; return(unumfun(ADCTOK));
; *}
;
; */***** xcall() *****/
;
; *xcall()
; *{
; return(unumfun(CALLTOK));
; *}
;
; */***** unumfun() *****/
;
; *unumfun(token)  /* common code for a uinary numerical function */
; *short token;
; *{
; short type[1];         /* setup argument 'type' array */
; type[0]=NUM;           /* set the 1st (only) argument type to NUM */
; dofunct(token,1,type); /* go do the function */
; return(NUM);           /* return the function type */
; *}
;
XEEP                equ       *                   ; PROGRAM A WORD OF EEPROM.
                    ldx       TBUFPTR             ; COMPENSATE FOR TOKEN PLACEMENT BU UNUMFUN
                    dex                           ; ROUTINE.
                    stx       TBUFPTR             ; SAVE POINTER.
                    lda       ,x                  ; GET TOKEN FROM BUFFER.
                    bsr       UNUMFUN             ; GO TREAT AS A UNIARY NUMERIC FUNCTION.
                    jmp       ASIGNMT1            ; GO USE ASSIGNMENT CODE FOR REST OF FUNCTION.

;
;
;
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

;
;
; */***** dofunct() *****/
;
; *dofunct(functok,nargs,type)
; *short functok,nargs,*type;
; *{
; *tbufptr++=functok;            /* put function token in buffer */
; if(*ibufptr!='(') { errcode=ILFSYERR; return; }
; *tbufptr++=OPARNTOK;           /* put open paren in token buffer */
; ++ibufptr;
;
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
;
;
; while(1)
; {
;  xexpres(*type++);             /* get the argument/expression */
;  if(errcode) return;           /* return if error */
;  if(--nargs==0) break;         /* if we have all the arguments, quit */
;  if(*ibufptr!=',')             /* if delimiter not present, return */
;   { errcode=ILFSYERR; return; }
;  *tbufptr++=COMMATOK;          /* if it is, put it in the token buffer */
;  ++ibufptr;                    /* point to the next character */
; }
;
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

;
;
; if(*ibufptr!=')')              /* must see closing paren */
; {
;  errcode=ILFSYERR;             /* if not, error */
;  return;
; }
; else                           /* saw closing paren */
; {
;  *tbufptr++=CPARNTOK;          /* put it in the token buffer */
;  ++ibufptr;                    /* advance input buffer pointer */
; }
; return;
; *}
;
DOFUNCT3            equ       *
                    jsr       GETCHR
                    cmpa      #')'
                    bne       DOFUNCT5
                    jsr       INCIBP
                    lda       #CPARNTOK
                    jmp       PUTTOK              ; PUT TOKEN IN BUFFER & RETURN.

;
;
;
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
;
;                            ; STEAL SOME CODE.
XPACCF              equ       *
XTIMEF              jsr       PUTTOK              ; PUT TOKEN IN BUFFER.
                    lda       #NUM                ; RETURN TYPE "NUM".
                    rts                           ; RETURN.

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "LEDITOR.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; */***** storlin() *****/
;
; *storlin()
; *{
; int *numptr,*linum;
; numptr=tknbuf;                 /* get int pointer into token buffer */
; if(*numptr>hiline)             /* line # larger than current hi line*/
; {
;  apendlin();                   /* append it to the end of the buffer */
;  hiline=*numptr;               /* make it the current hi line number */
;  return;
; }
;
;
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

;
;
; linum=findline(*numptr);       /* look for line # in the program buffer */
; if(*linum==*numptr)            /* is it the same line #? */
; {
;  repline(linum);               /* yes. replace it with the new line */
;  return;
; }
; insrtlin(linum);               /* no. insert the new line in the buffer */
; return;
; *}
;
;
STORLIN1            equ       *
                    bsr       FINDLINE
                    ldd       ,x
                    pshx
                    ldx       TKNBUFS
                    cpd       ,x
                    pulx
                    bne       INSRTLIN
                    jmp       REPLINE

;
;
; /***** delline() *****/
;
; *delline(num)            /* delete line from basic buffer */
; *int num;
; *{
; int *linum;
; char *ptr;
; if(num > hiline) return;       /* line number can't be there, return */
; linum=findline(num);           /* look for the requested line # */
;
;
DELLINE             equ       *
;        PSHD       ; SAVE THE LINE NUMBER TO DELETE.
                    pshb
                    psha
                    tsy                           ; POINT TO THE LINE NUMBER WE SAVED.
                    cpd       HILINE              ; IS IT HIGHER THAN THE HIGHEST LINE ENTERED SO FAR?
                    bls       DELLINE1            ; NO. GO SEE IF THE LINE EXISTS.
DELLINE2            lda       #1                  ; YES. THE LINE CANNOT EXIST.
                    sta       CONTFLAG
;        PULD       ; PULL THE LINE NUMBER OFF THE STACK.
                    pula
                    pulb
                    rts                           ; RETURN.

DELLINE1            bsr       FINDLINE            ; GO SEE IF THE LINE EXISTS.
; RETURN A POINTER TO A LINE NUMBER IN THE BASIC PROGRAM BUFFER.
;
;
; if(*linum!=num) return;        /* if the line # doesn't exist, return */
; ptr=linum;                     /* make the int pointer a char pointer */
; closespc(ptr[2],ptr);          /* go delete the line */
; if(num==hiline) hiline=findhiln();
; return;
; *}
;
;
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

;
;
; */***** closespc() *****/        /* close up space in the BASIC buffer */
;
; *closespc(bytes,ptr)
; *char bytes,*ptr;
; *{
; char *to,*from;                /* define the from/to pointers */
; to=ptr;                        /* set up destination pointer */
; from=ptr+bytes;                /* setup source pointer */
; while(from<basend)             /* while we're not at the end of the buff */
; { *to++=*from++; }             /* move source to destination */
; basend=to;                     /* set new basend pointer */
; return;
; *}
;
;
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
                    jsr       MOVEVARSDN          ; MOVE ALL THE VARIABLES DOWN.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

;
;
; */***** findline() *****/        /* return pointer to line number or next
;                                   highest line number */
; *findline(linenum)
; *int linenum;
; *{
; char *linelen;
; int *basbufp;
; basbufp=basbeg;                /* set pointer to start of basic buffer */
; while(*basbufp<linenum)        /* do until we find line # or one higher */
; {
;  linelen=basbufp;              /* convert int pointer to char pointer */
;  basbufp=linelen+linelen[2];   /* convert char ptr back to int pointer */
; }
; return(basbufp);               /* return the pointer */
; *}
;
;
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

;
;
; */***** findhiln() *****/
;
; *findhiln()                      /* find highest line number in basic buffer **/*{
; int *iptr,lnum;
; char *cptr;
; lnum=0;                        /* set line # to 0 */
; iptr=basbeg;                   /* set int pointer to basbeg */
; while(iptr!=basend)            /* we're not to the end of the basic buff */
; {
;  lnum=*iptr;                   /* get current line # */
;  cptr=iptr;                    /* convert int pointer to char pointer */
;  iptr=cptr+cptr[2];            /* add line length, point to next line */
; }
; return(lnum);                  /* return highest line number */
; *}
;
;
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

;
;
; */***** insrtlin() *****/
;
; *insrtlin(ptr)
; *char *ptr;
; *{
; openspc(tknbuf[2],ptr);        /* go open space in the program bufer */
; if(errcode) return;            /* return if out of memory */
; putline(ptr);                  /* put line into buffer */
; return;
; *}
;
;
INSRTLIN            equ       *
                    pshx
                    ldx       TKNBUFS
                    ldb       2,x
                    pulx
                    pshx
                    bsr       OPENSPC
                    pulx
                    bra       PUTLINE

;
;
; */***** openspc() *****/         /* open space in program buffer */
;
; *openspc(bytes,ptr)
; *char bytes,*ptr;
; *{
; char *to,*from;                /* declare to/from pointers */
; from=basend;                   /* set source at end of buffer */
; to=basend+bytes;               /* set destination "bytes" beyond source */
; if(to>basmend)                 /* if out of memory, return an error */
; { errcode=OMEMERR; return; }
; basend=to;                     /* set new end of buffer */
; while(from>=ptr)
; { *to--=*from--; }             /* open up area in buffer */
; return;
; *}
;
OPENSPC             equ       *
                    pshy
                    pshx
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       OPENSPC4
                    jsr       MOVEVARSUP
                    ldx       BASEND
                    pshx
                    abx
                    pshx
                    tsy
                    ldd       ,y
OPENSPC1            std       BASEND
OPENSPC3            ldd       2,y
                    cpd       4,y
                    blo       OPENSPC2
                    ldx       2,y
                    lda       ,x
                    dex
                    stx       2,y
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

;
;
; */***** putline() *****/         /* move line from token buffer to program
;                                   buffer */
; *putline(cptr)
; *char *cptr;
; *{
; short count;
; count=tknbuf[2];               /* get length of line in token buffer */
; tbufptr=tknbuf;                /* point to start of token buffer */
; while(count)
; {
;  *cptr++=*tbufptr++;           /* move a byte */
;  --count;                      /* decrement the byte count */
; }
; return;
; *}
;
;
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

;
;
; */***** apendlin() *****/        /* appent line to end of program buffer */
;
; *apendlin()
; *{
; if((basend+tknbuf[2])<=basmend)  /* do we have enough memory left? */
; {
;  putline(basend);              /* move the line */
;  basend+=tknbuf[2];            /* set the new end of basic pointer */
; }
; else errcode=OMEMERR;          /* not enough memory, error */
; return;
; *}
;
;
APENDLIN            equ       *
                    ldx       TKNBUFS
                    ldb       2,x
                    ldx       VAREND
                    abx
                    cpx       BASMEND
                    bhi       APENDLN1
;                   ldb       TKNBUF+2
                    bsr       MOVEVARSUP
                    ldx       BASEND
                    abx
                    xgdx
                    ldx       BASEND
                    std       BASEND
                    bra       PUTLINE

APENDLN1            lda       #OMEMERR
                    jmp       RPTERR

;
;
; */***** repline() *****/         /* replace line in buffer */
;
; *repline(ptr)
; *char *ptr;
; *{
; short lendif,temp1,temp2;
; temp1=*(ptr+2);                /* convert type from char to int */
; temp2=(tknbuf[2]);
; lendif=temp1-temp2;            /* get the difference in line length */
;      if(lendif==0)             /* if the same, just write over the old */
;      {
;       putline(ptr);
;      }
;
;
REPLINE             equ       *
                    ldb       2,x
                    pshx
                    ldx       TKNBUFS
                    subb      2,x
                    pulx
                    bne       REPLINE1
                    bra       PUTLINE

;
;
; else if(lendif<0)              /* if line in tknbuf is larger */
;      {
;       lendif=-lendif;          /* make it a positive number */
;       openspc(lendif,ptr);     /* tru to open up a space */
;       if(errcode) return;      /* if not enough memory, return */
;       putline(ptr);            /* if ok, copy line to program buffer */
;      }
;
;
REPLINE1            equ       *
                    bpl       REPLINE2
                    negb
                    pshx
                    jsr       OPENSPC
                    pulx
                    bra       PUTLINE

;
;
; else                           /* if line in tknbuf is smaller */
;      {
;       closespc(lendif,ptr);    /* close up excess space */
;       putline(ptr);            /* put new line in program buffer */
;      }
; return;
; *}
;
;
REPLINE2            equ       *
                    pshx
                    jsr       CLOSESPC
                    pulx
                    bra       PUTLINE

;
;
MOVEVARSUP
                    pshy                          ; SAVE THE Y REGISTER.
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

;
;
MOVEVARSDN
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

;
;
DecCount
                    ldd       VarSize             ; get the size of the variable table.
                    subd      #1                  ; decrement it.
                    std       VarSize             ; save the new value.
                    rts                           ; return.


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "COMMAND1.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

; */***** chckcmds() *****/
;
; *chckcmds()
; *{
; if(match("LIST")) clist();
; else if(match("RUN")) crun();
; else if(match("NEW")) cnew();
; else if(match("CONT")) ccont();
; else if(match("CLEAR")) cclear();
; else if(match("HELP")) chelp();
; else if(match("FERASE")) cflerr();
; else return(0);
; return(1);
; *}
;
;
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

;
;
CMDTBL              equ       *
                    fcc       "LIST"
                    fcb       0
                    fdb       CLIST
                    fcc       "RUN"
                    fcb       0
                    fdb       CRUN
                    fcc       "NEW"
                    fcb       0
                    fdb       CNEW
                    fcc       "CONT"
                    fcb       0
                    fdb       CCONT
                    fcc       "CLEAR"
                    fcb       0
                    fdb       CCLEAR
                    fcc       "HELP"
                    fcb       0
                    fdb       CHELP
                    fcc       "FERASE"
                    fcb       0
                    fdb       CFLERR
                    fcc       "ESAVE"
                    fcb       0
                    fdb       CESAVE
                    fcc       "ELOAD"
                    fcb       0
                    fdb       CELOAD
                    fcc       "LLIST"
                    fcb       0
                    fdb       CLLIST
                    fcc       "AUTOST"
                    fcb       0
                    fdb       CAUTOST
                    fcc       "NOAUTO"
                    fcb       0
                    fdb       CNOAUTO
                    fcc       "FREE"
                    fcb       0
                    fdb       CFREE
                    fcb       0                   ; END OF TABLE MARKER.


; */***** clist() *****/
;
; *clist()
; *{
; int *intptr;
; char token;
; if(basbeg==basend) return;             /* if basic buffer empty, return */
; skipspcs();                    /* skip any spaces after "LIST" */
;
;
CLIST               equ       *
                    jsr       NL2
                    ldd       BASBEG
                    cpd       BASEND
                    bne       CLIST1
                    rts

CLIST1              jsr       SKIPSPCS
;
;
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
;
;
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
;
;
;
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
;
;
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

CLIST10             bsr       LVARCON
                    bra       CLIST8

CLIST9              jsr       NL
                    ldx       TOKPTR
                    inx
                    stx       TOKPTR
                    bra       CLIST6

;
;
; */***** lvarcon() *****/
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
;
;
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
;
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

;
;
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
;
;
;
;
; */***** livar() *****/
;
; *livar()
; *{
; lfvar();
; outbyte('%');
; return;
; *}
;
;
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

;
;
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

;
;
; */***** licon() *****/
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
;
;
LICON               equ       *
                    ldd       TOKPTR
                    addd      #ISIZ+1
                    bra       LFCON2

;
;
; */***** lscon() *****/
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
;
;
LSCON               equ       *
                    ldd       TOKPTR
                    addd      #1
                    bra       LFCON2

;
;
; */***** llcon *****/
;
; *llcon()
; *{
; int *intptr;
; intptr=++tokptr;
; tokptr+=2;
; outdeci(*intptr);
; return;
; *}
;
;
LLCON               equ       *
                    ldx       TOKPTR
                    inx
                    ldd       ,x
                    inx
                    inx
                    stx       TOKPTR
                    jsr       OUTDECI
                    rts

;
;
; */***** lkeyword *****/
;
; *lkeyword()
; *{
; char *charptr,token;
; token=*tokptr++
;
;
LKEYWORD            equ       *
                    ldx       TOKPTR
                    lda       ,x
                    inx
                    stx       TOKPTR
                    cmpa      #MSCNTOK
                    bne       LKEYWRD3
                    bra       LMSPCS

LKEYWRD3            cmpa      #REMTOK
                    bne       LKEYWRD4
                    bra       LREMLINE

LKEYWRD4            cmpa      #DATATOK
                    bne       LKEYWRD5
                    bra       LDATALIN

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

;
;
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

;
;
LDATALIN            equ       *
                    ldx       #DATA
                    jsr       PL
                    bra       LREM3

;
;
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

;
;
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
                    fcb       CLSTOK
                    fdb       CLS
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
;
;
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
COLLINC             fcc       ":"
                    fcb       0
SEMIC               fcb       59                  ; ";"
                    fcb       0
COMMAC              fcc       ","
                    fcb       0
OPARN               fcc       "("
                    fcb       0
CPARN               fcc       ")"
                    fcb       0
SPACE               fcc       " "
                    fcb       0
PORTE               fcc       "PORTE"
                    fcb       0
POUNDSGN            fcc       "#"
                    fcb       0
;
;
CRUN                equ       *
                    jsr       NL2                 ; DO 2 CR/LF SEQUENCES.
                    jsr       RUNINIT             ; INITALIZE RUNTIME VARIABLES.
                    lda       #1                  ; SET THE RUN MODE FLAG.
                    sta       RUNFLAG
;
;        END OF POINTER INITIALIZATIONS
;
                    ldy       BASBEG              ; POINT TO THE START OF THE PROGRAM.
                    cpy       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       CRUN5               ; YES. GO RUN IT.
                    rts                           ; NO. RETURN.

;
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
                    ldb       2,y                 ; GET LENGTH OF LINE.
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
;
;
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

;
;
;
RSKIPSPC            lda       ,y                  ; GET A CHARACTER.
                    bmi       RSKIP2
                    cmpa      #SSCNTOK            ; IS IT A SINGLE SPACE?
                    beq       RSKIP1              ; YES. BUMP IP BY 1.
                    blo       RSKIP2
                    iny                           ; BUMP IP BY 2 FOR MULTIPLE SPACES.
RSKIP1              iny                           ; BUMP IP.
RSKIP2              rts                           ; RETURN.

;
;
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
                    fdb       RCLS
;
;
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

;
;
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

;
;
;
;
RUNINIT             bsr       CCLEAR              ; GO CLEAR ALL VARIABLE STORAGE.
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

;
;
CCONT               equ       *
                    jsr       NL2
                    tst       CONTFLAG
                    bne       CCONT1
                    ldy       IPSAVE
                    jmp       CRUN7

CCONT1              lda       #CNTCNERR
                    sta       ERRCODE
                    jmp       RPTERR5

;
;
CNEW                ldx       #PSSTART
                    lda       ASFLAG              ; GET THE AUTO START FLAG.
                    cmpa      #$55                ; IS IT SET?
                    bne       CNEW1               ; NO. GO INITIALIZE EVERYTHING.
                    lda       #$AA                ; YES. RESET (ERASE) IT.
                    ldy       #ASFLAG             ; autostart flag address
                    jsr       WRITE               ; erase auto-start flag
CNEW1               jsr       INITVARS            ; INITIALIZE EVERYTHING.
                    rts                           ; RETURN.

;
;
CCLEAR              bsr       RUNINIT1            ; GO INITALIZE ALL STACKS ETC.
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

;*************
; display help screen

CHELP               ldx       #CHELP_SCN
                    jmp       PL

;*************
; erase Flash program memory

CFLERR              psha
                    jsr       FL_RAM              ; check/copy to RAM
                    sei                           ; disable interrupts
                    lda       #%00000000          ; first sector of bank 0
                    jsr       FL_BE-FLRC_SA+FL2RAM  ; erase sector now
                    cli                           ; re-enable interrupts
                    pula
                    rts

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       Include  'COMMAND2.Asm'
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

;************
; save basic program to FLASH

; check to see if there is a program to save

CESAVE              ldd       BASBEG              ; GET POINTER TO THE START OF THE BASIC PROGRAM.
                    cpd       BASEND              ; IS THERE A PROGRAM IN MEMORY?
                    bne       CESAVE1             ; YES. GO SAVE IT.
                    rts                           ; NO. RETURN.

; check to see if program will fit in available program storage FLASH

CESAVE1             ldd       VAREND              ; end of program
                    subd      BASBEG              ; start of program
                    cpd       #PSSIZE
                    bls       CESAVE5             ; yes, save it
                    lda       #EETOSMAL           ; no, report error
                    jmp       RPTERR

; erase flash sector and save basic program

CESAVE5             jsr       FL_RAM              ; check/copy to RAM
                    sei                           ; disable interrupts
                    lda       #%00000000          ; first sector of bank 0
                    jsr       FL_BE-FLRC_SA+FL2RAM  ; erase sector now
                    cli                           ; re-enable interrupts
                    lda       #%00000000          ; first sector of bank 0
                    jsr       FL_EC               ; erase check
                    bcc       CESAVE6             ; yes, erased
                    lda       #FLERERR            ; no, report error
                    jmp       RPTERR

; save basic program info variables (10 bytes)

CESAVE6             ldy       #PSSTART            ; target, start of the program storage in FLASH
                    ldx       #BASBEG             ; source
                    ldb       #4                  ; byte count
                    stb       COUNT               ; "
CESAVE3             ldd       ,x                  ; get byte
                    subd      RAMSTART            ; ?? (offset = RAM base address)

                    jsr       WRITE               ; save 1st byte (A)
                    iny                           ; next target address
                    tba                           ; move B to A
                    jsr       WRITE               ; save 2nd byte (B)
                    iny                           ; next target address
                    inx                           ; next source address
                    inx                           ; "
                    dec       COUNT               ; counter -1
                    bne       CESAVE3             ; loop

                    ldd       ,x                  ; get byte
                    jsr       WRITE               ; save 1st byte
                    iny                           ; next target address
                    tba                           ; move B to A
                    jsr       WRITE               ; save 2nd byte

; loop to save program

                    ldy       #PSSTART            ; target address
                    xgdy                          ; \
                    addd      #SSTART             ; | add SSTART to Y
                    xgdy                          ; /
                    ldx       BASBEG              ; source address

CESAVE4             lda       ,x                  ; get byte
                    jsr       WRITE               ; save byte
                    iny
                    inx
                    cpx       VAREND              ; finished ?
                    bls       CESAVE4             ; no
CESAVE9             rts                           ; RETURN.

;************
;                               load program info variables

CELOAD              ldx       #PSSTART            ; point to the start of the program storage FLASH.
                    ldy       #BASBEG             ; point to the start of the program pointer storage area.
                    ldb       #4                  ; number of words to move.
                    stb       COUNT               ; save the count.
CELOAD3             ldd       ,x                  ; get the offset that was saved.
                    cpd       #$FFFF              ; check data
                    bne       CELOAD5             ; yes, program to load
                    lda       #$AA                ; \
                    ldy       #ASFLAG             ; | erase auto-start flag
                    jsr       WRITE               ; /
                    lda       #NOFLPROG           ; no, report error
                    jmp       RPTERR

CELOAD5             addd      RAMSTART            ; add the starting address of the RAM to it.
                    std       ,y                  ; save the resulting pointer
                    inx                           ; point to the next offset.
                    inx                           ; "
                    iny                           ; point to the next pointer in RAM
                    iny                           ; "
                    dec       COUNT               ; have we gotten all the pointers yet?
                    bne       CELOAD3             ; no. keep going.

                    ldd       ,x                  ; yes. get the high line number.
                    std       ,y                  ; save it in RAM.

;                               now load the actual program from EEPROM

                    ldx       #PSSTART            ; point to the start of the EEPROM
                    ldy       BASBEG              ; point to the start of the BASIC program buffer.
CELOAD4             lda       SSTART,x            ; get a byte of the program.
                    sta       ,y                  ; put it in the program buffer.
                    inx                           ; point to the next program byte
                    iny                           ; point to the next buffer location.
                    cpy       VAREND              ; have we finished loading the program.
                    bls       CELOAD4             ; no. keep loading.
                    sty       STRASTG             ; yes. initialize the array storage area.
                    rts                           ; RETURN.

;*************

CLLIST              lda       #$01                ; USE DEVICE #1 FOR HARD COPY LISTING.
                    sta       DEVNUM
                    jsr       CLIST               ; GO DO A STANDARD LIST COMMAND.
                    clr       DEVNUM
                    rts                           ; RETURN.

;*************
; set/clear the auto-start flag

CAUTOST             lda       #$55                ; set the auto-start flag
CAUTOST1            ldy       #ASFLAG             ; flag address
                    jsr       WRITE               ; save it
                    rts                           ; return

CNOAUTO             lda       #$AA                ; erase the auto-start flag
                    bra       CAUTOST1            ; now

;*************

AUTOLOAD            ldx       #PSSTART
                    ldd       #PSSTART
                    addd      #SSTART
                    std       BASBEG
                    ldd       #PSSTART
                    addd      SBASEND,x
                    addd      #SSTART
                    std       BASEND

                    ldd       SVAREND,x
                    subd      SVARBEG,x
                    addd      #RAMSTART
                    std       VAREND
                    ldd       #RAMSTART
                    std       VARBEGIN
                    xgdy
                    ldd       #PSSTART
                    addd      SVARBEG,x
                    xgdx
                    bra       CELOAD4

;*************
;
CFREE               jsr       NL2
                    ldd       VARMEND
                    subd      STRASTG
                    jsr       OUTDECI
                    jsr       NL
                    rts

;
;
CDUMP               equ       *
;                   jsr       NL2                 ; PRINT TWO BLANK LINES.
;                   clr       DNAME+2             ; ZERO THE LAST BYTE OF THE VARIABLE NAME 'ARRAY'
;                   ldx       VARBEGIN            ; POINT TO THE START OF THE VARIABLE TABLE.
;CDUMP2             lda       ,x                  ; GET AN ENTRY. IS IT THE END OF THE TABLE?
;                   bne       CDUMP3              ; YES. WE'RE DONE.
                    rts

;CDUMP3             lda       1,x                 ; NO. GET THE FIRST CHARACTER OF THE NAME.
;                   sta       DNAME
;                   lda       2,x
;                   sta       DNAME+1
;                   ldx       #DNAME
;                   jsr       PL
;                   lda       ,x                  ; GET THE VARIABLE TOKEN.
;                   cmpa      #IVARTOK            ; IS IT AN INTEGER?
;                   beq       CDUMP9              ; YES. DUMP ITS VALUE.
;                   cmpa      #IAVARTOK           ; NO. IS IT AN INTEGER ARRAY?
;                   bne       CDUMP99             ; NO.
;                   ldd       3,x                 ; YES. GET THE POINTER TO THE ARRAY STORAGE. HAS IT BEEN DIMENSIONED?
;                   bne       CDUMP5              ; YES. GO PRINT ALL THE VALUES.
;                   ldx       #UNDIM
;                   jsr       PL
;CDUMP6             ldb       #5
;                   abx
;                   bra       CDUMP2
;CDUMP5             pshx                          ; SAVE THE POINTER TO THE VARIABLE TABLE.
;                   xgdx                          ; POINT TO THE ARRAY STORAGE AREA.
;                   ldd       ,x                  ; GET THE MAXIMUM SUBSCRIPT.
;                   std       SUBMAX
;                   clra
;                   clrb
;                   std       SUBCNT
;CDUMP77            lda       #'('
;                   jsr       OUTBYTE
;                   ldd       SUBCNT
;                   jsr       OUTDECI
;                   ldx       #CPEQ
;                   jsr       PL
;                   inx
;                   inx
;                   ldd       ,x
;                   jsr       OUTDECI
;                   jsr       NL
;                   ldd       SUBCNT
;                   addd      #1
;                   cmpd      SUBMAX
;                   bhi       CDUMP88
;                   std       SUBCNT
;                   ldx       #DNAME
;                   jsr       PL
;                   bra       CDUMP77
;CDUMP88            pulx
;                   bra       CDUMP6
;CDUMP9             lda       #'='
;                   jsr       OUTBYTE
;                   ldd       3,x
;                   jsr       OUTDECI
;                   jsr       NL
;                   bra       CDUMP6
;
;UNDIM              fcb       '=[?]',0
;CPEQ               fcb       ')=',0


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "RUNTIME1.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

RREM                equ       *                   ; NON-EXECUTIBLE STATEMENT JUST SKIP IT.
RDATA               equ       *
                    ldb       ,y                  ; GET LENGTH OF REMARK OR DATA LINE.
                    aby                           ; POINT TO THE EOLTOK.
                    rts                           ; RETURN.

;
;
;
;
RLET                equ       *
                    lda       ,y                  ; GET VARIABLE FLAG.
                    bsr       RVARPTR             ; GET POINTER TO ASIGNMENT VARIABLE.
;        PSHD                SAVE POINTER TO VARIABLE.
                    pshb
                    psha
                    iny                           ; PUT IP PAST THE "=" TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET VALUE INTO D.
                    pulx                          ; POINT TO THE DICTIONARY ENTRY.
                    std       ,x                  ; STORE VALUE.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

;
;
RVARPTR             lda       ,y                  ; GET VARIABLE FLAG.
                    bita      #$02                ; IS IT A STRING VARIABLE?
                    bne       RVARPTR2            ; YES. GO GET POINTER FOR A STRING DESCRIPTOR.
                    bita      #$10                ; IS IT A NUMERIC ARRAY VARIABLE?
                    bne       RVARPTR1            ; YES. GO CALCULATE THE SUBSCRIPT.
RVARPTR3            ldd       1,y                 ; GET THE OFFSET TO THE DICTIONARY ENTRY.
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
;        PSHD                SAVE THE SUBSCRIPT VALUE.
                    pshb
                    psha
                    tsx                           ; POINT TO THE VALUES.
                    lsld                          ; MULT BY 2.
                    addd      ,x                  ; MULT BY 3.
                    ins                           ; GET RID OF SUBSCRIPT VALUE.
                    ins
                    tsx                           ; POINT TO BASE ADDRESS.
                    bra       RVARPTR4

;
;
;
;
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
                    cpd       1,y                 ; IS IT > THE LINE NUMBER WE ARE TO "GOTO"?
                    bhi       RGOTO3              ; YES. START THE SEARCH AT THE BEGINING.
RGOTO2              ldd       ,x                  ; GET THE NEXT LINE NUMBER INTO D.
                    cpd       1,y                 ; IS THIS THE CORRECT LINE?
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

;
;
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
                    bra       RGOTO               ; GO DO A "GOTO".

;
;
RRETURN             equ       *
                    ldx       GOSTACK             ; GET THE GOSUB STACK POINTER.
                    cpx       STGOSTK             ; IS THERE A RETURN ADDRESS ON THE GOSUB STACK?
                    bne       RRETURN1            ; YES. GO RETURN.
                    lda       #RWOGERR            ; NO. RETURN W/O GOSUB ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

RRETURN1            ldy       ,x                  ; GET THE RETURN ADDRESS IN THE IP.
                    inx                           ; REMOVE THE ADDRESS FROM THE STACK.
                    inx
                    stx       GOSTACK             ; SAVE THE STACK POINTER.
                    rts                           ; BACK TO THE MAIN INTERPRET LOOP.

;
;
RSTOP               equ       *
                    ldx       #STOPSTR
                    jsr       PL
                    ldd       CURLINE
                    jsr       OUTDECI
                    sty       IPSAVE
                    bra       REND1

REND                equ       *
                    jsr       NL
                    lda       #1
                    sta       CONTFLAG
REND1               ldd       #0
                    std       CURLINE
                    jmp       MAINW

;
;
STOPSTR             fcb       $0A,$0D
                    fcc       "STOPPED AT LINE # "
                    fcb       0
;
;
RWHILE              equ       *
                    ldx       WHSTACK             ; GET THE WHILE STACK POINTER.
                    dex                           ; POINT TO THE NEXT STACK LOCATION.
                    dex
                    cpx       EWHSTK              ; ARE WE AT THE END OF THE STACK?
                    bhs       RWHILE4             ; NO. GO STACK IT.
                    lda       #WHSOVERR           ; YES. WHILE STACK OVER FLOW.
                    jmp       RPTRERR             ; REPORT THE ERROR.

RWHILE4             stx       WHSTACK             ; SAVE THE WHILE STACK POINTER.
                    sty       ,x                  ; PUT IT ON THE STACK.
                    ldb       #$01                ; GET THE WHILE COUNT INTO B. (FOR NESTED WHILE'S)
RWHILE3             pshb
                    ldy       ADRNXLIN            ; GET THE ADDRESS OF THE NEXT LINE.
                    bne       RWHILE2
                    rts

RWHILE2             pshy                          ; SAVE THE IP.
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
                    bne       RWHILE1             ; NO. GO CHECK FOR ENDWH.
                    incb                          ; YES. UP THE NESTED WHILE COUNT.
RWHILE1             cmpa      #ENDWHTOK           ; IS IT THE END WHILE STATEMENT?
                    bne       RWHILE3             ; NO. GO LOOK AT THE NEXT LINE.
                    decb                          ; YES. IS IT THE CORRECT 'ENDWH'?
                    bne       RWHILE3             ; NO. LOOK FOR ANOTHER ONE.
                    jmp       RGOTO8              ; BACK TO INTERPRET LOOP.

;
;
;
;
RENDWH              equ       *
                    ldx       WHSTACK             ; GET THE WHILE STACK POINTER.
                    cpx       STWHSTK             ; HAS A WHILE STATEMENT BEEN EXECUTED?
                    bne       RENDWH1             ; YES. GO GET THE ADDRESS OF THE WHILE STATEMENT.
                    lda       #ENDWHERR           ; NO. GET ENDWHILE ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

RENDWH1             pshy                          ; SAVE THE IP IN CASE THE WHILE TEST FAILS.
                    ldy       ,x                  ; GET THE IP POINTER TO THE WHILE EXPRESSION.
                    jsr       DONEXP              ; YES. GO EVALUATE A NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET RESULT OFF NUMERIC STACK. IS IT TRUE?
                    bne       RENDWH3             ; YES. GO EXECUTE CODE BETWEEN WHILE & ENDWH.
                    puly                          ; NO. GET THE ADDRESS OF THE NEXT LINE/STATEMENT.
                    ldx       WHSTACK             ; GET WHILE STACK POINTER.
                    inx                           ; TAKE ADDRESS OFF OF WHILE STACK.
                    inx
                    stx       WHSTACK             ; SAVE STACK POINTER.
                    bra       RENDWH5             ; GO TO INTERPRET LOOP.

RENDWH3             ins                           ; REMOVE POINTER TO STATEMENT AFTER "ENDWH"
                    ins                           ; FROM STACK.
RENDWH5             rts                           ; GO EXECUTE LINES TILL "ENDWH".

;
;
RON                 equ       *
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

;
;
RPOKE               equ       *
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

;
;
RPORTA              ldb       #PORTAIO
RPORTA1             ldx       IOBASEV             ; GET ADDRESS OF PORTA I/O REGISTER.
                    abx
                    pshx                          ; SAVE POINTER TO VARIABLE.
                    iny                           ; PUT IP PAST THE "=" TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET VALUE INTO D.
                    tsta                          ; IS THE VALUE <0 AND >255?
                    beq       RPORTA2             ; NO. GO PUT THE VALUE IN THE PORT.
                    lda       #PRTASERR           ; YES. ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

RPORTA2             pulx                          ; POINT TO THE DICTIONARY ENTRY.
                    stb       ,x                  ; STORE VALUE.
                    rts                           ; BACK TO MAIN INTERPRET LOOP.

;
;
RPORTB              ldb       #PORTBIO            ; GET ADDRESS OF PORTB I/O REGISTER.
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.

;
;
RPORTC              ldb       #PORTCIO            ; GET ADDRESS OF PORTC I/O REGISTER.
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.

;
;
RPORTD              ldb       #PORTDIO            ; GET ADDRESS OF PORTD I/O REGISTER.
                    bra       RPORTA1             ; GO DO AN ASIGNMENT.


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "RUNTIME2.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

RTRON               equ       *
                    lda       #$FF                ; SET FLAG TO TURN TRACE MODE ON.
                    sta       TRFLAG              ; PUT IT IN THE FLAG BYTE.
RTRON1              rts                           ; BACK TO THE INTERPRET LOOP.

;
;
RTROFF              equ       *
                    clr       TRFLAG              ; TURN THE TRACE MODE OFF.
                    rts                           ; BACK TO THE INTERPRET LOOP.

;
;
RSLEEP              equ       *
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

;
;
RPRINT              equ       *
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
                    addb      1,y                 ; ADD IN LENGTH BYTE.
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
                    lda       1,y                 ; GET THE FUNCTION TYPE.
                    cmpa      #TABTOK             ; IS IT A TAB?
                    bne       RPRINT11            ; NO GO CHECK FOR "CHR$".
                    bsr       RTAB                ; GO DO TAB.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT11            cmpa      #CHRTOK             ; IS IT THE CHR$ FUNCTION.
                    bne       RPRINT12            ; NO. GO CHECK FOR HEX().
                    bsr       RCHRS               ; YES. GO DO CHR$.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT12            cmpa      #HEXTOK             ; IS IT THE HEX() FUNCTION?
                    bne       RPRINT10            ; NO. GO DO A NUMERIC EXPRESSION.
                    bsr       RHEX                ; YES. GO PRINT THE NUMBER AS HEX.
                    bra       RPRINT4             ; GO SEE IF THERE'S MORE TO PRINT.

RPRINT10            cmpa      #HEX2TOK            ; IS IT THE HEX2() FUNCTION?
                    bne       RPRINT14            ; NO. GO DO A NUMERIC EXPRESSION.
                    bsr       RHEX2               ; YES GO PRINT A NUMBER >=255 AS 2 HEX BYTES.
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
                    cmpa      #SEMITOK            ; IS IT A SEMICOLIN?
                    bne       RPRINT6             ; NO. MUST BE AN EOLTOK.
                    iny                           ; DO NOTHING BUT BUMP THE IP.
                    bra       RPRINT7             ; GO CHECK FOR EOL AFTER COMMA OR SEMICOLIN.

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

;
;
RTAB                equ       *
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       RTAB1               ; YES. GO DO TAB.
                    lda       #TABARGER           ; NO. ERROR.
RTAB3               jmp       RPTRERR             ; REPORT ERROR.

RTAB1               cmpb      PRINTPOS            ; ARE WE ALREADY PAST THE "TAB" POSITION?
                    bls       RTAB2               ; YES. DONE.
                    lda       #SPC                ; GET A SPACE.
                    jsr       OUTBYTE             ; PRINT IT.
                    bra       RTAB1

RTAB2               rts                           ; RETURN.

;
;
RCHRS               equ       *
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       RCHRS1              ; YES. GO DO TAB.
                    lda       #CHRARGER           ; NO. ERROR.
                    bra       RTAB3               ; REPORT ERROR.

RCHRS1              tba                           ; PUT BYTE INTO A
                    jmp       OUTBYTE             ; PRINT THE BYTE & RETURN.

;
;
RHEX2               equ       *
                    bsr       PFUNCOM             ; GO GET ARG. & CHECK MAGNITUDE. IS ARG. OK?
                    beq       RHEX1               ; YES. GO PRINT 2 HEX CHARACTERS & RETURN.
                    lda       #HEX2AERR           ; NO. ARG. MUST BE >=0 & <=255.
                    bra       RTAB3               ; GO REPORT ERROR.

;
;
RHEX                equ       *
                    bsr       PFUNCOM             ; GO DO COMMON CODE FOR PRINT FUNCTIONS
                    bsr       PRNT2HEX            ; GO PRINT 2 HEX CHARACTERS.
RHEX1               tba                           ; PUT LOWER BYTE IN A.
;              ; FALL THRU TO PRINT 2 HEX CHARACTERS & RETURN.
;
;
PRNT2HEX            equ       *
                    psha                          ; SAVE THE CHARACTER.
                    bsr       PRNTHXL             ; PRINT THE LEFT HEX NYBBLE.
                    pula                          ; GET BYTE BACK.
                    bra       PRNTHXR             ; PRINT RIGHT NYBBLE & RETURN.

;
;
PRNTHXL             lsra                          ; GET UPPER NYBBLE INTO LOWER ONE.
                    lsra
                    lsra
                    lsra
PRNTHXR             anda      #$0F                ; MASK OFF UPPER NYBBLE.
                    adda      #$30                ; MAKE IT A HEX NUMBER.
                    cmpa      #$39                ; IS IT?
                    bls       PRNTHXR1            ; YES. PRINT IT.
                    adda      #$07                ; NO. MAKE IT A HEX LETTER.
PRNTHXR1            jmp       OUTBYTE             ; PRINT IT & RETURN.

;
;
PFUNCOM             equ       *
                    ldb       #3                  ; POINT PAST FUNCTION FLAG, FUNCTION TOKEN, &
                    aby                           ; OPEN PAREN.
                    jsr       DONEXP              ; GO GET POSITION TO TAB TO.
                    iny                           ; BUMP IP PAST CLOSING PAREN.
                    jsr       PULNUM              ; GET OPERAND OFF STACK.
                    tsta                          ; CHECK THAT OPERAND IS >0 & <=255 FOR FUNCTIONS
;              ; THAT REQUIRE IT.
                    rts                           ; RETURN.

;
;
;
RDIM                equ       *
                    lda       ,y                  ; GET VARIABLE FLAG/TYPE.
                    bita      #$10                ; IS IT A SUBSCRIPTED VARIABLE?
                    bne       RDIM1               ; YES. GO DIMENSION IT.
                    lda       #NOSUBERR           ; NO. GET ERROR.
RDIM3               jmp       RPTRERR             ; GO REPORT THE ERROR.

RDIM1               ldd       1,y                 ; GET THE OFFSET INTO THE DICTIONARY.
                    addd      VARBEGIN            ; ADD IN THE START OF THE DICTIONARY.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    ldd       3,x                 ; GET THE POINTER TO THE STORAGE. BEEN DIMENSIONED?
                    beq       RDIM2               ; NO. GO DIMENSION IT.
                    lda       #REDIMERR           ; YES. ERROR.
                    bra       RDIM3

RDIM2               pshx                          ; SAVE THE POINTER TO THE DICTIONARY.
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
                    bpl       RDIM8               ; ONLY POSITIVE SUBSCRIPTS ALLOWED.
                    lda       #NEGSUBER           ; NEGATIVE NUMBER.
                    bra       RDIM9               ; REPORT ERROR.

RDIM8               pshx
                    ldx       3,x                 ; GET POINTER TO STORAGE.
                    std       ,x                  ; PUT MAX SUBSCRIPT IN POOL STORAGE.
                    addd      #1                  ; COMPENSATE FOR "0" SUBSCRIPT.
                    pulx                          ; RESTORE POINTER TO DICTIONARY ENTRY.
                    lsld                          ; MULT. BY 2 (2 BYTES/INTEGER).
                    addd      STRASTG             ; ADD IN CURRENT POINTER TO POOL.
                    cpd       STRASTG             ; WAS THE SUBSCRIPT SO BIG WE WRAPPED AROUND?
                    bls       RDIM4               ; YES. ERROR.
                    cpd       VARMEND             ; DO WE HAVE ENOUGH MEMORY?
                    bls       RDIM5               ; YES.
RDIM4               lda       #OMEMERR            ; NO. ERROR.
RDIM9               jmp       RPTRERR             ; GO REPORT THE ERROR.

RDIM5               std       STRASTG             ; SAVE POINTER.
                    ldx       3,x                 ; POINT TO START OF STORAGE.
                    inx
                    inx                           ; POINT PAST THE SUBSCRIPT LIMIT.
RDIM6               clr       ,x                  ; CLEAR THE STORAGE.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       STRASTG             ; ARE WE DONE?
                    bne       RDIM6               ; NO. KEEP GOING.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET THE NEXT CHARACTER.
                    cmpa      #EOLTOK             ; ARE WE AT THE END OF THE LINE.
                    beq       RDIM7               ; YES.
                    iny                           ; BUMP IP PAST THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    bra       RDIM                ; DO DIMENSION THE NEXT VARIABLE.

RDIM7               rts                           ; BACK TO MAIN INTERPRET LOOP.


;*******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "RUNTIME3.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

RFOR                equ       *
                    ldd       FORSTACK            ; GET FOR STACK POINTER.
                    subd      #10                 ; ALLOCATE NEW FOR-NEXT DESCRIPTOR BLOCK.
                    cpd       EFORSTK             ; HAVE WE RUN OUT OF FOR-NEXT STACK SPACE?
                    bhs       RFOR1               ; NO. CONTINUE.
                    lda       #FORNXERR           ; YES. ERROR.
                    jmp       RPTRERR             ; REPORT ERROR.

RFOR1               std       FORSTACK            ; SAVE NEW STACK POINTER.
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
RFOR3               std       2,x                 ; PUT IT IN THE STACK.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #STEPTOK            ; IS THE STEP CLAUSE PRESENT?
                    beq       RFOR2               ; YES. GO GET THE "STEP" VALUE.
                    sty       6,x                 ; PUT TERMINATING CHARACTER OF "FOR" STATEMENT ON.
                    rts                           ; EXECUTE NEXT STATEMENT.

RFOR2               iny                           ; SKIP PAST THE "STEP" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; GO CALCULATE THE "STEP" VALUE.
                    jsr       PULNUM              ; GET VALUE OFF OPERAND STACK.
                    ldx       FORSTACK            ; GET POINTER TO FOR STACK.
                    bra       RFOR3               ; GO PUT VALUE IN STACK.

;
;
RNEXT               equ       *
                    jsr       RVARPTR             ; GET POINTER TO LOOP INDEX VARIABLE.
                    ldx       FORSTACK            ; GET "FOR" STACK POINTER.
                    cpd       ,x                  ; IS THE LOOP VARIABLE THE SAME?
                    beq       RNEXT1              ; YES. CONTINUE.
                    lda       #MFRNXERR           ; NO. ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

RNEXT1              pshy                          ; SAVE IP.
                    ldy       ,x                  ; GET POINTER TO CONTROL VARIABLE.
                    ldd       ,y                  ; GET CONTROL VARIABLE VALUE.
                    addd      2,x                 ; ADD THE STEP VALUE TO IT.
                    bvs       RNEXT4              ; ON OVERFLOW, EXIT (bug fix)
                    std       ,y                  ; SAVE THE RESULT.
                    tst       2,x                 ; IS THE STEP VALUE NEGATIVE?
                    bmi       RNEXT2              ; YES. GO DO TEST.
                    cpd       4,x                 ; NO. ARE WE DONE?
                    ble       RNEXT3              ; NO. GO DO THE LOOP AGAIN.
RNEXT4              puly                          ; RESTORE THE CURRENT IP.
                    xgdx                          ; PUT "FOR - NEXT" STACK POINTER IN D.
                    addd      #10                 ; REMOVE DESCRIPTOR FROM STACK.
                    std       FORSTACK            ; SAVE NEW STACK VALUE.
                    jmp       RSKIPSPC            ; SKIP SPACES AFTER CONTROL VARIABLE.

RNEXT2              cpd       4,x                 ; ARE WE DONE?
                    blt       RNEXT4              ; YES. CONTINUE.
RNEXT3              puly                          ; CLEAN Y OFF OF STACK.
                    ldy       6,x                 ; GET NEW IP.
                    ldd       8,x                 ; GET LINE NUMBER OF FOR STATEMENT.
                    std       CURLINE             ; MAKE IT THE CURRENT LINE.
                    rts

;
;
;
;
RINPUT              equ       *
                    bsr       CHCKDEV             ; CHECK FOR ALTERNATE INPUT DEVICE.
                    lda       ,y                  ; GET A TOKEN.
                    cmpa      #SCONTOK            ; IS THERE A PROMPT TO PRINT?
                    bne       RINPUT1             ; NO JUST GO GET THE DATA.
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
                    bra       RINPUT6

RINPUT1             jsr       NL
RINPUT6             equ       *
                    ldx       #QSP                ; POINT TO PROMPT.
                    jsr       PL                  ; PRINT IT.
                    jsr       GETLINE             ; GET THE DATA IN THE INPUT BUFFER.
                    bsr       RINRDC
                    bcs       RINPUT1
                    jsr       NL
                    clr       DEVNUM              ; SET DEVICE NUMBER BACK TO 0.
                    rts

;
;
QSP                 fcc       "? "
                    fcb       0
;
;
CHCKDEV             lda       ,y                  ; GET A TOKEN.
                    cmpa      #PNUMTOK            ; IS AN ALTERNATE DEVICE SPECIFYED?
                    beq       CHCKDEV1            ; YES. CONTINUE.
                    rts                           ; NO. RETURN.

CHCKDEV1            iny                           ; YES. PASS THE '#' TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    jsr       DONEXP              ; GO EVALUATE THE NUMERIC EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    bpl       CHCKDEV2            ; NEGATIVE NUMBERS NOT ALLOWED.
CHCKDEV3            lda       #ILLIOERR           ; REPORT THE ERROR.
                    jmp       RPTRERR

CHCKDEV2            cpd       #$0007              ; IS IT LARGER THAN 7?
                    bhi       CHCKDEV3
                    stb       DEVNUM              ; MAKE IT THE NEW DEVICE NUMBER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    cmpa      #EOLTOK             ; IF THIS IS A PRINT STATEMENT, IS IT EOL?
                    beq       CHCKDEV4            ; YES. DON'T BUMP THE IP.
                    iny                           ; BYPASS THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
CHCKDEV4            rts                           ; RETURN.

;
;
;
RINRDC              jsr       SKIPSPCS
                    cmpa      #EOL
                    bne       RINRDC1
                    sec
                    rts

RINRDC1             bsr       INNUMD
                    jsr       RSKIPSPC
                    lda       ,y
                    cmpa      #EOLTOK
                    beq       RINRDC2
                    cmpa      #MEOLTOK
                    beq       RINRDC2
                    iny                           ; BUMP PAST THE COMMA.
                    jsr       RSKIPSPC
                    bra       RINRDC

RINRDC2             clc
                    rts

;
;
INNUMD              equ       *
                    cmpa      #'$'
                    bne       INNUM2
                    jsr       INCIBP
                    jsr       GETHEX
                    bra       INNUM3

INNUM2              bsr       INDECI
INNUM3              equ       *
;        PSHD
                    pshb
                    psha
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
                    pula
                    pulb
                    std       ,x
                    rts

;
OUTSTR              equ       *
                    tstb
                    beq       OUTSTR2
OUTSTR1             lda       ,x
                    inx
                    jsr       OUTBYTE
                    decb
                    bne       OUTSTR1
OUTSTR2             rts

;
;
INDECI              equ       *
                    jsr       GETCHR              ; GET A CHARACTER.
                    cmpa      #'-'                ; IS IT A NEGATIVE NUMBER?
                    bne       INDECI1             ; NO. GO GET POSITIVE NUMBER.
                    jsr       INCIBP              ; YES. BUMP INPUT BUFFER PAST IT.
                    jsr       GETDECI             ; GET THE NUMBER.
                    coma                          ; NEGATE IT.
                    comb
                    addd      #1
                    rts                           ; RETURN.

INDECI1             jsr       GETDECI
                    rts

;
;
RREAD               equ       *
                    ldx       DATAPTR             ; GET POINTER TO DATA. IS IT POINTING TO DATA?
                    bne       RREAD1              ; YES. CONTINUE TO READ DATA.
                    bsr       RRESTOR             ; NO. GO GET POINTER TO FIRST DATA STATEMENT.
                    ldx       DATAPTR             ; GET POINTER TO DATA.
RREAD1              stx       IBUFPTR             ; PUT IT IN THE INPUT BUFFER POINTER.
                    bsr       RINRDC              ; GO USE INPUT/READ COMMON CODE.
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

;
;
RRESTOR             equ       *
                    pshy                          ; SAVE Y.
                    ldy       BASBEG              ; START SEARCH FOR "DATA" STATEMENTS AT THE BEGIN.
RESTOR2             pshy                          ; SAVE POINTER TO THIS LINE.
                    ldb       2,y                 ; GET LINE LENGTH.
                    aby                           ; GET START OF NEXT LINE.
                    sty       DATAPTR             ; SAVE IN "DATAPTR".
                    puly                          ; RESTORE POINTER.
                    ldb       #3
                    aby                           ; POINT TO FIRST TOKEN IN LINE.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    lda       ,y                  ; GET THE KEYWORD.
                    cmpa      #DATATOK            ; IS IT A DATA LINE?
                    beq       RESTOR1             ; YES. GO SET UP POINTER.
                    ldy       DATAPTR             ; GET ADDRESS OF NEXT LINE.
RESTOR3             cpy       BASEND              ; ARE WE AT THE END OF THE PROGRAM?
                    bne       RESTOR2             ; NO. KEEP LOOKING.
                    lda       #ODRDERR            ; OUT OF DATA ERROR.
                    jmp       RPTRERR             ; REPORT THE ERROR.

RESTOR1             iny                           ; POINT PAST DATA TOKEN & THE DATA LENGTH.
                    iny
                    sty       DATAPTR             ; SAVE POINTER TO DATA.
                    puly                          ; RESTORE Y.
                    rts                           ; RETURN.

;
;
RESTOR4             pshy                          ; CALL TO COMPENSATE FOR PULL OF Y ON RETURN.
                    bra       RESTOR3

;
;
RIF                 equ       *
                    jsr       DONEXP              ; GO DO A NUMERIC EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; SKIP PAST "THEN" TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THEN.
                    jsr       PULNUM              ; GET RESULT OF EXPRESSION FROM OPERAND STACK.
                    beq       RIF1                ; NOT TRUE. SEE IF ELSE CLAUSE PRESENT.
RIF3                jmp       RGOTO               ; RESULT WAS TRUE. GOTO PROPER LINE NUMBER.

RIF1                ldb       #3                  ; BUMP IP PAST LINE NUMBER.
                    aby
                    jsr       RSKIPSPC            ; SKIP SPACES IF PRESENT.
                    lda       ,y                  ; GET NEXT TOKEN.
                    cmpa      #ELSETOK            ; IS IT THE "ELSE" CLAUSE.
                    bne       RIF2                ; NO RETURN.
                    iny                           ; PASS ELSE TOKEN.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    bra       RIF3                ; DO A GOTO.

RIF2                rts                           ; RETURN.

;*************
; program a word of EEPROM

REEP                iny                           ; PASS UP THE OPEN PAREN.
                    jsr       RSKIPSPC            ; PASS UP ANY SPACES.
                    jsr       DONEXP              ; GO GET THE "SUBSCRIPT" OF THE EEPROM LOCATION.
                    iny                           ; PASS UP THE CLOSING PAREN.
                    iny                           ; PASS UP THE EQUALS TOKEN.
                    jsr       DONEXP              ; GET VALUE TO FROGRAM INTO EEPROM.
                    pshy                          ; SAVE THE Y REG.
                    ldy       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    ldd       2,y                 ; GET THE SUBSCRIPT FOR THE EEPROM LOCATION.
                    bmi       REEP1               ; NEGATIVE SUBSCRIPTS NOT ALLOWED.
                    cpd       #MAXEESUB           ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       REEP2               ; YES. CONTINUE.
REEP1               lda       #EESUBERR           ; EEPROM SUBSCRIPT ERROR.
                    jmp       RPTRERR             ; REPORT IT.

REEP2               lsld                          ; MULT THE SUBSCRIPT BY 2.
                    addd      #EEPBASAD           ; ADD IN THE EEPROM BASE ADDRESS.
                    xgdx                          ; PUT THE ADDRESS INTO X.
                    lda       ,x                  ; GET THE MOST SIGNIFIGANT BYTE OF THE CURRENT NUM.
                    cmpa      #$FF                ; DOES IT NEED ERASING?
                    beq       REEP3               ; NO. SEE IF NEXT BYTE NEEDS ERASING.
                    bsr       ERASEBYT            ; YES. GO ERASE IT.
REEP3               inx                           ; POINT TO NEXT BYTE.
                    lda       ,x                  ; GET NEXT BYTE.
                    cmpa      #$FF                ; DOES THIS BYTE NEED TO BE ERASED?
                    beq       REEP4               ; NO. GO WRITE DATA TO EEPROM.
                    bsr       ERASEBYT            ; YES. GO ERASE THE BYTE.
REEP4               lda       1,y                 ; GET LS BYTE OF WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    dex                           ; POINT TO THE MOST SIGNIFIGANT EEPROM LOCATION.
                    lda       ,y                  ; GET THE MS BYTE OF THE WORD.
                    bsr       PROGBYTE            ; GO PROGRAM THE BYTE.
                    puly                          ; RESTORE Y.
                    jsr       PULNUM              ; FIX UP NUM STACK.
                    jsr       PULNUM
                    rts                           ; RETURN.

;*************
;

ERASEBYT            equ       *
                    pshy
                    ldy       IOBASEV             ; Point to the base address of the I/O Registers.
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

;*************
; write data A to address Y

; PROGBYTE       XGDY                            ; swap X & Y
; XGDX                            ;  "
; JSR     WRITE                   ; write A to Y
; XGDX                            ; swap X & Y
; XGDY                            ;  "
; RTS                             ; return

;*************
;

PROGBYTE            equ       *
                    pshy
                    ldy       IOBASEV             ; Point to the base address of the I/O Registers.
PROGBYT2            ldb       #$02                ; SET UP NORMAL PROGRAMING MODE, ADDRESS/DATA
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
                    bne       PROGBYT2            ; NO. TRY AGAIN.
                    puly                          ; Restore Y.
                    rts                           ; YES. RETURN.

;
;
DLY10MS             equ       *
                    pshx                          ; SAVE X.
                    ldx       #3330               ; GET DELAY CONSTANT.
DLY10MS1            dex                           ; DECREMENT THE COUNT. DONE?
                    bne       DLY10MS1            ; NO. DELAY SOME MORE.
                    pulx                          ; RESTORE X.
                    rts                           ; RETURN.

;
;
RINBYTE             equ       *
                    jsr       CHCKDEV             ; GO CHECK FOR AN ALTERNATE DEVICE DESIGNATION.
                    jsr       RVARPTR             ; GO GET POINTER TO THE BYTE INPUT VARIABLE.
                    xgdx                          ; PUT THE POINTER INTO X.
                    jsr       INBYTE              ; GO GET A BYTE FROM THE SPECIFIED INPUT DEVICE.
                    tab                           ; PUT THE BYTE IN THE L.S.BYTE.
                    clra                          ; ZERO THE UPPER BYTE.
                    std       ,x                  ; PUT IT IN THE VARIABLE.
                    clr       DEVNUM              ; RESET TO DEVICE #0.
                    rts                           ; RETURN.

;
;
RTIME               equ       *
                    iny                           ; POINT PAST THE EQUALS TOKEN.
                    jsr       DONEXP              ; GO EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    rts                           ; RETURN.

;*************
; CLS command

RCLS                ldx       #CLSSTR
                    jmp       PL

CLSSTR              fcc       ESC,"[2J"           ; CLR SCREEN
                    fcc       ESC,"[0;0H"         ; TOP LEFT CORNER FCC "BASIC11 V1.55"
                    fcb       $00

;*************
;
RRTIME              equ       *
                    sei                           ; disable interrupts.
                    lda       #SWPRE+1            ; ADD 1 TO NORMAL PRE SCALER.
                    sta       TIMEPRE             ; SET UP THE SOFTWARE PRESCALER.
                    ldx       IOBASEV             ; Point to the I/O Base Address.
                    ldd       TCNT,x              ; get the current value of the timer counter.
                    jsr       TIMINTS3            ; go initialize the TOC using the timer interrupt code.
                    clra
                    clrb
                    std       TIMEREG             ; PUT IT IN THE TIME REGISTER.
                    cli
                    rts                           ; RETURN.

;
;
RPACC               equ       *
                    iny                           ; POINT PAST EQUALS TOKEN.
                    jsr       DONEXP              ; EVALUATE THE EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    tsta                          ; IS THE NUMBER WITHIN RANGE?
                    beq       RPACC1              ; YES. GO SETUP THE PACC REGISTER.
                    lda       #PACCARGE           ; NO. REPORT AN ERROR.
                    jmp       RPTRERR

RPACC1              ldx       IOBASEV
                    stb       PACNT,x             ; PUT NUMBER IN PULSE ACC.
                    rts                           ; RETURN.

;
;
RONTIME             equ       *
                    bsr       CHCKIMID            ; NOT ALLOWED IN IMMIDIATE.
                    jsr       DONEXP              ; GO EVALUATE THE TIME "MATCH" EXPRESSION.
                    jsr       PULNUM              ; GET THE NUMBER OFF THE STACK.
                    std       TIMECMP             ; PUT IN THE COMPARE REGISTER.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; PASS UP COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    sty       ONTIMLIN            ; SAVE THE POINTER TO THE LINE NUMBER.
                    bra       RONIRQ2             ; GO FINISH UP.

;
;
RONIRQ              equ       *
                    bsr       CHCKIMID
                    jsr       DONEXP              ; GO CHECK TO SEE IF WE ARE TO ENABLE OR DISABLE.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO COMMA.
                    iny                           ; BYPASS COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES UP TO LINE NUMBER.
                    jsr       PULNUM              ; GET MODE. SHOULD WE ENABLE THE FUNCTION?
                    bne       RONIRQ1             ; YES.
                    std       ONIRQLIN            ; NO. MAKE THE LINE NUMBER 0.
                    bra       RONIRQ2             ; GO FINISH UP.

RONIRQ1             sty       ONIRQLIN            ; SAVE THE POINTER TO THE LINE NUMBER,
RONIRQ2             ldb       #3                  ; MOVE IP PAST THE LINE NUMBER.
                    aby
                    rts                           ; RETURN.

;
;
RRETI               equ       *
                    bsr       CHCKIMID
                    tpa                           ; CHECK TO SEE IF THE INTERRUPT MASK IS SET.
                    bita      #$10                ; ARE WE IN AN INTERRUPT ROUTINE?
                    bne       RRETI1              ; SINCE THE IRQ MASK IS SET WE MUST BE.
                    lda       #NOTINTER           ; NO. FLAG AN ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

RRETI1              ldd       SCURLINE            ; RESTORE THE MAIN PROGRAM CURRENT LINE.
                    std       CURLINE
                    ldd       SADRNXLN            ; RESTORE MAIN PROGRAM "ADDRESS OF THE NEXT LINE".
                    std       ADRNXLIN
                    ins                           ; TAKE THE RETURN ADDRESS OFF THE STACK.
                    ins
                    rti                           ; GO BACK TO WHERE WE LEFT OFF.

;
;
CHCKIMID            equ       *
                    tst       IMMID               ; ARE WE IN THE IMMIDIATE MODE?
                    beq       CHCKIMI1            ; NO. JUST RETURN.
                    lda       #NOTALERR           ; YES. THIS COMMAND NOT ALLOWED.
                    jmp       RPTRERR             ; REPORT THE ERROR.

CHCKIMI1            rts                           ; RETURN.

;
;
RONPACC             equ       *
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
RONPACC1            cpd       #1                  ; IS THE ARGUMENT <=1?
                    bls       RONPACC2            ; YES. ARG. OK.
RONPACC5            lda       #INTMODER           ; NO. GET ERROR CODE.
                    jmp       RPTRERR

RONPACC2            lda       #$10                ; GET BIT TO ENABLE INTERRUPT.
                    tstb                          ; WAS THE ARGUMENT 0?
                    beq       RONPACC3            ; YES. GO ENABLE INTS. ON EACH COUNT.
                    lsla                          ; NO. ENABLE INTS. ON PACC OVERFLOW ONLY.
RONPACC3            ldx       IOBASEV
                    sta       TMSK2,x
                    jsr       PULNUM              ; GET THE COUNT MODE OFF THE STACK.
                    bne       RONPACC4            ; GO SET THE MODE IF NOT 0.
                    ldx       IOBASEV
                    clr       PACTL,x             ; TURN OFF THE PULSE ACCUMULATOR.
                    std       ONPACLIN            ; CLEAR POINTER TO LINE NUMBER.
                    bra       RONPACC6            ; GO CLEAN UP & RETURN.

RONPACC4            cpd       #4                  ; IS THE ARGUMENT IN RANGE?
                    bhi       RONPACC5            ; YES. REPORT AN ERROR.
                    addb      #3                  ; GET BIT TO ENABLE PACC.
                    lslb
                    lslb
                    lslb
                    lslb
                    ldx       IOBASEV
                    stb       PACTL,x             ; ENABLE THE PACC & SET MODE.
RONPACC6            pula                          ; GET OLD I-BIT STATUS OFF STACK.
                    tap                           ; RESTORE OLD STATUS.
                    ldb       #3
                    aby                           ; PASS UP LINE NUMBER.
                    rts                           ; RETURN.


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "REXPRES.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

;******************************************************************************
;                                                                            *
;               RUNTIME EXPRESSION EVALUATION SUBROUTINE                     *
;                                                                            *
;******************************************************************************

DONEXP              lda       #OPARNTOK           ; USE AN OPEN PAREN AS AN END OF EXPRESSION MARKER.
                    jsr       PSHOP               ; PUSH OPEN PAREN ON THE STACK.
DONEXP1             lda       ,y                  ; GET THE NEXT CHARACTER IN THE EXPRESSION.
                    cmpa      #OPARNTOK           ; IS IT AN OPEN PAREN?
                    bne       DONEXP4             ; NO. CONTINUE.
                    iny                           ; POINT TO NEXT TOKEN.
                    bsr       DONEXP              ; GO DO A SUBEXPRESSION.
                    iny                           ; MOVE THE IP PAST THE CLOSING PAREN.
                    bra       DONEXP1             ; GO GET THE NEXT CHARACTER.

DONEXP4             tsta                          ; CHECK FOR OPERATOR OR OPERAND.
                    bpl       DONEXP2             ; IF NOT VARIABLE OR CONSTANT, GO CHECK FOR FUNCT.
                    bsr       PSHNUM              ; GO PUSH OPERAND ONTO STACK.
                    bra       DONEXP1             ; GO GET NEXT TOKEN.

DONEXP2             jsr       CHKNFUN             ; GO CHECK FOR FUNCTION THAT RETURNS A NUMBER.
                    jsr       CHCKEE              ; GO CHECK FOR END OF EXPRESSION.
                    bcc       DONEXP3             ; IF NOT END OF EXPRESSION, GO PUSH OPERATOR.
                    rts                           ; IF AT END, RETURN.

DONEXP3             iny                           ; POINT TO THE NEXT TOKEN.
                    jsr       PSHOP               ; PUSH OPERATOR ONTO STACK.
                    bra       DONEXP1             ; GO GET NEXT TOKEN.

;
;
;
;        PSHNUM SUBROUTINE
;
;        PUSHES A NUMERIC OPERAND (CONSTANT OR VARIABLE) VALUE ONTO THE
;        OPERAND STACK.
;
;
PSHNUM              cmpa      #IVARTOK            ; IS IT AN INTEGER SCALER VARIABLE?
                    bne       PSHNUM1             ; NO. GO CHECK FOR CONSTANT.
                    ldd       1,y                 ; YES. GET THE "OFFSET" ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START ADDRESS OF THE VARIABLE TABLE.
                    xgdx                          ; GET THE ADDRESS INTO X.
                    ldb       #$03                ; BUMP INTERPRETER POINTER PAST "VARIABLE".
                    aby
                    ldd       3,x                 ; GET THE VARIABLE VALUE.
                    bra       PSHNUM4             ; GO PUT IT ON THE STACK.

PSHNUM1             cmpa      #ICONTOK            ; IS IT AN INTEGER CONSTANT?
                    bne       PSHNUM2             ; NO. GO CHECK FOR AN INTEGER ARRAY VARIABLE.
                    ldx       1,y                 ; GET THE CONSTANT VALUE INTO X.
                    ldb       #$04
                    addb      3,y
                    aby
                    xgdx                          ; PUT THE CONSTANT VALUE INTO D.
                    bra       PSHNUM4             ; GO PUT IT ON THE STACK.

PSHNUM2             cmpa      #IAVARTOK           ; IS IT AN INTEGER ARRAY?
                    bne       PSHNUM3             ; NO. GO CHECK FOR A STRING VARIABLE.
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

PSHNUM3             lda       #ILTOKERR
                    jmp       RPTRERR

PSHNUM4             ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    dex                           ; MAKE ROOM ON THE STACK FOR NEW OPERAND.
                    dex
                    cpx       ENUMSTK             ; HAS THE STACK OVERFLOWED?
                    bhs       PSHNUM5             ; NO. GO STACK THE VALUE.
                    lda       #MSTKOERR           ; YES.
                    sta       ERRCODE
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

PSHNUM5             stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    std       ,x                  ; PUT THE VALUE ON THE STACK.
                    rts                           ; RETURN.

;
;
;        THIS SUBROUTINE CALCULATES BOTH THE BASE ADDRESS AND THE
;        SUBSCRIPT OF THE ARRAY VARIABLE THAT IS CURRENTLY POINTED TO BY
;        THE Y-REG. IT CHECKS TO SEE IF THE VARIABLE HAS BEEN DIMENTIONED
;        AND IF THE SUBSCRIPT IS IN RANGE. THE ROUTINE RETURNS WITH THE
;        ADDRESS OF THE ARRAY IN THE X-REG. & THE SUBSCRIPT IN THE D-REG.
;
CALCSUB             ldd       1,y                 ; GET THE VARIABLE OFFSET ADDRESS.
                    addd      VARBEGIN            ; ADD IN THE START OF THE VARIABLE AREA.
                    xgdx                          ; PUT ADDRESS INTO X.
                    ldx       3,x                 ; GET THE ACTUAL STORAGE ADDRESS.
;                             ; HAS THE ARRAY BEEN DIMENTIONED?
                    bne       CALCSUB2            ; YES. CONTINUE.
                    lda       #UNDIMERR           ; NO. UNDIMENTIONED ARRAY REFERENCE.
CALCSUB1            jmp       RPTRERR             ; GO REPORT THE ERROR.

CALCSUB2            ldb       #$4                 ; SET POINTER TO START OF SUBSCRIPT EXPRESSION.
                    aby
                    pshx                          ; SAVE THE POINTER TO THE ARRAY STORAGE AREA.
                    jsr       DONEXP              ; GO GET THE SUBSCRIPT.
                    iny                           ; BUMP IP PAST THE CLOSING PAREN OF THE SUBSCRIPT.
                    pulx                          ; RESTORE X.
                    bsr       PULNUM              ; GET SUBSCRIPT FROM THE OPERAND STACK.
                    cpd       ,x                  ; IS THE SUBSCRIPT WITHIN RANGE?
                    bls       CALCSUB3            ; YES. CONTINUE.
                    lda       #SUBORERR           ; NO. SUBSCRIPT OUT OF RANGE ERROR.
                    bra       CALCSUB1            ; GO REPORT IT.

CALCSUB3            inx                           ; BYPASS THE SUBSCRIPT LIMIT.
                    inx
                    rts

;
;
PULNUM              pshx                          ; SAVE THE X-REG.
                    ldx       NUMSTACK            ; GET THE OPERAND STACK POINTER.
                    ldd       ,x                  ; GET THE OPERAND.
                    inx                           ; BUMP THE STACK POINTER.
                    inx
                    stx       NUMSTACK            ; SAVE THE STACK POINTER.
                    pulx                          ; RESTORE THE X-REG.
                    cpd       #0                  ; "TEST" THE OPERAND BEFORE WE RETURN.
                    rts                           ; RETURN.

;
;
;        /***** chcknfun() *****/
;
;        /* checks for a numeric function and performs it if present */
;
CHKNFUN             cmpa      #FUNCTFLG           ; IS THIS A FUNCTION CALL?
                    beq       CHKNFUN1            ; YES. GO DO THE FUNCTION.
                    rts                           ; NO. JUST RETURN.

CHKNFUN1            lda       1,y                 ; GET THE FUNCTION CODE BYTE IN B.
                    deca                          ; SUBTRACT 1 FOR INDEXING.
                    ldb       #3                  ; BUMP THE IP.
                    aby                           ; POINT TO THE FIRST ELEMENT IN THE EXPRESSION.
                    tab                           ; PUT THE FUNCTION NUMBER INTO B.
                    aslb                          ; MULT BY THE NUMBER OF BYTES/ADDRESS.
                    ldx       #RNFUNCT            ; POINT TO THE FUNCTION ADDRESS TABLE.
                    abx                           ; POINT TO THE PROPER FUNCTION.
                    ldx       ,x                  ; GET THE ADDRESS INTO X.
                    jsr       ,x                  ; GO DO THE FUNCTION.
                    iny                           ; PUT IP PAST THE CLOSING PAREN.
                    lda       ,y                  ; GET NEXT CHARACTER.
                    rts                           ; RETURN.

;
;
RNFUNCT             equ       *
                    fdb       RFDIV
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
;
;
;        /***** chckee() *****/
;
;        /* if the current token is a semicolin, comma, colin, or space */
;        /* all pending operations on the math stack are performed and */
;        /* we return with the carry set */
;
CHCKEE              equ       *
                    cmpa      #CPARNTOK           ; IS IT A CLOSED PAREN?
                    beq       CHCKEE2             ; YES.
                    cmpa      #MEOLTOK            ; IS IT ONE OF THE "EXPRESSION END" TOKENS?
                    bhs       CHCKEE1             ; YES.
                    clc                           ; FLAG "NOT AT THE END OF EXPRESSION".
                    rts                           ; RETURN.

CHCKEE1             lda       #CPARNTOK           ; END OF EXPRESSION FOUND. PERFORM ALL PENDING
CHCKEE2             bsr       PSHOP               ; OPERATIONS.
                    sec                           ; FLAG END OF EXPRESSION.
                    rts

;
;
PSHOP               ldx       OPSTACK             ; GET THE OPERATOR STACK POINTER.
                    dex                           ; DECREMENT THE STACK POINTER.
                    cpx       EOPSTK              ; DID THE STACK OVERFLOW?
                    bne       PSHOP1              ; NO. CONTINUE.
                    lda       #MSTKOERR           ; YES.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

PSHOP1              stx       OPSTACK
                    sta       ,x                  ; PUT IT ON THE STACK.
PSHOP2              ldx       OPSTACK
                    lda       ,x                  ; GET THE NEW OPERATOR OFF THE TOP OF STACK.
                    cmpa      #OPARNTOK           ; IS IT AN OPEN PAREN?
                    beq       PSHOP5              ; YES. GO PUSH IT.
                    ldb       1,x                 ; GET THE PREVIOUS OPERATOR OFF THE STACK.
                    andb      #$F0                ; MASK ALL BUT THE PRECIDENCE VALUE.
                    anda      #$F0                ; MASK ALL BUT THE OPERATOR PRECIDENCE.
                    cba                           ; IS THE PRECIDENCE OF THE CURRENT OPERATOR >=
;                             ; THE OPERATOR ON THE TOP OF THE STACK?
                    bhi       PSHOP5              ; NO. JUST GO PUSH IT ON THE STACK.
                    lda       1,x                 ; YES. GET THE PREVIOUS OPERATOR FROM THE STACK.
                    ldb       ,x                  ; GET THE CURRENT OPERATOR FROM THE STACK.
                    cmpb      #CPARNTOK           ; IS THE CURRENT OPERATOR A CLOSED PAREN?
                    bne       PSHOP3              ; NO. CONTINUE.
                    cmpa      #OPARNTOK           ; YES. IS THE PREVIOUS OPERATOR AN OPEN PAREN?
                    bne       PSHOP3              ; NO. CONTINUE.
                    inx                           ; YES. KNOCK BOTH OPERATORS OFF THE STACK.
                    inx
                    stx       OPSTACK             ; SAVE THE STACK POINTER.
PSHOP5              rts                           ; RETURN.

PSHOP3              stb       1,x                 ; PUT IT ON THE STACK.
                    inx                           ; UPDATE THE STACK POINTER.
                    stx       OPSTACK
                    bsr       DOOP                ; GO DO THE OPERATION.
                    bra       PSHOP2              ; GO TRY FOR ANOTHER OPERATION.

;
;
DOOP                cmpa      #$70                ; IS IT A UINARY OPERATOR?
                    blo       DOOP1               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$70                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR7              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP1               cmpa      #$60                ; IS IT THE "^" OPERATOR?
                    blo       DOOP2               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$60                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR6              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP2               cmpa      #$50                ; IS IT MULTIPLY, DIVIDE, OR MOD?
                    blo       DOOP3               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$50                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR5              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP3               cmpa      #$40                ; IS IT ADD OR SUBTRACT?
                    blo       DOOP4               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$40                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR4              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP4               cmpa      #$30                ; IS IT A LOGICAL OPERATOR?
                    blo       DOOP5               ; NO. GO CHECK THE NEXT GROUP.
                    suba      #$30                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR3              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP5               cmpa      #$20                ; IS IT AND, OR, OR EOR?
                    blo       DOOP6               ; NO. ERROR.
                    suba      #$20                ; YES. SUBTRACT THE BASE VALUE OF THE GROUP.
                    ldx       #HEIR2              ; POINT TO THE EXECUTION ADDRESS TABLE.
                    bra       DOOP7               ; GO DO THE OPERATION.

DOOP6               lda       #ILTOKERR           ; ILLEGAL OPERATOR TOKEN ENCOUNTERED.
                    jmp       RPTRERR             ; GO REPORT THE ERROR.

DOOP7               tab                           ; PUT THE OFFSET IN B.
                    aslb                          ; MULTIPLY THE OFFSET BY 2.
                    abx                           ; POINT TO THE ROUTINE ADDRESS.
                    ldx       ,x                  ; GET THE ADDRESS.
                    jmp       ,x                  ; GO DO THE OPERATION & RETURN.

;
;
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
;
;
REOR                jsr       PULNUM
                    ldx       NUMSTACK
                    eora      ,x
                    eorb      1,x
REOR1               std       ,x
                    rts

;
;
RORV                jsr       PULNUM
                    ldx       NUMSTACK
                    ora       ,x
                    orb       1,x
                    bra       REOR1

;
;
RAND                jsr       PULNUM
                    ldx       NUMSTACK
                    anda      ,x
                    andb      1,x
                    bra       REOR1

;
;
RPLUS               jsr       PULNUM
                    ldx       NUMSTACK
                    addd      ,x
                    bra       REOR1

;
;
RMINUS              ldx       NUMSTACK
                    ldd       2,x
                    subd      ,x
                    inx
                    inx
                    std       ,x
                    stx       NUMSTACK
                    rts

;
;
RDIV                bsr       RDIVS               ; GO DO A SIGNED DIVIDE.
                    jsr       PULNUM              ; GET INTEGER RESULT OFF STACK.
                    ldx       NUMSTACK            ; POINT TO NUMERIC STACK.
                    std       ,x                  ; OVERWRITE REMAINDER.
                    rts                           ; RETURN.

;
;
;
RDIVS               ldx       NUMSTACK            ; POINT TO NUMERIC STACK.
                    lda       ,x                  ; GET UPPER BYTE OF DIVISOR.
                    eora      2,x                 ; GET SIGN OF THE RESULT.
                    psha                          ; SAVE RESULT.
                    ldd       ,x                  ; GET DIVISOR OFF NUMERIC STACK. IS IT ZERO?
                    bne       RDIV1               ; NO. CONTINUE.
RDIV2               lda       #ZDIVERR            ; YES. GET DIVIDE BY ZERO ERROR.
                    jmp       RPTRERR             ; GO REPORT IT.

RDIV1               bpl       RDIV3               ; IF POSITIVE IT'S OK.
                    bsr       RNEG                ; IF NOT MAKE IT POSITIVE.
RDIV3               tst       2,x                 ; IS THE DIVIDEND NEGATIVE?
                    bpl       RDIV4               ; NO. CONTINUE.
                    ldd       2,x                 ; YES. GET THE NUMBER.
                    coma                          ; NEGATE IT.
                    comb
                    addd      #1
                    std       2,x                 ; SAVE THE RESULT.
RDIV4               ldd       ,x                  ; GET THE DIVISOR.
                    ldx       2,x                 ; GET THE DIVIDEND.
                    xgdx                          ; PUT THEM IN THE PROPER REGISTERS.
                    idiv                          ; DO AN UNSIGNED DIVIDE.
                    pshx                          ; SAVE THE QUOTIENT.
                    ldx       NUMSTACK            ; POINT TO THE NUMERIC STACK.
                    std       2,x                 ; SAVE THE REMAINDER.
;        PULD                 ; GET THE QUOTIENT.
                    pula
                    pulb
                    std       ,x                  ; PUT IT ON THE NUMERIC STACK.
                    pula                          ; GET THE SIGN OF THE RESULT.
                    tsta                          ; SET THE CONDITION CODES.
                    bpl       RDIV5               ; IF PLUS, RESULT OK AS IS.
                    bsr       RNEG                ; MAKE THE QUOTIENT NEGATIVE.
                    ldd       2,x                 ; GET THE REMAINDER.
                    coma                          ; MAKE IT NEGATIVE.
                    comb
                    addd      #1
                    std       2,x                 ; SAVE THE RESULT.
RDIV5               rts                           ; RETURN.

;
;
RMOD                bsr       RDIVS               ; GO GET QUOTIENT & REMAINDER.
                    jsr       PULNUM              ; REMOVE INTEGER RESULT & LEAVE REMAINDER.
                    rts                           ; RETURN.

;
;
RMULT               pshy
                    ldx       NUMSTACK
                    lda       1,x
                    ldb       3,x
                    mul
;        PSHD
                    pshb
                    psha
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
;        PULD
                    pula
                    pulb
                    std       ,x
                    stx       NUMSTACK
                    puly
                    rts

;
;
RINDIR              equ       *
                    rts

;
;
RNOT                equ       *
                    ldx       NUMSTACK
                    ldd       ,x
                    coma
                    comb
                    std       ,x
                    rts

;
;
RNEG                equ       *
                    bsr       RNOT
                    addd      #1
                    std       ,x
                    rts

;
;
RLT                 equ       *
                    bsr       CMPNUM
                    bge       RLT1
RLT2                inc       3,x
RLT1                inx
                    inx
                    stx       NUMSTACK
                    rts

;
;
RGT                 equ       *
                    bsr       CMPNUM
                    ble       RLT1
                    bra       RLT2

;
;
RLTEQ               equ       *
                    bsr       CMPNUM
                    bgt       RLT1
                    bra       RLT2

;
;
RGTEQ               equ       *
                    bsr       CMPNUM
                    blt       RLT1
                    bra       RLT2

;
;
REQ                 equ       *
                    bsr       CMPNUM
                    bne       RLT1
                    bra       RLT2

;
;
RNOTEQ              equ       *
                    bsr       CMPNUM
                    beq       RLT1
                    bra       RLT2

;
;
CMPNUM              equ       *
                    ldx       NUMSTACK
                    ldd       2,x
                    clr       2,x
                    clr       3,x
                    cpd       ,x
                    rts

;
;
RPWR                equ       *
                    rts

;
;
RABS                equ       *
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    bpl       RABS1
RABS2               coma
                    comb
                    addd      #1
RABS1               std       ,x
                    rts

;
;
RSGN                equ       *
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldd       ,x
                    beq       RABS1
                    ldd       #1
                    tst       ,x
                    bpl       RABS1
                    bra       RABS2

;
;
RCALL               equ       *
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    jsr       ,x
                    bra       RPEEK1

;
;
RPEEK               equ       *
                    jsr       DONEXP
                    ldx       NUMSTACK
                    ldx       ,x
                    ldb       ,x
                    clra
RPEEK1              ldx       NUMSTACK
                    std       ,x
                    rts

;
;
RFEEP               equ       *
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

;
;
RFDIV               equ       *
                    jsr       DONEXP              ; GO EVALUATE THE DIVIDEND EXPRESSION.
                    jsr       RSKIPSPC            ; SKIP SPACES.
                    iny                           ; PASS UP THE COMMA.
                    jsr       RSKIPSPC            ; SKIP SPACES AFTER THE COMMA.
                    jsr       DONEXP              ; EVALUATE THE DIVISOR EXPRESSION.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    ldd       2,x                 ; GET THE DIVIDEND.
                    ldx       ,x                  ; GET THE DIVISOR.
                    fdiv                          ; DO THE FRACTIONAL DIVIDE.
                    bvc       RFDIV1              ; ALL IS OK IF V=0. (IX > D).
                    lda       #OVDV0ERR           ; ERROR. EITHER OVERFLOW OR /0 ERROR.
RFDIV2              bra       RFEEP2              ; GO REPORT IT.

RFDIV1              xgdx                          ; PUT QUOTIENT IN D.
                    ldx       NUMSTACK            ; POINT TO OPERAND STACK.
                    inx                           ; REMOVE DIVISOR FROM STACK.
                    inx
                    std       ,x                  ; PUT QUITIENT ON OPERAND STACK.
                    stx       NUMSTACK            ; SAVE NEW VALUE OF STACK POINTER.
                    rts                           ; RETURN.

;
;
RADC                equ       *
                    jsr       DONEXP              ; GO GET THE CHANNEL NUMBER TO CONVERT.
                    ldx       NUMSTACK            ; POINT TO THE RESULT.
                    ldd       ,x                  ; GET THE CHANNEL NUMBER.
                    bmi       RADC4               ; NEGATIVE CHANNEL NUMBERS ARE ILLEGAL.
                    cpd       #7                  ; IS IT A VALID CHANNEL NUMBER?
                    bls       RADC1               ; YES. GO CONVERT IT.
RADC4               lda       #INVCHERR           ; NO. INVALID CHANNEL NUMBER.
                    bra       RFDIV2              ; GO REPORT THE ERROR.

RADC1               ldx       IOBASEV
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

;
;
RRND                equ       *
                    jsr       DONEXP              ; GO GET FUNCTION ARGUMENT.
                    ldx       NUMSTACK            ; GET ARGUMENT OFF STACK. GET NEW RANDOM NUMBER?
                    ldd       ,x
                    beq       RRND2               ; YES. GO GET NEXT RANDOM NUMBER IN THE SERIES.
                    bmi       RRND1               ; IF NEG., START A NEW SERIES.
                    ldd       RANDOM              ; IF POSITIVE, GET LAST RANDOM NUMBER.
                    bra       RRND3               ; RETURN.

RRND1               ldx       IOBASEV
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

;
;
ITAB                equ       *
ICHRS               equ       *
IHEX                equ       *
IHEX2               equ       *
                    lda       #PRFUNERR           ; THESE FUNCTIONS MUST BE USED ONLY IN
                    jmp       RPTRERR             ; PRINT STATEMENTS.

;
;
RFTIME              ldd       TIMEREG             ; GET THE TIME IN SECONDS.
                    bra       RFPORTA2            ; GO PUT NUMBER ON THE STACK.

;
;
RFPACC              ldx       IOBASEV
                    ldb       PACNT,x             ; GET THE CURRENT VALUE OF THE PULSE ACCUMULATOR.
                    clra
                    bra       RFPORTA2            ; GO PUT THE NUMBER ON THE STACK.

;
;
RFPORTA             equ       *
                    ldb       #PORTAIO            ; GET DATA FROM PORTA.
RFPORTA1            ldx       IOBASEV
                    abx
                    ldb       ,x
                    clra                          ; CLEAR UPPER BYTE OF WORD.
RFPORTA2            dey                           ; DECREMENT IP BECAUSE CALLING ROUTINE WILL TRY
                    dey                           ; TO BUMP IT PAST AN OPENING & CLOSING PAREN
;                             ; WHICH ISN'T THERE.
                    jmp       PSHNUM4             ; GO PUSH VALUE ON OPERAND STACK & RETURN.

;
RFPORTB             equ       *
                    ldb       #PORTBIO
                    bra       RFPORTA1

;
RFPORTC             equ       *
                    ldb       #PORTCIO
                    bra       RFPORTA1

;
RFPORTD             equ       *
                    ldb       #PORTDIO
                    bra       RFPORTA1

;
RFPORTE             equ       *
                    ldb       #PORTEIO
                    bra       RFPORTA1


;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "IOPKG.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       COMMUNICATIONS ROUTINES
;******************************************************************************

OUTBYTE             inc       PRINTPOS            ; INCREMENT THE CURRENT PRINT POSITION.
                    pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #OUTABLE            ; POINT TO THE OUTPUT VECTOR TABLE.
OUTBYTE1            ldb       DEVNUM              ; GET THE CURRENT DEVICE NUMBER.
                    aslb                          ; MULT BY 2.
                    abx                           ; POINT TO THE ADDRESS OF THE OUTPUT ROUTINE.
                    ldx       ,x                  ; GET THE ADDRESS. HAS THE VECTOR BEEN INITALIZED?
                    bne       OUTBYTE2            ; YES. GO OUTPUT THE CHARACTER.
                    clr       DEVNUM              ; NO. RESET TO DEVICE #0.
                    lda       #UNINIERR           ; GO REPORT AN UNINITALIZED I/O VECTOR ERROR.
                    jmp       RPTRERR

OUTBYTE2            jsr       ,x                  ; GO OUTPUT THE CHARACTER.
                    pulx                          ; RESTORE X.
                    pulb                          ; RESTORE B.
                    rts                           ; RETURN.

;************

INBYTE              pshb                          ; SAVE THE B-REG.
                    pshx                          ; SAVE THE X-REG.
                    ldx       #INTABLE            ; POINT TO THE INPUT VECTOR TABLE.
                    bra       OUTBYTE1            ; GO USE THE SAME CODE AS OUTBYTE.

;************

SCIIN               equ       *
                    pshx                          ; Save the index register.
                    ldx       IOBASEV
SCIIN1              lda       SCSR,x              ; GET SCI STATUS.
                    anda      #$20                ; HAS A CHARACTER BEEN RECIEVED?
                    beq       SCIIN1              ; NO. WAIT FOR CHARACTER TO BE RECIEVED.
                    lda       SCDR,x              ; GET THE CHARACTER.
                    pulx                          ; Restore X.
                    rts                           ; RETURN.

;************

SCIOUT              pshx                          ; Save the index register.
                    ldx       IOBASEV
                    psha                          ; SAVE THE CHARACTER TO SEND.
SCIOUT1             lda       SCSR,x              ; GET THE SCI STATUS.
                    bita      #$80                ; HAS THE LAST CHARACTER BEEN SHIFTED OUT?
                    beq       SCIOUT1             ; NO. WAIT TILL IT HAS.
                    pula                          ; RESTORE CHARACTER TO SEND.
                    sta       SCDR,x              ; SEND THE CHARACTER.
                    pulx                          ; Restore X.
                    rts                           ; RETURN.

;************

SCISTAT             equ       *
                    pshx                          ; Save the index register.
                    ldx       IOBASEV
                    psha                          ; SAVE THE A-REG.
                    lda       SCSR,x              ; GET THE SCI STATUS.
                    bita      #$20                ; CHECK TO SEE IF A CHARACTER HAS BEEN RECIEVED.
                    pula                          ; RESTORE STATUS.
                    pulx                          ; Restore X.
                    rts                           ; RETURN W/ STATUS.

;************

IODevInit
                    bsr       INITSCI
                    lda       #JMPOP
                    sta       CONSTAT             ; INITIALIZE THE CONSOLE STATUS VECTOR.
                    sta       INCONNE             ; INITIALIZE THE INPUT FROM CONSOLE NO ECHO VECT.
                    ldd       #SCISTAT            ; CONSOLE IS INITIALLY THE SCI.
                    std       CONSTAT+1
                    ldd       #SCIIN              ; GET BYTE FROM SCI, DON'T ECHO IT.
                    std       INCONNE+1
                    rts

;************

INITSCI
                    pshx                          ; Save the index register.
                    ldx       IOBASEV
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

;************

PROUT                                             ; SEND A CHARACTER TO THE PRINTER.
                    bsr       SCISTAT             ; WAS AN "X-OFF" RECIEVED?
                    beq       PROUT1              ; NO. GO SEND THE CHARACTER.
                    psha                          ; SAVE THE CHARACTER TO SEND.
                    bsr       SCIIN               ; YES. GO RESET THE SCI RECEIVER STATUS.
                    cmpa      XOFFCH              ; WAS IT AN XOFF?
                    bne       PROUT2              ; NO. SO GO SEND THE CHARACTER.
PROUT3              bsr       SCIIN               ; GO WAIT FOR AN "X-ON" CHARACTER.
                    cmpa      XONCH               ; IS IT AN X-ON CHARACTER?
                    bne       PROUT3              ; NO. GO WAIT FOR AN X-ON CHARACTER.
PROUT2              pula                          ; GET THE CHARACTER TO SEND.
PROUT1              bra       SCIOUT              ; SEND THE CHARACTER TO THE PRINTER & RETURN.


;************************************************************************
;************************************************************************
;************************************************************************
; test routine to dump memory
;************************************************************************
;************************************************************************
;************************************************************************

; This application is a loader and utility package. Its purpose is to allow
;  other application code to be loaded and setup in Flash memory.

; This code is initially loaded into RAM using the FTC_LOAD.BAS program,
;  from RAM it is executed.

COM                 ldx       IOBASEV

                    jsr       CRLF
                    ldd       #$0000
                    std       DMPGO
                    ldd       #$00FF
                    std       DMPEND
                    bsr       DUMP

                    jsr       CRLF
                    ldd       #$1000
                    std       DMPGO
                    ldd       #$106F
                    std       DMPEND
                    bsr       DUMP

                    bsr       CRLF
                    ldd       #$FF00
                    std       DMPGO
                    ldd       #$000F
                    std       DMPEND
                    bsr       DUMP

                    bsr       DELAY
                    bra       COM

;************************************************************************
;************************************************************************
;************************************************************************
; SUBROUTINES
;************************************************************************
;************************************************************************
;************************************************************************

; SHORT DELAY

DELAY               pshy
                    ldy       #$07FFF
DLY1                dey
                    bne       DLY1
                    puly
                    rts

TESTEND

;***************************************************************
;***************************************************************

DUMP                bsr       DELAY

                    bsr       CRLF

                    ldb       CONFIG,x
                    bsr       TXHEX               ; SEND CONFIG REGISTER

                    bsr       CRLF

                    ldy       DMPGO               ; START POINT

ADDRESS             lda       #$10                ; BYTES PER MESSAGE LINE
                    sta       COUNT1
                    sty       TEMP1
                    ldd       TEMP1
                    pshb
                    psha
                    pulb
                    bsr       TXHEX               ; SEND HI BYTE OF ADDRESS
                    pulb
                    bsr       TXHEX               ; SEND LO BYTE OF ADDRESS
                    ldb       #$20
                    bsr       TXDAT               ; SEND SPACE

DATA2               ldb       ,y
                    bsr       TXHEX

                    ldb       #$20                ; send "space"
                    bsr       TXDAT

                    iny                           ; NEXT ADDRESS
                    cpy       DMPEND              ; END POINT
                    bne       DATA1
                    rts                           ; finished

DATA1               dec       COUNT1
                    bne       DATA2               ; CONITNUE WITH CURRENT LINE

SD1                 bsr       CRLF

                    bsr       DELAY
                    bra       ADDRESS

;***************************************************************
;***************************************************************

; CONVERT & SEND BYTE IN "B"

TXHEX               pshb                          ; SAVE COPY
                    bsr       OUTLHLF             ; SEND LEFT HALF
                    pulb
                    bsr       OUTRHLF             ; SEND RIGHT HALF
                    rts

OUTLHLF             lsrb                          ; shift data to right
                    lsrb
                    lsrb
                    lsrb
OUTRHLF             andb      #$0F                ; mask top half
                    addb      #$30                ; convert to ascii
                    cmpb      #$39
                    ble       OUTA                ; jump if 0-9
                    addb      #$07                ; convert to hex A-F
OUTA                bsr       TXDAT               ; output character
                    rts

; TRANSMIT BYTE IN "B"

TXDAT               brclr     SCSR,x,#%10000000,TXDAT  ; TRANSMITT ONE BYTE
                    stb       SCDR,x
TXDAT2              brclr     SCSR,x,#%10000000,TXDAT2  ; WAIT UNTILL SENT
                    rts

; SEND CRLF

CRLF                ldb       #$0D
                    bsr       TXDAT               ; SEND CR
;                   ldb       #$0A
;                   bsr       TXDAT               ; SEND LF
                    rts


FLASHER             brset     [PORTA,x,LED5,FLS1  ; check LED5
                    bset      [PORTA,x,LED5       ; LED5 = on
                    bra       FLS2

FLS1                bclr      [PORTA,x,LED5       ; LED5 = off

FLS2                pshy
                    ldy       #$07FFF
FLS3                dey
                    bne       FLS3
                    puly
                    rts

                    bra       FLASHER

;*************************************************************************
;                       FLASH MEMORY - IMPORTANT NOTES
;*************************************************************************

; the original CPU_1A1 notes have been removed from here.
; See other CPU_3A2 documentation notes for flash memory details.

;*************************************************************************
; WRITE()

; Writes the data in "A" to the address of "Y".
;  It handles RAM EEPROM and FLASH.
;  On return, carry is set if the write failed, set if passed.

; "X" not affected
; "Y" address to save data to
; "A" data to save
; "B" not affected

WRITE               pshx                          ; save X
                    ldx       IOBASEV             ; register base
; confirm valid target address
                    cpy       #$C000              ; stay out of flash sector 1
                    blo       WR0                 ; lower is OK
                    cpy       #$FE00              ; stay out of flash sector 1
                    blo       WR_END              ; within sector, not valid

                    cpy       #ASFLAG             ; autostart flag
                    beq       WR6                 ; yes
                    cpy       #$FE00              ; buffalo vector
                    beq       WR_END              ; yes
                    cpy       #$FE01              ; buffalo vector
                    beq       WR_END              ; yes
                    cpy       #$FE02              ; buffalo vector
                    beq       WR_END              ; yes

                    cpy       #$FF80              ; eeprom boot code
                    blo       WR0                 ; no
WR_END              rts                           ; address not valid

WR0                 cpy       #CONFIG
                    beq       WR6                 ; CONFIG = EEPROM

                    cpy       #EEP_SA             ; START OF EEPROM
                    bhs       WR6                 ; MUST BE EEPROM

WR1                 cpy       #FLS_EA             ; END OF FLASH
                    bhi       WR2                 ; HIGHER THAN FLASH
                    cpy       #FLS_SA             ; START OF FLASH
                    bhs       WR7                 ; MUST BE FLASH

WR2                 sta       ,y                  ; write RAM
WR3                 cmpa      ,y                  ; verify write
                    beq       WR4                 ; passed
                    sec                           ; set carry = failed
                    bra       WR5                 ; return from subroutine

WR4                 clc                           ; clear carry = passed
WR5                 pulx                          ; restore X
                    rts                           ; return from subroutine

WR6                 sei                           ; disable interrupts
                    bsr       EE_WR               ; byte EEPROM program
                    cli                           ; re-enable interrupts
                    bra       WR3                 ; DONE

WR7                 sei                           ; disable interrupts
                    jsr       FL_RAM              ; check/copy to RAM
                    jsr       FL_WR-FLRC_SA+FL2RAM  ; BYTE PROGRAM FLASH
                    cli                           ; re-enable interrupts
                    bra       WR3                 ; DONE

;*************************************************************************
;*************************************************************************
; EEPROM MEMORY WRITE, ERASE & BULK ERASE

; EE_WR         Writes data in "A" to address in "Y"
; EE_EB         Erases the byte at address in Y.
; EE_BE         Bulk erases EEPROM.
;                Whether eebulk erases the config or not depends on the
;                 address it receives in "Y".

; "X" points to registers
; "Y" address to save data to
; "A" data to save
; "B" not affected

EE_WR               cmpa      ,y                  ; does it need programming ??
                    beq       EE5                 ; NO !!!
                    ldb       ,y                  ; read destination address
                    cmpb      #$FF                ; is it erased ??
                    beq       EE1                 ; NO !!!
                    bsr       EE_EB               ; Erase one byte

EE1                 pshb
                    ldb       #$02
                    stb       PPROG,x
                    sta       ,y
                    ldb       #$03
                    bra       EE2

;*************
; BYTE ERASE ADDRESS Y

EE_EB               pshb
                    ldb       #$16
                    stb       PPROG,x
                    ldb       #$FF
                    stb       ,y
                    ldb       #$17
                    bra       EE2

;*************
; BULK ERASE EEPROM

EE_BE               pshb
                    ldb       #$06
                    stb       PPROG,x
                    sta       ,y                  ; erase config or not ...
                    ldb       #$07                ; ... depends on X addr

EE2                 bne       EE3
                    clrb                          ; fail safe
EE3                 stb       PPROG,x
                    pulb

; delay 10ms at E = 2MHz

                    pshx
                    ldx       #$0D06              ; 10mS time
EE4                 dex
                    bne       EE4
                    pulx
EE5                 clr       PPROG,x
                    rts

;*************************************************************************
;*************************************************************************
; FLASH MEMORY IDENTIFY

; Returns manufacturer and device ID in "A" & "B"

; "X" points to registers
; "A" returns manufacturer ID
; "B" returns device ID

FLID                bsr       FL_RAM              ; ensure setup
                    ldb       #$09                ; third cycle byte (90H = auto select)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
                    ldd       FLS_SA              ; read id, bytes 1 & 2
                    pshb                          ; save B
                    ldb       #$0F                ; third cycle byte (F0H = reset)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
                    pulb                          ; restore B
                    rts                           ; return from subroutine

;*************************************************************************
;*************************************************************************
; FLASH SECTOR ERASE CHECK

; "A" contains the bank address & sector to be checked.
; Carry clear if sector erased, Set if not erased.

; "X" unchanged, points to registers
; "Y" unchanged
; "A" unchanged, indicates the bank & sector to be checked. (returns result)
; "B" unchanged

FL_EC               pshx                          ; save X
                    pshy                          ; save y
                    pshb                          ; save B
                    ldx       IOBASEV             ; register base
; sort out bank
                    ldb       PORTGIO,x
                    andb      #%11101111          ; mask other bits
                    pshb                          ; save bank address
                    psha                          ; save A, bank address to check
                    anda      #%00001111          ; mask all but bank select bits
                    ldb       PORTGIO,x           ; read port G
                    andb      #%1110000           ; mask bank select bits
                    aba                           ; merge A & B to A
                    sta       PORTGIO,x           ; select bank to check
                    pula                          ; restore A

FL_EC1              bita      #%0010000           ; test sector ID bit (ANDA)
                    bne       FL_EC1A             ; check 2nd sector of bank
                    ldy       #FLS_SA             ; first sector
                    bra       FL_EC2              ; continue

FL_EC1A             ldy       #FLS_SA+$4000       ; second sector
; check now
FL_EC2              ldb       ,y                  ; read byte
                    cmpb      #$FF                ; check
                    bne       FL_EC4              ; sector not erased
                    iny                           ; next address
                    bita      #%00010000          ; test sector ID bit (ANDA)
                    bne       FL_EC3              ; checking 2nd sector of bank
                    cpy       #FLS_SA+$4000       ; last address + 1
                    beq       FL_EC5              ; sector erased
                    bra       FL_EC2              ; continue

FL_EC3              cpy       #FLS_EA+1           ; last address + 1
                    beq       FL_EC5              ; sector erased
                    bra       FL_EC2              ; continue

; log result
FL_EC4              sec                           ; restore A (not erased)
                    bra       FL_EC6              ; continue

FL_EC5              clc                           ; restore A (erased)
; finished
FL_EC6              pulb                          ; recover active bank
                    bclr      PORTGIO,x,%00001111  ; clear existing
                    orb       PORTGIO,x           ; merge
                    stb       PORTGIO,x           ; restore active bank
                    pulb                          ; restore B
                    puly                          ; restore Y
                    pulx                          ; restore X
FL_EC9              rts                           ; return from subroutine

;*************************************************************************
;*************************************************************************
; COPY FLASH ROUTINES TO RAM

; This routine checks to see if the code is already in RAM. If not, it
;  is copied accross.
; The routines that must be run from RAM are stored in FLASH. Before thay
;  can be used, the code must be copied to RAM.
; The source and destination addresses are preset/fixed for simplicity.

; "X" unchanged
; "Y" unchanged
; "A" unchanged
; "B" unchanged

FL2RAM              equ       $7E00               ; destination address

FL_RAM              psha                          ; save A
                    pshb                          ; save B
                    ldd       FL_WR-FLRC_SA+FL2RAM  ; check 1st word
                    cpd       FL_COM              ; mask
                    bne       FL_RAM1             ; load flash routines
                    ldd       FL_WR-FLRC_SA+FL2RAM+1  ; check 2nd word
                    cpd       FL_COM+2            ; mask
                    bne       FL_RAM1             ; load flash routines
FL_RAM0             pulb                          ; restore B
                    pula                          ; restore A
                    rts                           ; finished

FL_RAM1             pshx                          ; save X
                    pshy                          ; save Y
                    ldx       #FLRC_SA            ; first source address
                    ldy       #FL2RAM             ; first destination address
FL_RAM2             lda       ,x                  ; read
                    sta       ,y                  ; write
                    iny                           ; next destination address
                    inx                           ; next source address
                    cpx       #FLRC_EA            ; last source address + 1
                    bne       FL_RAM2             ; next byte
                    puly                          ; restore Y
                    pulx                          ; restore X
                    bra       FL_RAM0             ; continue

;*************************************************************************
;*************************************************************************
; THESE ROUTINES ARE RUN FROM RAM                               RAM

FLRC_SA                                           ; first address

; This code is assembled to be relocatable. This allows it to be copied
;  to another area of memory and still work. The JMP and JSR commands
;  cause the problems.

;       JSR     FL_COM          to      FL_COM-FLRC_SA+FL2RAM   (command)
;       JSR     FL_WR           to      FL_WR-FLRC_SA+FL2RAM    (write)
;       JSR     FL_BE           to      FL_BE-FLRC_SA+FL2RAM    (erase)

; A copy is required to be held in FLASH for later reloading.
; The initial FLASH loader should be able to copy the code from RAM to
;  FLASH. This will ensure that when it is copied back for use, it will
;  work.

;*************************************************************************
;*************************************************************************
; SUBBROUTINE = ENTER FLASH COMMAND

; "X" unchanged, points to registers
; "Y" unchanged
; "A" unchanged
; "B" unchanged, contains third cycle data byte (00 = skip third cycle)

FL_COM              pshx                          ; save X
                    psha                          ; save A
                    ldx       IOBASEV             ; register base
                    lda       PORTGIO,x
                    anda      #%00001111          ; mask other bits
                    psha                          ; save active bank address
                    lda       PORTGIO,x
                    anda      #%11110000          ; mask other bits
                    sta       PORTGIO,x           ; select first bank
                    lda       #$55
                    sta       FLS_SA+$55AA        ; first cycle AA to 5555
                    lda       #$AA
                    sta       FLS_SA+$2A55        ; second cycle 55 to 2AAA
                    tstb                          ; test B for 00
                    beq       FL_COM1             ; skip third cycle
                    stb       FLS_SA+$55AA        ; third cycle ?? to 5555
FL_COM1
                    pula                          ; recover active bank
                    bclr      PORTGIO,x,%00001111  ; clear existing
                    ora       PORTGIO,x           ; merge
                    sta       PORTGIO,x           ; restore active bank
                    pula                          ; restore B
                    pulx                          ; restore X
                    rts                           ; return from subroutine

;*************************************************************************
;*************************************************************************
; FLASH MEMORY WRITE

; Writes data in "A" to address in "Y"
; This code must be run from RAM
; See notes at end of FLASH routines

; "X" unchanged, points to registers
; "Y" unchanged, address to save data to
; "A" unchanged, data to save
; "B" unchanged

FL_WR               pshx                          ; save X
                    pshb                          ; save B
                    ldx       IOBASEV             ; register base
                    ldb       ,y                  ; read location
                    cmpb      #$FF                ; check erased
                    bne       FL_W2               ; not erased
                    ldb       #$05                ; third cycle byte (A0)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
                    sta       ,y                  ; write data
FL_W1               cmpa      ,y                  ; check progress
                    bne       FL_W1               ; not finished
FL_W2               pulb                          ; restore B
                    pulx                          ; restore X
                    rts                           ; not erased

;*************************************************************************
;*************************************************************************
; FLASH MEMORY BULK ERASE                     ( caution, very dangerous )

; "A" contains the bank address and sector to be erased. If "A" contains
;  FF then the entire chip is erased. Take care, no second chances.
; This code must be run from RAM
; See notes at end of FLASH routines

; Bits xxxx1111 specify the bank, bit xxx1xxxx, indicates the sector within
;  the bank. The 29F010 has 2 16K sectors per 32K bank.

; "X" points to registers
; "A" indicates the sector to be erased, not affected
; "B" not affected

FL_BE               pshx                          ; save X
                    psha                          ; 1 save A
                    pshb                          ; 2 save B
                    ldx       IOBASEV             ; register base

FL_BE1              ldb       #$01                ; third cycle byte (80H = erase)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
                    cmpa      #$FF                ; test A for FF (erase all)
                    beq       FL_BE6              ; erase chip (all) now
; sort out bank
                    ldb       PORTGIO,x
                    andb      #%11101111          ; mask other bits
                    pshb                          ; 3 save initial bank address
                    psha                          ; 4 save A
                    anda      #%00001111          ; mask all but bank select bits
                    ldb       PORTGIO,x           ; read port G
                    andb      #%11110000          ; mask bank select bits
                    aba                           ; merge A & B to A
                    sta       PORTGIO,x           ; select bank to erase
                    pula                          ; 4 restore A
; erase sector
                    ldb       #$00                ; third cycle byte (skip)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
                    ldb       #$0C
                    bita      #%00010000          ; test sector ID bit (ANDA)
                    bne       FL_BE3              ; erase 2nd sector of bank
                    stb       FLS_SA              ; third cycle (30H = erase 1st sector)
FL_BE2              ldb       FLS_SA              ; wait for erase to complete
                    andb      #%00000001          ; DQ7 (swapped)
                    beq       FL_BE2              ; wait
                    bra       FL_BE5              ; continue

FL_BE3              stb       FLS_SA+$4000        ; third cycle (30H = erase 2nd sector)
FL_BE4              ldb       FLS_SA+$4000        ; wait for erase to complete
                    andb      #%00000001          ; DQ7 (swapped)
                    beq       FL_BE4              ; wait
FL_BE5              pulb                          ; 3 recover initial bank address
                    bclr      PORTGIO,x,%00001111  ; clear existing
                    orb       PORTGIO,x           ; merge
                    stb       PORTGIO,x           ; restore active bank
                    bra       FL_BE8

; erase chip
FL_BE6              ldb       #$08                ; third cycle byte (10H = chip erase)
                    jsr       FL_COM-FLRC_SA+FL2RAM  ; enter command mode
FL_BE7              lda       FLS_SA              ; wait for erase to complete
                    anda      #%000000001         ; DQ7 (swapped)
                    beq       FL_BE7              ; wait

FL_BE8              pulb                          ; 2 restore B
                    pula                          ; 1 restore A
                    pulx                          ; restore X
FL_BE9              rts                           ; return from subroutine

FLRC_EA                                           ; last address

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
; variable initalisation
;******************************************************************************

; */***** initvars() *****/
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
;
;
INITVARS
                    ldx       #RAMSTART           ; YES. VARIABLES START AT RAMBEG.
                    stx       VARBEGIN            ; SET POINTERS TO BOTH START AND END OF VARIABLE
                    stx       VAREND              ; STORAGE.
                    stx       BASBEG              ; SET POINTERS TO BOTH THE START AND END OF THE
                    stx       BASEND              ; BASIC PROGRAM.
                    xgdx                          ; GET THE START OF RAM INTO D
                    addd      #RAMSIZE            ; add the size of the RAM to it.
                    subd      #SWSTKSize+1        ; subtract the size of the software stack, token & input buffers.
; stack moved to lower RAM (below CPU registers)
                    std       VARMEND             ; SAVE THE POINTER.
                    std       BASMEND             ; MAKE IT THE END OF BASIC PROGRAM MEMORY.
                    addd      #1                  ; Set up a pointer to the input buffer.
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

; clear basic program storage space

INIT1               clr       ,x                  ; CLEAR THE STORAGE TO ZERO.
                    inx                           ; POINT TO THE NEXT LOCATION.
                    cpx       BASMEND             ; ARE WE DONE?
                    bls       INIT1               ; NO. KEEP CLEARING.

; check for auto-start

                    ldx       #PSSTART
                    lda       ASFLAG              ; GET THE AUTO START FLAG AGAIN.
                    cmpa      #$55                ; IS THE AUTO START MODE SET?
                    bne       INIT5               ; NO. DO A NORMAL INIT.
                    jsr       AUTOLOAD            ; GO LOAD the program and VARIABLES INTO RAM.

; misc

INIT5               ldd       #0                  ; MAKE THE HIGHEST LINE IN THE PROGRAM 0.
                    std       HILINE
                    std       CURLINE             ; MAKE THE CURRENT LINE #0.
                    jsr       RUNINIT             ; GO INITALIZE ALL THE SOFTWARE STACKS.
                    clr       TRFLAG              ; TURN THE TRACE MODE OFF.
                    lda       #1                  ; "CONT" COMMAND NOT ALLOWED.
                    sta       CONTFLAG
                    clr       DEVNUM              ; MAKE THE DEFAULT DEVICE NUMBER 0 (CONSOLE).
                    clr       IMMID               ; clear the immediate mode flag (added 9/17/90).
                    ldx       VAREND              ; GET THE POINTER TO THE END OF VARIABLE STORAGE.
                    inx                           ; BUMP IT BY 1.
                    stx       STRASTG             ; POINT TO THE DYNAMIC ARRAY STORAGE.

                    rts                           ; RETURN.

;******************************************************************************
;       interrupt service routines
;******************************************************************************

;************
; interrupt 1

SCIINT              lda       #01
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 2

SPIINT              lda       #02
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 3

PACCINT
                    ldx       IOBASEV
                    lda       #$30                ; RESET BOTH THE TIMER OVERFLOW & INPUT FLAG.
                    sta       TFLG2,x
                    ldy       ONPACLIN            ; GET POINTER TO LINE NUMBER OF THE SERVICE ROUT.
                    bne       TIMINTS2
                    rti

;************
; interrupt 4

PACOINT             lda       #04
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 5

TOINT               lda       #05
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 6

OC5INT              lda       #06
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 7

OC4INT              lda       #07
                    sta       GWIZ1
                    jmp       WRONG

;************
; interrupt 8

OC3INT              lda       #08
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 9

OC2INT              lda       #09
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 10

OC1INT              bsr       TIMINTS             ; service interrupt
                    rti                           ; return from interrupt

; periodic time interrupt

TIMINTS             ldx       IOBASEV             ; Point to the I/O Base Address.
                    ldd       TOC1,x              ; GET THE VALUE OF THE TIMER/COUNTER.
TIMINTS3            addd      #TIMEVAL            ; ADD IN 62500 FOR NEXT COMPARE ( 2 HZ INT.).
                    std       TOC1,x              ; PUT IT IN THE OUTPUT COMPARE REGISTER.
                    lda       #$80                ; SETUP TO CLEAR THE OC1 FLAG.
                    sta       TFLAG1,x
                    dec       TIMEPRE             ; HAVE TWO OUTPUT COMPARES OCCURED?
                    bne       TIMINTS1            ; NO. JUST RETURN.
                    lda       #SWPRE              ; YES. RELOAD THE REGISTER.
                    sta       TIMEPRE
                    ldd       TIMEREG             ; GET THE CURRENT VALUE OF "TIME".
                    addd      #01                 ; ADD 1 SECOND TO THE COUNT.
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

TIMINTS1            rts                           ; RETURN

;************
; interrupt 11

IC3INT              lda       #11
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 12

IC2INT              lda       #12
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 13

IC1INT              lda       #13
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 14

RTIINT              lda       #14
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 15

IRQINT              ldy       ONIRQLIN            ; GET POINTER TO LINE NUMBER OF THE IRQ SERVICE.
                    bne       TIMINTS2            ; GO DO IT.
                    rti                           ; IF IT'S 0, "ONIRQ" HAS NOT BEEN EXECUTED.

;************
; interrupt 16

XIRQINT             lda       #16
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 17

SWINT               lda       #17
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 18

ILOPINT             lda       #18
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 19

COPINT              lda       #19
                    sta       GWIZ1
                    bra       WRONG

;************
; interrupt 20

CLMINT              lda       #20
                    sta       GWIZ1
; bra     WRONG

;************
; unexpected interrupt

WRONG               lda       ASFLAG              ; GET THE AUTO START FLAG.
                    cmpa      #$55                ; IS IT SET?
                    bne       WRONG1              ; NO. DUMP MEMORY
                    lda       #$AA                ; YES. RESET (ERASE) IT.
                    ldy       #ASFLAG             ; autostart flag address
                    jsr       WRITE               ; erase auto-start flag
                    jmp       $C000               ; restart Basic11

WRONG1              jmp       COM                 ; dump memory to SCI

;******************************************************************************
; ROM tables
;******************************************************************************

; vector tables in ROM

;************
; This table is used to initalise the I/O routine table in RAM.

IOVects             fdb       SCIIN               ; input device #1 (ACIA)
                    fdb       SCIIN               ; input device #2 (SCI)
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       PROUT               ; output device #1 (ACIA)
                    fdb       PROUT               ; output device #2 (SCI)
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0
                    fdb       0

;************
; This table is used to initalise the interrupt vector table in RAM.

;                       routine         rom     ram     description

INTVECT_SA          fdb       SCIINT              ; D6 00C4 SCI (FFD6)
                    fdb       SPIINT              ; D8 00C7 SPI
                    fdb       PACCINT             ; DA 00CA PULSE ACCUMULATOR I/P EDGE
                    fdb       PACCINT             ; DC 00CD PULSE ACCUMULATOR OVERFLOW
                    fdb       TOINT               ; DE 00D0 TIMER OVERFLOW
                    fdb       OC5INT              ; E0 00D3 TIMER O/P COMPARE 5
                    fdb       OC4INT              ; E2 00D6 TIMER O/P COMPARE 4
                    fdb       OC3INT              ; E4 00D9 TIMER O/P COMPARE 3
                    fdb       OC2INT              ; E6 00DC TIMER O/P COMPARE 2
                    fdb       OC1INT              ; E8 00DF TIMER O/P COMPARE 1
                    fdb       IC3INT              ; EA 00E2 TIMER I/P COMPARE 3
                    fdb       IC2INT              ; EC 00E5 TIMER I/P COMPARE 2
                    fdb       IC1INT              ; EE 00E8 TIMER I/P COMPARE 1
                    fdb       RTIINT              ; F0 00EB REAL TIME INTERRUPT
                    fdb       IRQINT              ; F2 00EE EXTERNAL IRQ
                    fdb       XIRQINT             ; F4 00F1 EXTERNAL XIRQ
                    fdb       SWINT               ; F6 00F4 SOFTWARE INTERRUPT (SWI)
                    fdb       ILOPINT             ; F8 00F7 ILLEGAL OPCODE
                    fdb       COPINT              ; FA 00FA COP OPERATED
                    fdb       CLMINT              ; FC 00FD CLOCK MONITOR OPERATED
INTVECT_EA          fdb       POWERUP             ; FE ---- RESET

;*************
; message display for help

CHELP_SCN           fcc       ESC,"[2J"           ; CLR SCREEN
                    fcc       ESC,"[0;0H"         ; TOP LEFT CORNER FCC "BASIC11 V1.55"

                    fcc       '                   B A S I C 1 1   for   C P U _ 3 A 2'
                    fcb       $0D,$0A,$0A
                    fcc       '                               Commands'
                    fcb       $0D,$0A
                    fcc       'AUTOST Auto-run program at reset          NOAUTO Cancell auto-run'
                    fcb       $0D,$0A
                    fcc       'CLEAR  Clear all variables                CONT   Continue after BREAK'
                    fcb       $0D,$0A
                    fcc       'ELOAD  Load program from Flash            ESAVE  Save program to Flash'
                    fcb       $0D,$0A
                    fcc       'FERASE Erase Flash program memory         FREE   Displays amount of free memory'
                    fcb       $0D,$0A
                    fcc       'HELP   You have found this one            LLIST  List current program'
                    fcb       $0D,$0A
                    fcc       'NEW    Clear for a new program            RUN    Run current program'
                    fcb       $0D,$0A,$0A

                    fcc       '                              Keywords'
                    fcb       $0D,$0A
                    fcc       '? (PRINT)   ABS       ADC       CALL      CHR$      CLS       DATA      DIM'
                    fcb       $0D,$0A
                    fcc       'EEP         ELSE      END       ENDWH     FDIV      FOR       GOSUB     GOTO'
                    fcb       $0D,$0A
                    fcc       'HEX         HEX2      IF        INBYTE    INPUT     LET       NEXT      NOT'
                    fcb       $0D,$0A
                    fcc       'ON          ONIRQ     ONPACC    ONTIME    PACC      PEEK      POKE      PORT'
                    fcb       $0D,$0A
                    fcc       'PORTA       PORTB     PORTC     PORTD     PORTE     PRINT     READ      REM'
                    fcb       $0D,$0A
                    fcc       'RESTORE     RETI      RETURN    RND       RTIME     SGN       SLEEP     STEP'
                    fcb       $0D,$0A
                    fcc       'STOP        TAB       THEN      TIME      TO        TROFF     TRON      WHILE'
                    fcb       $0D,$0A,$0A

                    fcc       '                          Symbols & Operators'
                    fcb       $0D,$0A
                    fcc       '+ - * / \ <= >= <> < > = - : ; , ( ) # .AND. .OR. .EOR.'
                    fcb       $00
;                   fdb       $0D,$0A

;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************
;       include "EEPROM.ASM"
;******************************************************************************
;******************************************************************************
;******************************************************************************
;******************************************************************************

;************
; EEPROM code for CPU_3A2

          #ifdef _EEPROM
; INITIAL CPU SETUP FOR LOADER

                    org       $FF80               ; special variable area

BOOT1               ldx       #$1000              ; register base address

;                   lda       #%10010011          ; adpu, irqe, dly, cop = long (1a1 version)
                    lda       #%10010001          ; adpu, irqe, dly, cop = 65mS
                    sta       OPTION
                    lda       #%00000000          ; TMSK2 REG. DATA
                    sta       TMSK2
                    lda       #$00
                    sta       BPROT               ; make CONFIG & EEPROM writable

                    lds       #$3FF               ; NORMALLY 03FFH

                    lda       #%00000000
                    sta       CSSTRH,x            ; CS cycle stretch
                    lda       #%10000101
                    sta       CSCTL,x             ; enable program CS for 32K and bank
                    lda       #%00000000
                    sta       CSGADR,x            ; RAM starts at address 0000H
                    lda       #%10000001
                    sta       CSGSIZ,x            ; RAM block size is 32K

                    lda       #%11111111          ; enable SRAM (CE2) and select banks
                    sta       BANKSR              ; select banks (Flash and RAM)
                    sta       BANKSS              ; make working copy

                    jmp       $C000               ; run application code

;************
; INTERRUPT VECTORS (IN EEPROM)

                    org       $FFFF-$29           ; TOP OF ROM

INTVECT             fdb       SCISS               ; D6 C4 SCI (FFD6)
                    fdb       SPITC               ; D8 C7 SPI
                    fdb       PACCIE              ; DA CA PULSE ACCUMULATOR I/P EDGE
                    fdb       PACCOVF             ; DC CD PULSE ACCUMULATOR OVERFLOW
                    fdb       TIMEROVF            ; DE D0 TIMER OVERFLOW
                    fdb       TIMOC5              ; E0 D3 TIMER O/P COMPARE 5
                    fdb       TIMOC4              ; E2 D6 TIMER O/P COMPARE 4
                    fdb       TIMOC3              ; E4 D9 TIMER O/P COMPARE 3
                    fdb       TIMOC2              ; E6 DC TIMER O/P COMPARE 2
                    fdb       TIMOC1              ; E8 DF TIMER O/P COMPARE 1
                    fdb       TIMIC3              ; EA E2 TIMER I/P COMPARE 3
                    fdb       TIMIC2              ; EC E5 TIMER I/P COMPARE 2
                    fdb       TIMIC1              ; EE E8 TIMER I/P COMPARE 1
                    fdb       REALTIMI            ; F0 EB REAL TIME INTERRUPT
                    fdb       IRQI                ; F2 EE EXTERNAL IRQ
                    fdb       XIRQ                ; F4 F1 EXTERNAL XIRQ
                    fdb       SWII                ; F6 F4 SOFTWARE INTERRUPT (SWI)
                    fdb       ILLOP               ; F8 F7 ILLEGAL OPCODE
                    fdb       COP                 ; FA FA COP OPERATED
                    fdb       CMF                 ; FC FD CLOCK MONITOR OPERATED
                    fdb       BOOT1               ; FE RESET
          #endif
