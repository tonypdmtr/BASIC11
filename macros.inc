;*******************************************************************************
;* Include   : MACROS.INC
;* Programmer: Tony Papadimitriou <tonyp@acm.org>
;* Purpose   : Sample macro definitions for ASM11 (Win32 & Linux versions, only)
;* Language  : Motorola/Freescale/NXP 68HC11 Assembly Language (aspisys.com/ASM11)
;* Status    : FREEWARE Copyright (c) 2020 by Tony Papadimitriou <tonyp@acm.org>
;* Original  : http://www.aspisys.com/code/hc11/macros.html
;* Note(s)   : Use: #Uses macros.inc
;*******************************************************************************

                    #Exit     _MACROS_
_MACROS_

;*******************************************************************************
; My personal preferences for most situations (adjust to suit your needs)

MyDefaultDirectives macro
                    #CaseOn                       ;Case-sensitive labels
                    #OptRelOn                     ;Jump->Branch warnings
                    #OptRtsOff                    ;No redundant RTS warnings
                    #SpacesOff                    ;No non-string spaces in operands
                    #ExtraOn                      ;Extra mnemonics enabled
                    #MapOn                        ;Source-level mapping enabled
                    #TraceOff                     ;No source-level macro mapping
                    #@Macro                       ;Macro syntax is @macro
                    #Parms                        ;Default macro parameter delimiter
                    endm

;*******************************************************************************
;*******************************************************************************
                    @MyDefaultDirectives          ;this one called at inclusion
;*******************************************************************************
;*******************************************************************************

;*******************************************************************************
; Assign a default value to a symbol (if the symbol is not already defined).
; If <expr> is missing, simply define the symbol with current address pointer

;*******************************************************************************

_cop_               macro
                    lda       #$55
                    sta       COPRST
                    coma
                    sta       COPRST
                    endm

;-------------------------------------------------------------------------------

COP                 macro
          #ifdef KickCOP
                    !jsr      KickCOP
                    mexit
          #endif
                    psha
                    @@_cop_
                    pula
                    endm

;*******************************************************************************
; Simulate 68HC08's RSP instruction

rsp                 macro     [[#]StackTop]       ;does LDS with most-used value
                    mdef      1,#STACKTOP
                    lds       ~1~
                    endm

;*******************************************************************************
; Clear a range to zero (or given value)
; Note: X is destroyed

ClrRange            macro     [#]FromAddress,[#]ToAddress[,[#]WithValue]
                    mreq      1,2:[#]FromAddress,[#]ToAddress[,[#]WithValue]
          #ifparm ~,2~ = ,x
                    merror    X-indexed mode in ToAddress not supported
          #endif
                    ldx       ~1~
Loop$$$
          #ifparm ~3~
                    psha
                    lda       ~3~
                    sta       ,x
                    pula
          #else
                    clr       ,x
          #endif
                    inx
                    cmpx      ~2~
                    blo       Loop$$$
                    endm

;*******************************************************************************
; Swap the values of two symbols (using old XOR technique, and no temp symbol)

SwapSymbols         macro     Symbol1,Symbol2
                    mreq      1,2:Symbol1,Symbol2
~1~                 set       {~1~}^{~2~}
~2~                 set       {~1~}^{~2~}
~1~                 set       {~1~}^{~2~}
                    endm

;*******************************************************************************
; Align the current segment (e.g, #ROM) to specific power-of-two block size.
; If a label is present on the same line as the macro call, set the label to the
; value after the alignment occurs.  Default power is 0, byte alignment.
; If the "UnalignedValue" expression is present (and its value is other than
; :PC, then instead of aligning the current segment, it only sets the label to
; aligned value.  This way, it can be used to align just a label without the
; segment.

Align2              macro     Power[,UnalignedValue]
                    mdef      1,0
                    mdef      2,:PC
                    mset      1,{1<{~1~}+{~2~}-1(h)}&{1<{~1~}-1^$FFFFFF(h)}
          #ifparm ~2~ = :PC
                    org       ~1~
          #endif
          #ifparm ~label~
~label~             set       ~1~
          #endif
                    mexit     ~1~
                    endm

;*******************************************************************************
; Define PIN names (PIN for port, PIN. for pin number/mask)
; (It can also be used for bit-mapped registers or variables)
; PinName can be either the 1st parameter (when three parameters are present),
; or the label on the left side of the macro (when two parameters are present)

Pin                 macro     [[PinName,]PORT[,BitNumber]]
          #ifb ~@~
              #ifb ~label~
                    merror    A label for the pin is required
              #endif
              #ifb ~text~
                    merror    PORT parm required on first use
              #endif
                    #temp     ~text~.+1
~label~             set       ~text~
~label~.            equ       1<{:temp}
                    mset      0,~label~
          #endif
          #ifb ~label~
                    mreq      1,2:PinName,PORT[,BitNumber]
                    mdef      3,0
                    mset      0,~1~
                    #temp     ~3~
~1~                 set       ~2~
~1~.                set       1<~3~
          #else
              #ifnb ~@~
                    mreq      1:PORT[,BitNumber]
                    mdef      2,0
                    mset      0,~label~
                    #temp     ~2~
~label~             set       ~1~
~label~.            equ       1<~2~
              #endif
          #endif
          #if :temp > 7
                    #Warning  BitNumber ({:temp}) > 7
          #endif
                    endm

;*******************************************************************************
; Check if a pin has been defined (normally via @PIN), and issue error otherwise
; (It can also be used for bit-mapped registers or variables)
; Place it inside a general-purpose module to warn the user including the module
; about missing but required pin definitions.  The user then simply needs to add
; the correct @pin definitions for each missing pin.
; (Note: MSET allows us to do a simple trick; use parameter 2 as an 'embedded
; macro' to define the error directive once, even though we use it twice.)

CheckPin            macro     PinName[,PinName]*
                    mset      0,#Error Pin \@~{:loop}.~\@ not defined with @Pin
          #ifndef ~{:loop}.~
                    ~text~
                    mtop      :n
                    mexit
          #endif
          #ifndef ~{:loop}.~.
                    ~text~
                    mtop      :n
          #endif
                    endm

;*******************************************************************************
; Define Bit names using Bit.

BitNum              macro     BinName,BitNumber
                    mreq      1,2:BinName,BitNumber
~1~.                equ       1<~2~
                    endm

;*******************************************************************************
; Define all BitName bits from MinNumber to MaxNumber

Bits                macro     BitName,MinNumber,MaxNumber[,FirstBit]
                    mreq      1,2,3:BitName,MinNumber,MaxNumber[,FirstBit]
                    mdef      4,0
                    mdo
~1~{~2~+:mloop-1}.  equ       1<{~4~+:mloop-1}
                    mloop     {~3~-~2~+1}
                    endm

;*******************************************************************************
; Make PIN an input or output, accordingly

Input               macro     PinName[,RegX|RegY]
                    mreq      1:PinName[,RegX|RegY]
          #ifz ]~1~
                    bclr      [~1~+DDR,#~1~.
          #else
                    bclr      [~1~+DDR,~2~,#~1~.
          #endif
                    endm

;-------------------------------------------------------------------------------

Output              macro     PinName[,RegX|RegY]
                    mreq      1:PinName[,RegX|RegY]
          #ifz ]~1~
                    bset      [~1~+DDR,#~1~.
          #else
                    bset      [~1~+DDR,~2~,#~1~.
          #endif
                    endm

;*******************************************************************************
; Delay X number of msec.  Automatically adjust delay to bus speed (BUS_KHZ).

DelayXms            macro     [#]msec
                    mset      #
                    mreq      1:[#]msec
          #ifndef DelayMS
                    bra       Skip$$$
;*******************************************************************************
                              #Cycles
DelayMS             proc
                    pshx
                    ldx       #DELAY@@
                              #Cycles
Loop@@              decx
                    bne       Loop@@
                              #temp :cycles
                    pulx
                    rts

DELAY@@             equ       BUS_KHZ-:cycles-:ocycles/:temp
;*******************************************************************************
Skip$$$             endp
          #endif
                    psha
                    lda       ~1~
Loop$$$
          #if *-DelayMS < 128
                    bsr       DelayMS
          #else
                    jsr       DelayMS
          #endif
                    deca
                    bne       Loop$$$
                    pula
                    endm

;*******************************************************************************
; Turn PIN On or Off, accordingly, and make sure it's an output.

On                  macro     PinName[,RegX|RegY]
                    mreq      1:PinName[,RegX|RegY]
          #ifz ]~1~
                    bset      ~1~,#~1~.
                    bset      ~1~+DDR,#~1~.
          #else
             #ifnoparm ~2~
                    merror    Usage: @~0~ PinName[,RegX|RegY]
             #endif
                    bset      [~1~,~2~,#~1~.
                    bset      [~1~+DDR,~2~,#~1~.
          #endif
                    endm

;-------------------------------------------------------------------------------

Off                 macro     PinName[,RegX|RegY]
                    mreq      1:PinName[,RegX|RegY]
          #ifz ]~1~
                    bclr      ~1~,#~1~.
                    bset      ~1~+DDR,#~1~.
          #else
             #ifnoparm ~2~
                    merror    Usage: @~0~ PinName[,RegX|RegY]
             #endif
                    bclr      [~1~,~2~,#~1~.
                    bset      [~1~+DDR,~2~,#~1~.
          #endif
                    endm

;*******************************************************************************
; Toggle Pin name

Toggle              macro     PinName
                    mreq      1:PinName
                    psha
                    lda       ~1~
                    eora      #~1~.               ;toggle pin
                    sta       ~1~
                    pula
                    endm

;*******************************************************************************
; CBEQA HC08/9S08 equivalent, and similar ones

cbeqa               macro     Value,Address
                    mreq      1,2:Value,Address
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    cmpa      ~1~
                    beq       ~2~
                    endm

cjeqa               macro     Value,Address
                    mreq      1,2:Value,Address
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    cmpa      ~1~
                    jeq       ~2~
                    endm

cbnea               macro     Value,Address
                    mreq      1,2:Value,Address
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    cmpa      ~1~
                    bne       ~2~
                    endm

cjnea               macro     Value,Address
                    mreq      1,2:Value,Address
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    cmpa      ~1~
                    jne       ~2~
                    endm

;*******************************************************************************
; ADD with Carry D

adcd                macro     [#]Operand
                    mreq      1:[#]Operand
          #ifparm ~#~
                    adcb      #~1~&$FF
                    adca      #~1~>8
                    mexit
          #endif
                    adcb      ~1,~+1~,1~,~2~
                    adca      ~@~
                    endm

;*******************************************************************************
; LSL for words

lsl.w               macro     Operand
                    mreq      1:Operand
                    lsl       ~1,~+1~,1~,~2~
                    rol       ~@~
                    endm

;*******************************************************************************

Copyright           macro     [SinceYear]
                    mdef      1,{:year}
          #ifparm ~1~ = {:year}
                    mset      1
          #else
                    mset      1,~1~-
          #endif
                    #Message  Copyright (c) ASPiSYS ~1~{:year}
                    fcs       'Copyright (c) ASPiSYS ~1~{:year}'
                    endm

;*******************************************************************************
; Some commonly-used OS11-related macros
;*******************************************************************************

;*******************************************************************************
; Give up current task's remaining timeslice if running under OS11

fNextTask           macro
          #ifdef _MTOS_
                    os        fNextTask
          #else
                    cli
                    nop
          #endif
                    endm

;*******************************************************************************
; Define one (or more) semaphore(s).  Skip already defined ones.

sema                macro     Sema1[,Sema2]*
                    mswap     1,:loop
          #ifndef ~1~
~1~                 exp       :index
MAXSEMAS            set       ~1~
          #endif
                    mtop      :n                  ;repeat for all parms
                    endm

;*******************************************************************************
; FCB (Form Constant Byte) with BCD value of the parameter constant (upto 99)

BCD                 macro     Constant[,Constant]*
                    mreq      1:Constant[,Constant]*
                    mswap     1,:loop
                    fcb       ~1~\10|{~1~\100/10<4}  ;;\100 to truncate high byte
                    mtop      :n
                    endm

;*******************************************************************************
; Define a Pascal-style string

StrPas              macro     'string text'
                    mset      #
                    mreq      1:'string text'
                    mstr      1
                    fcc       :1-2,~1~            ;length, string text
                    endm

;*******************************************************************************
; Fill a (normally) non-RAM memory range with specific value or address low byte

FillROM             macro     From,To[,Value]     ;Fill a ROM range with value
                    mreq      1,2:From,To[,Value]
                    org       ~1~                 ;beginning at specified location
                    mset      1,{~2~-~1~+1}       ;now 1 holds the number of bytes
          #ifnoparm ~3~
                    mdo                           ;place value (low byte of address)
                    fcb       {:pc&$FF(h)}
                    mloop     ~1~                 ;repeat with next location
                    mexit
          #endif
                    mdo                           ;place value (user-supplied)
                    fcb       ~3~
                    mloop     ~1~                 ;repeat with next location
                    endm

;*******************************************************************************
; Symbol to the left of macro call (if present) and :MEXIT internal variable
; are SET to the integer Log2 (log base two) of the given expression.
; Quick-n-dirty calculation of integer part of log2(n) for values upto 2^31-1
; Useful to get power from value (e.g., as when used with various prescalers.)
; With the optional ShiftLeftBits parameter, one can shift the result into the
; expected bit positions (e.g., within a larger bitmap).

#ifnomdef Log2
Log2                macro     Expr[,ShiftLeftBits]
                    mreq      1:Usage: Label @~0~ Expression[,ShiftLeftBits]
                    mdef      2,0
                    #temp     :loop-2
#ifz ~1~
                    #temp     :temp<{~2~}
          #ifparm ~label~
~label~             set       :temp
          #endif
                    mexit     :temp
#endif
                    mset      1,{~1~>1}
                    mtop
                    endm
#endif

;*******************************************************************************
; Macro for showing end-of-program statistics (to be updated as needed)

EndStats            macro     [Module Start Label]
          #ifincluded
                    mexit
          #endif
                    #Message  +-------------------------------------------------
                    #Message  | Statistics (from \@~mfilename~/~0~\@ macro)
                    #Message  +-------------------------------------------------
          #ifdef ?_OBJECT_?
                    mdef      1,?_OBJECT_?
          #endif
          #ifnb ~1~
                    #Message  | Module size...: {*-~1~} bytes
          #endif
                    #temp                         ;;initialize total to zero
                    mset      0
          #ifdef XRAM
                    #temp     XRAM_END-:XRAM+1    ;;count XRAM if available
                    mset      0,[includes XRAM]
          #endif
                    #temp     RAM_END-:RAM+1+:temp ;;add RAM
                    #Message  | Available RAM : {:temp} byte(s) ~text~
          #if :temp-REQUIRED_STACK < 0
                    mset      0,[WARNING] ~text~
          #endif
          #ifdef _MTOS_
                    #temp     :temp-{REQUIRED_STACK*MAXTASKS}
          #else
                    #temp     :temp-REQUIRED_STACK
          #endif
                    #Message  | Non-stack RAM : {:temp} byte(s) ~text~
          #if ROM_END-:ROM+1 >= 0
                    #Message  | Available ROM : {ROM_END-:ROM+1} byte(s)
          #endif
                    mset      0
          #ifdef XROM
                    #Message  | Available XROM: {XROM_END-:XROM+1} byte(s)
          #endif
          #ifdef NUMBER_OF_OS_CALLS
                    #Message  | Total OS calls: {NUMBER_OF_OS_CALLS} (of {MAX_OS_CALLS})
          #endif
                    #Message  | Macros called : {:totalmacrocalls}
                    #Message  | [#]PROCs used : {:proc}
                    #Message  | Max stack used: {:spmax}
                    #Message  +-------------------------------------------------
                    endm

;*******************************************************************************
                    #Exit
;*******************************************************************************
;                   Test various macro expansions
;*******************************************************************************

                    #ListOff
                    #Uses     mcu.inc
                    #ListOn

                    @MyDefaultDirectives

                    @Pin      LED,PORTA,0
                    @On       LED,x
                    @Off      LED,y

                    @BitNum   COP,1
                    @Bits     Flag,5,8            ;define Flag5..Flag8 bits & masks
                    @Bits     A,1,5,3             ;define A1..A5 starting from 3

                    @DelayXms 10
                    @DelayXms 20

                    fcb       :year\100,:month,:date  ;decimal date stored as is
                    @bcd      :year,:month,:date  ;decimal date stored as BCD

MyCopyright         @Copyright

                    @StrPas   'Hello World!'      ;a Pascal string
                    @StrPas   '.. etc ..'         ;and another one
                    @StrPas   and w/o quotes

                    @FillROM  $D000,$D0FF,$AA     ;fill range with $AA
                    @FillROM  $D100,$D1FF         ;fill range with low address byte

LogOf4096           @Log2     2*2048              ;Set symbol to log2 of following expression
                    #Message  Log2 returned: {LogOf4096}
