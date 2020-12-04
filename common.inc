;*******************************************************************************
;*           COMMON EQUATES -- MCU INDEPENDENT -- FOR ASM11 ASSEMBLER
;*******************************************************************************
;* Language  : Motorola/Freescale/NXP 68HC11 Assembly Language (aspisys.com/ASM11)
;* Status    : FREEWARE Copyright (c) 2020 by Tony Papadimitriou <tonyp@acm.org>
;* Original  : http://www.aspisys.com/code/hc11/common.html
;*******************************************************************************
; Dot-ending names (XXX.) indicates a unique bit mask (possibly multiple bits)
; Underscore surrounded names (_XXX_) indicates special system symbols such as
; CCR offsets, etc.
; Constants use uppercase (XXX)
; Variables use lowercase and underscores as needed (my_var).
; Pointers begin with a point/dot (.xxx).
; Code labels use CamelCase
; Local labels end with @@ (although ASM11 allows @@ to be anywhere inside label)
;*******************************************************************************

#ifmain ;-----------------------------------------------------------------------
                    #Fatal    This file can not be used stand-alone
#endif ;------------------------------------------------------------------------

NOT                 def       -1                  ;Used with XOR to get NOT result

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
; Restrict the value of a constant label between a minimum and a maximum
; printing a warning if the value is outside the allowed range

ConstMinMax         macro     Symbol[,MinValue][,MaxValue]
                    mreq      1:Symbol[,MinValue][,MaxValue]
                    mset      0,~2~<=~1,~<=~3~ [not {~1,~}]
          #ifb ~2~~3~
                    merror    No MinValue or MaxValue given
          #endif
          #ifnb ~2~
            #if ~1,~ < ~2~
                    #Warning  ~text~
            #endif
          #endif
          #ifnb ~3~
            #if ~1,~ > ~3~
                    #Warning  ~text~
            #endif
          #endif
                    endm

;*******************************************************************************
; Restrict the value of a constant label to one of the specified values
; printing a warning if the value is not in the given set

ConstValues         macro     Symbol,Value[,Value]*
                    mreq      1,2:Symbol,Value[,Value]*
          #ifndef ~1,~
                    mexit     ;;mstop     ~1,~ not yet defined
          #endif
                    mdo       2
          #if ~1,~ = ~{:mloop}.~
                    mexit
          #endif
                    mloop     :n
                    mset      0,~1,~
                    mdel      1
                    mset      #
                    #Warning  ~text~ ({~text~}) not in [~1~]
                    endm

;*******************************************************************************

VDD                 def       5000                ;MCU voltage in mV

;===============================================================================
#ifndef Hz
 #ifndef KHZ
  #ifndef MHZ
MHZ                 def       8                   ;default for most anything
   #ifdef __DK68HC11__
HZ                  def       9830400
   #endif
  #endif
 #endif
#endif
;-------------------------------------------------------------------------------
?                   macro     FromHz,ToHz
                    mreq      1,2:FromHz,ToHz
#ifdef BUS_~1~
                    mset      1,BUS_~1~
                    mset      2,BUS_~2~
#endif
#ifdef ~1~
          #if ~1~\1000 >= 500
~2~                 set       ~1~/1000+1
          #else
~2~                 set       ~1~/1000
          #endif
#endif
#ifparm ~1.1.4~ = BUS_
~1.5~               set       ~1~*4
#endif
                    endm
;-------------------------------------------------------------------------------
#ifdef BUS_MHZ
BUS_HZ              def       BUS_MHZ*1000000
BUS_KHZ             def       BUS_MHZ*1000
#endif
                    @?        HZ,KHZ
                    @?        KHZ,MHZ

MHZ                 def       8                   ;default crystal speed
KHZ                 def       MHZ*1000
HZ                  def       KHZ*1000

BUS_HZ              def       HZ/4
BUS_KHZ             def       BUS_HZ/1000
BUS_MHZ             def       BUS_KHZ/1000

MSEC_CYCLES         def       BUS_KHZ             ;number of cycles per millisecond
;-------------------------------------------------------------------------------
?                   macro
                    #temp     :index-1
Bit{:temp}.         equ       1<{:temp}
                    mtop      16
                    endm

                    @?                            ;Define Bit0 .. Bit15
;-------------------------------------------------------------------------------
; ISR (zero-based) offsets account for the return address of OS11 dispatcher's JSR
;-------------------------------------------------------------------------------
_DATA_              equ       0                   ;start of stacked data, if any
_CCR_               next      _DATA_,1
_B_                 next      _DATA_,1
_A_                 next      _DATA_,1
_X_                 next      _DATA_,2
_Y_                 next      _DATA_,2
_PC_                next      _DATA_,2

_D_                 equ       _B_,2               ;This is for loading D reversed
          ;-------------------------------------- ;Alternative (old style) names
DATA_               equ       _DATA_              ;start of stacked data, if any
PC_                 equ       _PC_,2
Y_                  equ       _Y_,2
X_                  equ       _X_,2
A_                  equ       _A_,1
B_                  equ       _B_,1
D_                  equ       _D_,2               ;This is for loading D reversed
CCR_                equ       _CCR_,1

;-------------------------------------------------------------------------------
; CCR bits (masks)
;-------------------------------------------------------------------------------

S.                  equ       %10000000           ;Stop inhibit
X.                  equ       %01000000           ;XIRQ disable
H.                  equ       %00100000           ;Half Carry
I.                  equ       %00010000           ;IRQ disable
N.                  equ       %00001000           ;Negative
Z.                  equ       %00000100           ;Zero
V.                  equ       %00000010           ;Overflow
C.                  equ       %00000001           ;Carry
          ;-------------------------------------- ;BPROT Bits
PTCON.              equ       Bit4.               ;Protect CONFIG programming
                    @bits     BPRT,0,3
          ;-------------------------------------- ;CONFIG Bits
NOSEC.              equ       Bit3.               ;NOSEC (no security option)
NOCOP.              equ       Bit2.               ;NOCOP in CONFIG register
ROMON.              equ       Bit1.               ;ROM is ON/enabled
EEON.               equ       Bit0.               ;EEPROM is ON/enabled
          ;-------------------------------------- ;OPTION Bits
ADPU.               equ       %10000000           ;A/D Power-Up
CSEL.               equ       %01000000           ;Charge Pump Clock Select
IRQE.               equ       %00100000           ;IRQ Select Edge Sensitive Only
DLY.                equ       %00010000           ;Delay out of STOP mode
CME.                equ       %00001000           ;Clock Monitor Enable
                    @bits     CR,0,1              ;COP Timer Rate Select
          ;-------------------------------------- ;SCI Bits
TIE.                equ       %10000000           ;SCI Transmitter Interrupt Enable
TCIE.               equ       %01000000           ;SCI Transmit Complete Interrupt Enable
RIE.                equ       %00100000           ;SCI Receiver Interrupt Enable
ILIE.               equ       %00010000           ;SCI Idle-Line Interrupt Enable
TE.                 equ       %00001000           ;SCI Transmitter Enable
RE.                 equ       %00000100           ;SCI Receiver Enable
RWU.                equ       %00000010           ;SCI Receiver Wakeup Control
SBK.                equ       %00000001           ;SCI Send Break

TDRE.               equ       %10000000           ;SCI Transmit Register Empty
TC.                 equ       %01000000           ;SCI Transmit Complete Flag
RDRF.               equ       %00100000           ;SCI Receive Register Full
IDLE.               equ       %00010000           ;SCI Idle Line Detected
OR.                 equ       %00001000           ;SCI Overrun Error
NF.                 equ       %00000100           ;SCI Noise Error
FE.                 equ       %00000010           ;SCI Framing Error
          ;-------------------------------------- ;SPI Bits
SPIE.               equ       %10000000           ;SPI Interrupt Enable
SPE.                equ       %01000000           ;SPI Enable
DWOM.               equ       %00100000           ;PortD OR Wired Mode
MSTR.               equ       %00010000           ;Master Mode Select
CPOL.               equ       %00001000           ;Clock Polarity
CPHA.               equ       %00000100           ;Clock Phase
                    @bits     SRP,0,1             ;SPI Clock Rate Select

SPIF.               equ       %10000000           ;SPI Transfer Complete Flag
WCOL.               equ       %01000000           ;Write Collision
MODF.               equ       %00010000           ;Mode Fault
          ;-------------------------------------- ;SPI Pins
MISO.               equ       Bit2.               ;Port D pins for manual SPI
MOSI.               equ       Bit3.
SCK.                equ       Bit4.
SS.                 equ       Bit5.

          #ifdef PORTD
SPI_MISO            @pin      PORTD,2
SPI_MOSI            @pin      PORTD,3
SPI_CLK             @pin      PORTD,4
SPI_SS              @pin      PORTD,5
          #endif
          ;-------------------------------------- ;ADCTL (A/D Control/Status) Bits
CCF.                equ       %10000000           ;Conversions Complete Flag
SCAN.               equ       %00100000           ;Continuous Scan Control
MULT.               equ       %00010000           ;Multiple/Single Channel Control
CD.                 equ       %00001000           ;Channel Select D
CC.                 equ       %00000100           ;Channel Select C
CB.                 equ       %00000010           ;Channel Select B
CA.                 equ       %00000001           ;Channel Select A
          ;-------------------------------------- ;PACTL (Pulse Accumulator Control) Bits
DDRA7.              equ       %10000000           ;Data Directive for PortA[7]
PAEN.               equ       %01000000           ;Pulse Accumulator System Enable
PAMOD.              equ       %00100000           ;Pulse Accumulator Mode
PEDGE.              equ       %00010000           ;Pulse Accumulator Edge Control
DDRA3.              equ       %00001000           ;Data Directive for PortA[3]
I4O5.               equ       %00000100           ;IC4/OC5 select
                    @bits     RTR,0,1             ;RTI Rate Select
          ;-------------------------------------- ;Timer Bits
TOF.                equ       %10000000
RTIF.               equ       %01000000
PAOVF.              equ       %00100000
PAIF.               equ       %00010000
          ;-------------------------------------- ;PIOC (Parallel IO Control) Bits
STAF.               equ       %10000000           ;STRA Interrupt Status Flag
STAI.               equ       %01000000           ;STRA Interrupt Enable Mask
CWOM.               equ       %00100000           ;Port C Wired-OR Mode
HNDS.               equ       %00010000           ;Handshake Mode
OIN.                equ       %00001000           ;Output or Input Handshake Select
PLS.                equ       %00000100           ;Pulse/Interlocked Handshake Operation
EGA.                equ       %00000010           ;Active Edge for STRA
INVB.               equ       %00000001           ;Invert STRB
          ;-------------------------------------- ;Vectors by name
Vsci                def       $FFD6
Vspi                def       $FFD8
Vpai                def       $FFDA
Vpao                def       $FFDC
Vrto                def       $FFDE
Vtic4               def       $FFE0
Vtoc5               def       Vtic4               ;alias for Vtic4
Vtoc4               def       $FFE2
Vtoc3               def       $FFE4
Vtoc2               def       $FFE6
Vtoc1               def       $FFE8
Vtic3               def       $FFEA
Vtic2               def       $FFEC
Vtic1               def       $FFEE
Vrti                def       $FFF0
Virq                def       $FFF2
Vxirq               def       $FFF4
Vswi                def       $FFF6
Villop              def       $FFF8
Vcop                def       $FFFA
Vcmf                def       $FFFC
Vreset              def       $FFFE
          ;-------------------------------------- ;Miscellaneous
BYTE                equ       1                   ;8-bit quantity
WORD                equ       2                   ;16-bit quantity
DWORD               equ       4                   ;32-bit quantity
QWORD               equ       8                   ;64-bit quantity

ERASED_STATE        def       $FFFF               ;[E]EPROM Erased State
JMP_OPCODE          equ       $7E                 ;JMP extended opcode

          #ifdef DEBUG
BPS_RATE            def       125                 ;for SIM11x debugging, use maximum speed
          #endif
BPS_RATE            def       96                  ;9600 bps for the SCI

;*******************************************************************************
; Displays an error message if the assembly-time condition is false
; (The condition should be written as to be compatible with the #IF directive)

assert              macro     Condition[,message]
                    mset      #','
          #if ~1~
                    mexit
          #endif
                    mdel      1                   ;;get rid of condition
                    mset      #                   ;;unify string message
                    mdef      1,Assertion failed (~1~) ;;default message
                    merror    ~1~
                    endm

;*******************************************************************************
; Macro(s)

SetChipSelects      macro     [[#]STACKTOP]       ;to be called first thing after reset
          #ifdef XRAM_END
                    mdef      1,#XRAM_END
          #else
                    mdef      1,#STACKTOP
          #endif
                    #temp     {RAM>8&$F0}|{REGS>12}
          #ifz :temp
                    clr       XINIT
          #else
                    lda       #:temp
                    sta       XINIT               ;Set register base
          #endif
#ifdef __ASPISYS__
          #if MHZ <= 16
                    clr       CSSTRH              ;No clock stretch
          #else
                    #Message  1 clock stretch for external EEPROM

                    lda       #1                  ;EEPROM 1 clock stretch
                    sta       CSSTRH              ;@>16MHz crystal (>4MHz bus)
          #endif
                    clr       CSGADR              ;RAM at $0000

                    lda       #%1                 ;32K RAM
                    sta       CSGSIZ

                    lda       #%01000101          ;CSIO1 Active High, 32K ROM at $8000
                    sta       CSCTL
#endif
                    lds       ~1~
                    endm

;*******************************************************************************
; Macro to dynamically execute another macro or instruction as one atomic
; operation (i.e., with interrupts temporarily disabled).
; When done with the operation, it restores interrupts to their original status.
; You could use with any macro.  For example, using an ADD.L (add longs) macro
; you could perform the addition with interrupts disabled, even if the original
; macro does not provide for atomicity of operation(s).  Example:
;                   @atomic   add.l,Number1,Number2,Answer

Atomic              macro     [@]CmdToDo[,Parameter]*
                    mreq      1:[@]CmdToDo[,Parameter]*
          #ifparm ~1.1.1~ = @
                    mset      1,@~1~              ;;make macro a @@ call
          #endif
                    mswap     0,1
                    mdel      1
                    mset      #
                    pshcc
                    sei

                    ~text~    ~1~

                    pulcc
                    endm

;*******************************************************************************
; Find a character in the string pointed to by X, and point right past it
; The user's original RegA is not destroyed

SkipChar            macro     [[#]Character]
                    mset      #
          #ifnb ~1~
                    psha
                    @@_lda_   ~1~
          #endif
Loop$$$
                    tst       ,x                  ;;is it end of ASCIZ string?
                    beq       Done$$$
                    cmpa      ,x
                    beq       Done$$$             ;;char found, done
                    inx
                    bra       Loop$$$             ;;repeat until end of ASCIZ string
Done$$$             inx
          #ifnb ~1~
                    pula
          #endif
                    endm

;*******************************************************************************
; Skip over the current and following characters as long as they match the target
; On completion, X -> first non-target character
; The user's original RegA is not destroyed

EatChar             macro     [[#]Character]
                    mset      #
          #ifnb ~1~
                    psha
                    @@_lda_   ~1~
          #endif
Loop$$$             cmpa      ,x                  ;;compare first char
                    bne       Done$$$             ;;if not same, CBEQ won't work
                    inx
                    bra       Loop$$$
Done$$$
          #ifnb ~1~
                    pula
          #endif
                    endm

;*******************************************************************************
; Find (in RegA) the minimum between RegA and operand

MinA                macro     Operand
                    mset      #
                    mreq      1:Operand
                    cmpa      ~1~
                    bls       Done$$$
                    lda       ~1~
Done$$$
                    endm

;*******************************************************************************
; Find (in RegA) the maximum between RegA and operand

MaxA                macro     Operand
                    mset      #
                    mreq      1:Operand
                    cmpa      ~1~
                    bhs       Done$$$
                    lda       ~1~
Done$$$
                    endm

;*******************************************************************************
; Point X to a TableEntry given its zero-based index, table address, and
; TableEntry bytesize.  Index is optional (in case it is already loaded in RegA)

TblOfs              macro     [[#]Index],[#]Table,[#]TableEntrySize
                    mreq      2,3:[[#]Index],[#]Table,[#]TableEntrySize
                    @@_not_x_ ~2~
                    pshd
                    @@_lda_   ~1~                 ;A = index
                    #temp
          #ifparm ~3.1.1~ = #                     ;;special case optimization
            #!if ~#3~ = 1                         ;;for defined immediate of size 1
                    #temp     1                   ;mark action taken
            #endif
            #ifnum ~#3~                           ;;for numeric immediate
              #if ~#3~ = 1                        ;;of size 1
                    #temp     1                   ;mark action taken
              #endif
            #endif
          #endif
          #ifz :temp
                    ldb       ~3~                 ;B = bytesize of TableEntry
                    mul                           ;D = offset into table
          #else
                    tab
                    clra                          ;D = offset into table
          #endif
                    addd      ~2~
                    xgdx                          ;X -> TableEntry
                    puld
                    endm

;*******************************************************************************
; Macro to use XRAM if XRAM has been defined, RAM otherwise

XRAM                macro
                    #RAM
          #ifdef XRAM
                    #XRAM
          #endif
                    endm

;*******************************************************************************
; EQU but it also copies the size of the original label
; (a single label is expected as parameter, no expressions)

equ                 macro     Label
                    mset      #
                    mreq      1:Label @~0~ Label
                    mset      0,Label expected, found
          #ifnum ~1~
                    mstop     ~text~ number (~1~)
          #endif
          #ifstr ~1~
                    mstop     ~text~ string (~1~)
          #endif
          #ifnb ~'()[]+-*/\,<>^&|'2~
                    mstop     ~text~ expr (~1~)
          #endif
~label~             set       ~1~,::~1~
                    endm

;*******************************************************************************
; Macro to allocate a variable to #RAM or #XRAM (if XRAM defined) based on size
; [Smaller variables (upto LONG size) go into #RAM, everything else into #XRAM]

Var                 macro     Size[,MaxSizeForRAM]
          #ifnb ~2~
                    mset      0,~2~               ;;set new MaxSizeForRAM default
            #ifb ~1~
                    mexit                         ;;special case only to define MaxSizeForRAM
            #endif
          #endif
                    mdef      0,4                 ;;startup MaxSizeForRAM default
                    #push
                    #RAM
          #ifdef XRAM                             ;;when XRAM is defined
            #if ~#1~ > ~text~                     ;;anything above ~TEXT~ goes to XRAM
                    #XRAM
            #endif
            #if :RAM+~#1~ > RAM_END               ;;if RAM is full or not enough, use XRAM
                    #XRAM
            #endif
          #endif
~label~             set       *,~1~
                    rmb       ~1~
                    #pull
                    endm

;*******************************************************************************
; Repeat a number of instructions a constant number of times
; Example:          @rep      2,lsla,rolx         ;shift left XA twice

Rep                 macro     Times,Instruction1[,Instruction2]*
                    mreq      1,2:Times,Instruction1[,Instruction2]*
          #ifz ~1~
                    mexit                         ;;nothing to do
          #endif
                    mdo       2
          #ifparm ~{:mloop}.1.2~ = @@             ;;convert @@macro to @macro
                    mset      :mloop,~{:mloop}.2~
          #endif
          #ifparm ~{:mloop}.1.1~ = @              ;;convert @macro to @@macro
                    mset      :mloop,@~{:mloop}.~
          #endif
                    ~{:mloop}.~
                    mloop     :n
                    mtop      ~1~
                    endm

;*******************************************************************************
; Checks the passed parameter and issues an error if it is X-indexed
; (This is meant to be called from other macros -- not to be called directly)

_not_x_             macro
                    mset      #
          #ifb ~1~
                    mexit                         ;;nothing to do
          #endif
          #ifparm ~'~,1~'.{:1}~ = x
                    mstop     X-index not allowed (~1~)
          #endif
                    endm

;*******************************************************************************
; Checks the passed parameter and issues an error if it is Y-indexed
; (This is meant to be called from other macros -- not to be called directly)

_not_y_             macro
                    mset      #
          #ifb ~1~
                    mexit                         ;;nothing to do
          #endif
          #ifparm ~'~,1~'.{:1}~ = y
                    mstop     Y-index not allowed (~1~)
          #endif
                    endm

;*******************************************************************************
; Checks the passed parameter and issues an error if it is immediate mode
; (This is meant to be called from other macros -- not to be called directly)

_not_#_             macro
          #ifb ~1~
                    mexit                         ;;nothing to do
          #endif
          #ifparm ~#~
                    mstop     Immediate mode not allowed (~1~)
          #endif
                    endm

;*******************************************************************************
; Checks the passed parameter and issues an error if it is NOT immediate mode
; (This is meant to be called from other macros -- not to be called directly)

_#_                 macro
          #ifb ~1~
                    mexit                         ;;nothing to do
          #endif
          #ifb ~#~
                    mstop     Immediate mode expected (~1~)
          #endif
                    endm

;*******************************************************************************
; Check if no size info exists for give symbol (parameter)

_nosize_            macro
                    mset      #
                    mset      0,mstop [~00~] No size (~1~)
          #ifnb ~#~
                    ~text~ [immediate]
          #endif
          #ifb ~1,~
                    ~text~ [blank]
          #endif
          #ifnum ~1,~
                    ~text~ [numeric]
          #endif
          #ifndef ~1,~
                    ~text~ [undefined]
          #endif
          #ifz ::~1,~
                    ~text~ [constant]
          #endif
                    endm

;*******************************************************************************
; Check if all given non-immediate mode operands have the same size
; (Skip immediate mode ones, and constants -- labels with size zero)
; (This is meant to be called from other macros -- not to be called directly)

_samesize_          macro     Operand1,Operand2[,Operand]*
                    mreq      1,2:Operand1,Operand2[,Operand]*
                    mset      0
                    mdo
                    mswap     1,{:mloop}
          #ifb ~#~
            #ifnb ~1,~
                    @@_nosize_ ~1~
              #ifb ~text~
                    mset      0,~1~
              #endif
              #if ::~text,~ <> ::~1,~
                    mstop     \@~text~\@ ({::~text,~}) size <> \@~1~\@ ({::~1,~})
              #endif
            #endif
          #endif
                    mloop     :n
                    endm

;*******************************************************************************
; Check if size exists, and optionally if it is one of the expected ones
; (This is meant to be called from other macros -- not to be called directly)

_size_              macro     Label[,ExpectedSize]*
                    @@_nosize_ ~1~
                    mdo       2
          #ifnb ~{:mloop}.~
            #if ::~1,~ = ~{:mloop}.~
                    mexit
            #endif
          #endif
                    mloop     :n
                    mswap     0,1
                    mdel      1
                    mset      #
                    mstop     \@~text,~\@ size {::~text,~} not in (~1~)
                    endm

;*******************************************************************************
; Optional LDA: Do a LDA instruction but only if the parameter is non-blank
; (This is meant to be called from other macros -- not to be called directly)

_lda_               macro
                    mset      #
          #ifb ~1~
                    mexit
          #endif
          #ifparm ~#~
            #!ifz ~#1~
                    clra
                    mexit
            #endif
          #endif
                    lda       ~1~
                    endm

;*******************************************************************************
; Optional STA: Do a STA instruction but only if the parameter is non-blank
; (This is meant to be called from other macros -- not to be called directly)

_sta_               macro
          #ifnb ~@~
                    sta       ~@~
          #endif
                    endm

;*******************************************************************************
; LDA & CMP combo but it optimizes LDA to CLRA and eliminates CMP if the
; corresponding effective value is #0
; (This is meant to be called from other macros, but it may be called directly)

_ldacmp_            macro     [#]Operand1,[#]Operand2
          #ifparm ~#~
          #ifdef ~#1~
                    mset      1,~#~{~#1~}
          #endif
          #endif
          #ifparm ~1~ = #0
                    clra
          #else
                    lda       ~1~
          #endif
                    mswap     1,2
          #ifparm ~#~
          #ifdef ~#1~
                    mset      1,~#~{~#1~}
          #endif
          #endif
                    cmpa      ~1~                 ;;needed even when #0 to adjust Carry
                    endm

;*******************************************************************************
; Swap two byte-size variables (does not protect RegA) - Primitive macro
; (This is meant to be called from other macros, but it may be called directly)

_swap_              macro     Variable1,Variable2
                    lda       ~1~
                    psha
                    lda       ~2~
                    sta       ~1~
                    pula
                    sta       ~2~
                    endm

;*******************************************************************************
; Swap two same-size variables

swap.s              macro     Variable1,Variable2
                    mreq      1,2:Variable1,Variable2
                    @@_samesize_ ~@~
                    psha
                    mdo
                    @@_swap_, ~1,~+{:mloop-1}~,1~ ~2,~+{:mloop-1}~,2~
                    mloop     ::~1,~
                    pula
                    endm

;*******************************************************************************
; Get A from the location pointed to by the parm, while bumping up that pointer

GetNextA            macro     Pointer
                    mreq      1:Pointer
          #ifparm ~#~
                    mstop     Pointer (~1~) must be a word variable
          #endif
                    @@_not_x_ ~@~
                    ldx       ~@~
                    lda       ,x
                    inx
                    stx       ~@~
                    endm

;*******************************************************************************
; Put A into the location pointed to by the parm, while bumping up that pointer

PutNextA            macro     Pointer
                    mreq      1:Pointer
          #ifparm ~#~
                    mstop     Pointer (~1~) must be a word variable
          #endif
                    @@_not_x_ ~@~
                    ldx       ~@~
                    sta       ,x
                    inx
                    stx       ~@~
                    endm

;*******************************************************************************
; Load Effective Address in RegX

lea                 macro
                    mset      #
          #ifb ~1~
                    mexit                         ;;no parm, nothing to do
          #endif
          #ifparm ~#~
                    ldx       ~1~                 ;;immediate value load as is
                    mexit
          #endif
          #ifparm ~1.1.1~ = .                     ;;dot-beginning symbols treat as pointers
                    ldx       ~1~
                    mexit
          #endif
          #ifparm ~1.1.2~ = ?.                    ;;dot-beginning file-local symbols treat as pointers
                    ldx       ~1~
                    mexit
          #endif
    #ifparm ~,1~                                  ;;indexed modes
          #ifparm ~,1~ = ,x
            #ifnb ~1,~
                    @aix      #~1,~               ;;X-indexed add possible offset
            #endif
                    mexit
          #endif
                    ldx       ~1~                 ;;any other index treat as pointer (regardless of name convention)
                    mexit
    #endif
                    ldx       #~1~                ;;everything else is fixed RAM
                    endm

;*******************************************************************************
; Find the Greatest Common Divisor (GCD) of two constants.
; Answer is placed in LABEL (if present) and :MEXIT variable

gcd                 macro     a,b
          #ifz ~1~
                    mexit     1
          #else ifz ~2~
                    mexit     1
          #endif
          #if ~1~ = ~2~
              #ifparm ~label~
~label~             set       ~1~
              #endif
                    mexit     ~1~
          #else if ~1~ > ~2~
                    mset      1,{~1~-{~2~}}
                    mtop
          #endif
                    mset      2,{~2~-{~1~}}
                    mtop
                    endm

;*******************************************************************************
; Find the Least Common Multiple (LCM) of two constants. Prerequisite macro: GCD
; Note: To avoid overflow, first divides, then multiplies. GCD certainly divides
;       both numbers, so no problem doing so, and it's more efficient.
;       Answer is placed in LABEL (if present) and :MEXIT variable

lcm                 macro     a,b
                    @@gcd     ~@~
          #ifparm ~label~
~label~             set       {~1~}/:mexit*{~2~}
          #endif
                    mexit     {~1~}/:mexit*{~2~}
                    endm

;*******************************************************************************
; Vector assignment
; (Places vector in #VECTORS segment, then returns to current segment)

Vector              macro     Vvector,ISR_Address
          #ifb ~2~
            #ifndef ~1~
                    mexit                         ;avoids error on empty but undefined vectors
            #endif
          #endif
                    mdef      2,AnRTI
                    #push
                    #VECTORS
                    #ppc
                    org       ~1~
                    dw        ~2~
                    org       :ppc
                    #pull
                    endm

;*******************************************************************************
; PUSH any variable onto the stack. If size is missing, size one is assumed.
; (All registers are destroyed)

;*******************************************************************************
; PUSH any variable onto the stack. If 'newname' given the variable will then be
; assigned the name inside the string.
; Constants require a size as it cannot be determined automatically.
; (All registers may be destroyed)

PushV               macro     variable[ SizeOf(var)][ 'newname']
                    mset      #' '
                    mset      0                   ;;cancel any stack names
                    mreq      1:variable[ SizeOf(var)][ 'newname']
          #ifstr ~{:nn}.~                         ;;if last parm is a string
                    mset      0,~{:nn}.~          ;;it is the stack name
                    mset      0,~text.2.{:text-2}~ ;;after removing quotes
                    mdel      :nn                 ;;parm no longer needed
          #endif
          #ifnb ~#~                               ;;immediate mode
                    mreq      1,2:Constant SizeOf(Constant)
            #if ~2~ < 0
                    merror    Negative size (~1~): ~2~
            #endif
                    #ifnb ~text~
                    mset      0,~text~,~2~
                    #endif
                    #temp
                    mdo
            #ifz ~#1~>{:mloop-1*8}&$FF
              #ifz :temp
                    #temp     1
                    clra
              #endif
            #else
                    #temp
                    lda       ~1~>{:mloop-1*8}&$FF
            #endif
            #if :mloop = ~2~
                    psha      ~text~
            #else
                    psha
            #endif
                    mloop     ~2~
            #if ~#1~ >= 0
              #ifnz ~#1~>{:mloop*8}
                    merror    Constant ~#1~ ({~#1~}) bigger than ~2~ byte(s)
              #endif
            #else
              #ifnz -{~#1~(h)}>{:mloop*8-1}
                    merror    Constant ~#1~ ({~#1~}) bigger than ~2~ byte(s)
              #endif
            #endif
                    #Message  ~0~ ~1~ ({~2~*8}-bit)
                    mexit
          #endif
          #ifnb ~1,~
            #ifnz ::~1,~
                    mdef      2,{::~1,~}
            #endif
          #endif
                    mdef      2,1
          #if ~2~ < 0
                    merror    Negative size (~1~): ~2~
          #endif
                    #Message  ~0~ ~1~ ({~2~*8}-bit)
                    #ifnb ~text~
                    mset      0,~text~,~2~
                    #endif
          #if ~2~ = 1                             ;;single-byte variable case
          #ifb ~1.1.1~ = .                        ;;but not for pointers
            #ifparm ~'~,1~'.1.3~ = ,sp            ;;for SP and SPX only
              #iftos ~1,~                         ;;if at TOS, special case
                    pula                          ;;A = TOS value
                    psha                          ;;put it back the way it was
                    psha      ~text~
                    mexit
              #endif
            #endif
                    lda       ~1~
                    psha      ~text~
                    mexit
          #endif
          #endif
                    #push                         ;;save current X offset etc.
          #ifb ~,1~ = ,x                          ;;only if not X-indexed
                    @@lea     ~1~                 ;;HX -> variable
                    #x                            ;;zero X offsets
          #else
                    #x        ~1,~                ;;make use of offset
          #endif
                    mdo
                    lda       {~2~-:mloop},x
          #if :mloop = ~2~
                    psha      ~text~
          #else
                    psha
          #endif
                    mloop     ~2~
                    #pull                         ;;restore current X offset etc.
          #ifspauto
                    #spadd    ~2~                 ;;needed because of #PUSH/#PULL
          #endif
                    endm

;*******************************************************************************
; PULL any variable from the stack. If size is missing, size one is assumed.
; (All registers may be destroyed)

PullV               macro     Variable[ SizeOf(Variable)]
                    mset      #' '
                    mreq      1:Variable[ SizeOf(Variable)]
                    @@_not_#_ ~1~
          #ifnz ::~1,~
                    mdef      2,{::~1,~}
          #endif
                    mdef      2,1
          #if ~2~ < 0
                    merror    Negative size (~1~): ~2~
          #endif
                    #Message  ~0~ ~1~ ({~2~*8}-bit)
          #if ~2~ = 1                             ;;single-byte variable case
          #ifb ~1.1.1~ = .                        ;;but not for pointers
            #ifparm ~'~,1~'.1.3~ = ,sp            ;;for SP and SPX only
              #iftos ~1,~                         ;;if at TOS, special case
                    mstop     ~0~ to same stack location!
              #endif
            #endif
                    pula
                    sta       ~1~
                    mexit
          #endif
          #endif
                    #push                         ;;save current X offset etc.
          #ifb ~,1~ = ,x                          ;;only if not X-indexed
                    @@lea     ~1~                 ;;HX -> variable
                    #x                            ;;zero X offsets
          #else
                    #x        ~1,~                ;;make use of offset
          #endif
                    mdo
                    pula
                    sta       {:mloop-1},x
                    mloop     ~2~
                    #pull                         ;;restore current X offset etc.
          #ifspauto
                    #spadd    -~2~                ;;needed because of #PUSH/#PULL
          #endif
                    endm

;*******************************************************************************
; A group of macros for use with any sized variable
;*******************************************************************************

;*******************************************************************************
; Clear any sized variable

ClrVar              macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
          #ifb ~,1~
                    mset      1,~#1~
            #if ::~1,~ < 256
              #ifz ]{~1~-1}
                    pshx
                    ldx       #::~1~
Loop$$$
                    clr       ~1~-1,x
                    dex
                    bne       Loop$$$
                    pulx
                    mexit
              #endif
                    pshx
                    psha
                    clra
                    ldx       #::~1~
Loop$$$
                    sta       ~1~-1,x
                    dex
                    bne       Loop$$$
                    pula
                    pulx
                    mexit
            #endif
          #endif
          #if ::~1,~ <= 8
            #ifnb ~,1~
                    @clr.s    ~1~
                    mexit
            #endif
          #endif
                    pshx
                    psha
                    lda       #::~1,~
                    ldx       ~1~
Loop$$$
                    clr       ,x
                    inx
                    deca
                    bne       Loop$$$
                    pula
                    pulx
                    endm

;*******************************************************************************
; Test for zero for any sized variable

zero?.s             macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
          #if :mloop = 1
                    lda       ~1,~+{:mloop-1}~,1~
          #else
                    ora       ~1,~+{:mloop-1}~,1~
          #endif
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; CLR for any sized variable

clr.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
          #ifb ~,1~
          #ifnz ]~1~
                    psha
                    clra
                    mdo
                    sta       ~1,~+{:mloop-1}~,1~
                    mloop     {::~1,~}
                    pula
                    mexit
          #endif
          #endif
                    mdo
                    clr       ~1,~+{:mloop-1}~,1~
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; INC for any sized variable

inc.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
                    inc       ~1,~+{::~1,~-:mloop}~,1~
          #if :mloop <> ::~1,~
                    bne       Done$$$
          #endif
                    mloop     ::~1,~
Done$$$
                    endm

;*******************************************************************************
; DEC (simulation) for any sized variable.

dec.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    psha
                    @@sub.s,  ~1~ #1 ~1~
                    @@cmp.s,  ~1~ #0
                    pula
                    endm

;*******************************************************************************
; COM for any sized variable

com.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
          #ifb ~,1~
          #ifnz ]~1~
                    psha
                    mdo
                    lda       ~1,~+{:mloop-1}~,1~
                    coma
                    sta       ~1,~+{:mloop-1}~,1~
                    mloop     {::~1,~}
                    pula
                    mexit
          #endif
          #endif
                    mdo
                    com       ~1,~+{:mloop-1}~,1~
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; ABS for any sized variable

abs.s               macro     Variable
                    mset      #
                    mreq      1:Variable
                    @@tst.b   ~1~
                    bpl       Done$$$
                    @@neg.s   ~1~
Done$$$
                    endm

;*******************************************************************************
; NEG for any sized variable

neg.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
          #if ::~1,~ > 1
                    mdo
                    com       ~1,~+{:mloop-1}~,1~
                    mloop     ::~1,~-1
          #endif
                    neg       ~1,~+{::~1,~-1}~,1~
          #if ::~1,~ > 1
                    mdo
                    bne       Done$$$
                    inc       ~1,~+{::~1,~-:mloop-1}~,1~
                    mloop     ::~1,~-1
Done$$$
          #endif
                    endm

;*******************************************************************************
; MOV for any sized variable (simpler version that does not test MSB)

move.s              macro     [#]Source,Destination
                    @copy.s   ~@~
                    endm

;*******************************************************************************
; MOV for any sized variable (but also tests MSB for negative)

mov.s               macro     [#]Source,Destination
                    @@move.s  ~@~
          #ifnb ~2~ = x+
            #if ::~1,~ < 2
                    mexit                         ;;single byte var, no need for TST
            #endif
                    tst       ~1~                 ;;var,x+ case
                    mexit
          #endif
          #if ::~2,~ < 2
                    mexit                         ;;single byte var, no need for TST
          #endif
                    tst       ~2~                 ;;all other cases
                    endm

;*******************************************************************************
; MOVA for any sized variable

mova.s              macro     [#]Source,Destination
                    mreq      1,2:Source,Destination
          #ifb ~#~
                    @@_samesize_ ~@~
          #else
                    @@_nosize_ ~2~
          #endif
                    mset      0                   ;;no CLRA used
                    mdo
          #ifnb ~#~
                    #temp                         ;;no optimization
            #ifdef ~#1~
              #ifz ~#1~>{::~2,~-:mloop*8}&$FF
                #ifb ~text~
                    clra
                #endif
                    mset      0,clra              ;;CLRA used
                    #temp     1                   ;;optimization
              #endif
            #endif
            #ifz :temp                            ;;if no optimization
                    lda       ~1~>{::~2,~-:mloop*8}&$FF
                    mset      0                   ;;no CLRA used
            #endif
          #else
                    lda       ~1,~+{:mloop-1}~,1~
          #endif
                    sta       ~2,~+{:mloop-1}~,2~
                    mloop     ::~2,~
                    endm

;*******************************************************************************
; Copy for any sized variable

copy.s              macro
                    psha
                    @@mova.s  ~@~
                    pula
                    endm

;*******************************************************************************
; LSL for any sized variable

lsl.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
          #if :mloop = 1
                    lsl       ~1,~+{::~1,~-:mloop}~,1~
          #else
                    rol       ~1,~+{::~1,~-:mloop}~,1~
          #endif
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; LSR for any sized variable

lsr.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
          #if :mloop = 1
                    lsr       ~1,~+{:mloop-1}~,1~
          #else
                    ror       ~1,~+{:mloop-1}~,1~
          #endif
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; ASR for any sized variable

asr.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
          #if :mloop = 1
                    asr       ~1,~+{:mloop-1}~,1~
          #else
                    ror       ~1,~+{:mloop-1}~,1~
          #endif
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; ROR for any sized variable

ror.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
                    ror       ~1,~+{:mloop-1}~,1~
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; ROL for any sized variable

rol.s               macro     Variable
                    mset      #
                    mreq      1
                    @@_nosize_ ~1~
                    mdo
                    rol       ~1,~+{::~1,~-:mloop}~,1~
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; ADD bytes
; Note: No destination required if you only need to add in RegA

add.b               macro     [#]Operand1,[#]Operand2[,Destination]
          #ifparm ~#~
            #ifdef ~#1~
                    mset      1,~#~{~#1~}
            #endif
          #endif
                    mswap     1,2
          #ifparm ~#~
            #ifdef ~#1~
                    mset      1,~#~{~#1~}
            #endif
          #endif
                    mswap     1,2
          #ifparm ~1~~2~ = #0#0
                    clra
                    @_sta_    ~3~
                    mexit
          #endif
          #ifparm ~1~ = #0
                    lda       ~2~
                    @_sta_    ~3~
                    mexit
          #endif
          #ifparm ~2~ = #0
                    lda       ~1~
                    @_sta_    ~3~
                    mexit
          #endif
                    lda       ~1~
                    adda      ~2~
                    @_sta_    ~3~
                    endm

;*******************************************************************************
; ADD bytes with Carry
; Note: No destination required if you only interested in the Carry flag

adc.b               macro     [#]Operand1,[#]Operand2[,Destination]
          #ifparm ~#~
            #ifdef ~#1~
                    mset      1,~#~{~#1~}
            #endif
          #endif
                    mswap     1,2
          #ifparm ~#~
            #ifdef ~#1~
                    mset      1,~#~{~#1~}
            #endif
          #endif
                    mswap     1,2
          #ifparm ~1~ = #0
                    clra
                    adca      ~2~
                    @_sta_    ~3~
                    mexit
          #endif
          #ifparm ~2~ = #0
                    clra
                    adca      ~1~
                    @_sta_    ~3~
                    mexit
          #endif
                    lda       ~1~
                    adca      ~2~
                    @_sta_    ~3~
                    endm

;*******************************************************************************
; ADD for any sized variable

add.s               macro     [#]Operand1,[#]Operand2,Destination
                    mreq      1,2,3:[#]Operand1,[#]Operand2,Destination
                    @@_samesize_ ~@~
                    mset      0,@@add.b,          ;;first time, do an ADD
                    mdo
          #ifparm ~#~
                    mset      0,~text~ ~1~>{:mloop-1*8}&$FF
          #else
                    mset      0,~text~ ~1,~+{::~3,~-:mloop}~,1~
          #endif
          #ifparm ~2.1.1~ = #
                    mset      0,~text~ ~2~>{:mloop-1*8}&$FF
          #else
                    mset      0,~text~ ~2,~+{::~3,~-:mloop}~,2~
          #endif
                    mset      0,~text~ ~3,~+{::~3,~-:mloop}~,3~
                    ~text~
                    mset      0,@@adc.b,          ;;next time(s), do an ADC
                    mloop     ::~3,~
                    endm

;*******************************************************************************
; SUB for any sized variable

sub.s               macro     [#]Operand1,[#]Operand2,Destination
                    mreq      1,2:[#]Operand1,[#]Operand2,Destination
                    @@_samesize_ ~@~
                    #temp
          #ifb ~3~
                    @@_nosize_ ~1~
                    #temp     ::~1,~
          #else
                    @@_nosize_ ~3~
                    #temp     ::~3,~
          #endif
                    mset      0,suba              ;;first time, do a SUBA
                    mdo
          #ifparm ~#~
                    lda       ~1~>{:mloop-1*8}&$FF
          #else
                    lda       ~1,~+{:temp-:mloop}~,1~
          #endif
          #ifparm ~2.1.1~ = #
                    ~text~    ~2~>{:mloop-1*8}&$FF
          #else
                    ~text~    ~2,~+{:temp-:mloop}~,2~
          #endif
          #ifnb ~3~
                    sta       ~3,~+{:temp-:mloop}~,3~
          #endif
                    mset      0,sbca              ;;next time(s), do a SBCA
                    mloop     :temp
                    endm

;*******************************************************************************
; AND for any sized variable

and.s               macro     [#]Operand1,[#]Operand2,Destination
                    mreq      1,2,3:[#]Operand1,[#]Operand2,Destination
                    @@_samesize_ ~@~
                    mdo
          #ifnb ~#~
                    @@_lda_   ~1~>{::~3,~-:mloop*8}&$FF
          #else
                    lda       ~1,~+{:mloop-1}~,1~
          #endif
          #ifnb ~2.1.1~ = #
                    and       ~2~>{::~3,~-:mloop*8}&$FF
          #else
                    and       ~2,~+{:mloop-1}~,2~
          #endif
                    sta       ~3,~+{:mloop-1}~,3~
                    mloop     ::~3,~
                    endm

;*******************************************************************************
; OR for any sized variable

ora.s               macro     [#]Operand1,[#]Operand2,Destination
                    mreq      1,2,3:[#]Operand1,[#]Operand2,Destination
                    @@_samesize_ ~@~
                    mdo
          #ifnb ~#~
                    @@_lda_   ~1~>{::~3,~-:mloop*8}&$FF
          #else
                    lda       ~1,~+{:mloop-1}~,1~
          #endif
          #ifnb ~2.1.1~ = #
                    ora       ~2~>{::~3,~-:mloop*8}&$FF
          #else
                    ora       ~2,~+{:mloop-1}~,2~
          #endif
                    sta       ~3,~+{:mloop-1}~,3~
                    mloop     ::~3,~
                    endm

;*******************************************************************************
; XOR for any sized variable

eor.s               macro     [#]Operand1,[#]Operand2,Destination
                    mreq      1,2,3:[#]Operand1,[#]Operand2,Destination
                    @@_samesize_ ~@~
                    mdo
          #ifnb ~#~
                    @@_lda_   ~1~>{::~3,~-:mloop*8}&$FF
          #else
                    lda       ~1,~+{:mloop-1}~,1~
          #endif
          #ifnb ~2.1.1~ = #
                    eor       ~2~>{::~3,~-:mloop*8}&$FF
          #else
                    eor       ~2,~+{:mloop-1}~,2~
          #endif
                    sta       ~3,~+{:mloop-1}~,3~
                    mloop     ::~3,~
                    endm

;*******************************************************************************
; CMP for any sized variable, without saving RegA

_cmp_.s             macro     [#]Operand1,[#]Operand2
                    mreq      1,2:[#]Operand1,[#]Operand2
                    @@_samesize_ ~@~
          #ifparm ~1.1.1~~2.1.1~ = ##
                    mset      #
                    mstop     Both operands are immediate! \@~1~\@
          #endif
                    #temp     1
          #ifb ~#~
                    #temp     ::~1,~
          #endif
          #ifb ~2.1.1~ = #
                    #temp     ::~2,~
          #endif
                    mdo
          #ifnb ~#~
                    @@_lda_   ~1~>{:temp-:mloop*8}&$FF
          #else
                    lda       ~1,~+{:mloop-1}~,1~
          #endif
          #ifnb ~2.1.1~ = #
                    cmpa      ~2~>{:temp-:mloop*8}&$FF      ;;needed even if #0 to adjust Carry
          #else
                    cmpa      ~2,~+{:mloop-1}~,2~
          #endif
          #if :mloop <> :temp
                    bne       Done$$$
          #endif
                    mloop     :temp
Done$$$
                    endm

;*******************************************************************************
; CMP for any sized variable

cmp.s               macro     [#]Operand1,[#]Operand2
                    psha
                    @@_cmp_.s ~@~
                    pula
                    endm

;*******************************************************************************
; DIV for any sized variable (HX assumed already set with appropriate values)

div.s               macro     Variable
                    mstop     Needs porting to HC11 from 9S08
                    mset      #
                    mreq      1
                    @@_not_x_ ~1~
                    @@_nosize_ ~1~
                    #temp     1
                    mdo       {:temp}
                    lda       ~1,~+{:mloop-1}~,1~
                    idiv
                    sta       ~1,~+{:mloop-1}~,1~
                    mloop     ::~1,~
                    endm

;*******************************************************************************
; A general-purpose FOR loop wrapper
; Limits : Start, Stop and Step (if not immediate) have to match size of
;        : ControlVar (or you will get an error)
; Note(s): Use FORS for signed version
;        : Negative steps not allowed (will be treated as positive)
; Example: FOR i 1 5 ... MRESUME (where i is any variable declared earlier)

fors                macro     ControlVar [#]Start [#]Stop[ [#]PositiveStep]
                    mset      #
                    @for      ~1~
                    endm
;-------------------------------------------------------------------------------
for                 macro     ControlVar [#]Start [#]Stop[ [#]PositiveStep]
                    mset      #' '
                    mreq      1,2,3:ControlVar [#]Start [#]Stop[ [#]PositiveStep]
                    mdef      4,#1                ;;default step is one
                    @@_nosize_  ~1~               ;;need sized control variable
                    #temp     ::~1,~
                    mdo       2
          #ifnum ~{:mloop}.~
                    mset      :mloop,#~{:mloop}.~
          #endif
                    mloop     4
                    @@_samesize_ ~@~
                    @@copy.s, ~2~ ~1~
Loop$$$
                    @@cmp.s,  ~1~ ~3~
          #ifparm ~00~ = fors
                    !jgt      Done$$$             ;;signed version (FORS)
          #else
                    !jhi      Done$$$             ;;unsigned version (FOR)
          #endif
                    msuspend
                    psha
                    @@add.s,  ~1~ ~4~ ~1~
                    pula
                    !jmp      Loop$$$
Done$$$
                    endm

;*******************************************************************************
; Math-related

_FindStkMth_        macro     BitVersion
                    #temp
                    mdo       ~1~/8               ;;find the next highest bit version included (but not less than requested)
            #ifdef _STKMTH{:mloop*8}_
              #ifz :temp
                    #temp     :mloop*8            ;;found
              #endif
            #endif
                    mloop     8                   ;;highest possible is 64 (8x8)
            #ifz :temp
                    mstop     Include STKMTH{~1~}.SUB (or higher)
            #endif
                    mexit     :temp
                    endm
;-------------------------------------------------------------------------------
_StkMthMax_         macro     Expression
                    mset      #
                    mtrim     1
                    #temp
                    mdo
                    mset      0,~'=()+-*\/&|^><'{:mloop}~
                    mtrim     0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~text.1.1~~text.{:text}~ = []
                    mset      0,~text.2.{:text-2}~
          #endif
                    mset      0,~text,~           ;;remove possible index
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~text.1.1~ = #
                    mset      0,~text.2~          ;;remove # from number
          #endif
          #ifnb ~text~
            #ifnb ~text.1.1~ = .
                    #Warning  Pointers (~text~) have unknown size
            #endif
            #ifnum ~text~                         ;estimate byte size for numeric constant
              #ifnz ~text~>16
                #if 3 > :temp
                    #temp     3
                #endif
              #endif
              #ifnz ~text~>24
                #if 4 > :temp
                    #temp     4
                #endif
              #endif
            #endif
            #ifb ~text.1.1~ = :
              #ifnonum ~text~
                #ifdef ~text~
                  #if {::~text~} > :temp
                    #temp     ::~text~
                  #endif
                #endif
              #endif
            #endif
          #endif
                    mloop     {:1/2}              ;;max term/factor estimate
          #ifz :temp
                    #Warning  Undetermined bit-size (using 32-bit)
                    #temp     4
          #endif
;         #ifb \@~'*'~\@ = \@~1~\@                ;if any multiplication is present
;           #if :temp < 4
;                   #temp     :temp+1             ;give some more bits (normally double, but on average this is OK)
;           #endif
;         #endif
          #if :temp > 8                           ;for exceptional cases, enforce
                    #temp     8                   ;maximum available bit-size
          #endif
                    @_FindStkMth_ {:temp*8}
                    endm
;-------------------------------------------------------------------------------
; Default version -- works with signed/unsigned operands
; (In :MEXIT, it returns the actual bit-size used)

Eval                macro     Expression
                    mset      #
                    mtrim     1
                    @@_StkMthMax_ ~1~
                    #temp      :mexit
                    @@_FindStkMth_ {:temp}
                    #temp     :mexit
                    @@Eval{:temp} ~1~
                    mexit     {:temp}
                    endm
;-------------------------------------------------------------------------------
; Signed version -- gives warning to alert you if SIGNED was not defined
; (In :MEXIT, it returns the actual bit-size used)

EvalS               macro     Expression
                    mset      #
                    @@_signed_
                    @Eval     ~1~
                    endm
;-------------------------------------------------------------------------------
_Eval_              macro     [BitSize,]Expression (eg. [ans=](a+b)*(a-b)/2)
          #if :macronest = 1
                    mstop     Macro not to be called directly (eg. use @Eval32)
          #endif
          #ifb ~00~ = ~0~
                    mdef      1,32
                    mswap     0,1                 ;;bitsize now in ~ text ~
                    mdel      1
                    mset      #
                    #Message  --------------------------------------------------
                    #Message  Expr: ~1~
                    #Message  --------------------------------------------------
                    @@_FindStkMth_ ~text~
                    mset      0,{:mexit},~text~   ;;(bitsize to use, actual bitsize)
          #endif
                    mset      #
                    mreq      1:Expression (eg. [ans=](a+b)*(a-b)/2 -- spaces OK)
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifb ~00~ = ~0~
                    mtrim     1                   ;;remove all spaces
                    mset      #'='                ;;split on assignment (if any)
            #if :nn > 2
                    mstop     Too many assignment operators
            #endif
            #if :nn > 1
                    @@~0~     ~2~
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                    mset      9                   ;;assume signed (when SIGNED)
          #ifparm ~1.1.1~~1.{:1}~ = []
                    mset      9,unsigned
                    mset      1,~1.2.{:1-2}~
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
              #ifparm ~1.1.1~ = .                 ;;pointers ...
                #if ~text,~/8 <> ~text','2~/8
                    @@ResizeTOS #~text,~/8\,#~text','2~/8\,\,~9~
                    @@lea     ~1~
                    @@_?sei_  ~1~
                    @@pullv   ~1~ ~text','2~/8    ;;are resized and then pulled
                    @_?cli_   ~1~
                    mexit
                #endif
              #endif
              #ifdef ~1,~                         ;;if assignment var defined
                    @Save~text,~ ~1~              ;;save to it
                    mexit
              #endif
                    merror    Unknown variable \@~1~\@
            #endif
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifstr ~1~
            #if :1-2 > 4
                    mstop     String constant (~1~) too long
            #endif
                    @Load~text,~ #~1~
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~1.1.1~ = -                     ;;process leading negative
            #ifnum ~1~
                    @Load~text,~ #~1~             ;;numerics use immediate mode
                    mexit
            #endif
            #ifdef ~1.2~
              #ifz ::~1.2~
                    @Load~text,~ #~1~             ;;named constants use immediate mode
                    mexit
              #endif
            #endif
                    @~0~      #0~1~               ;;anything else, subtract from zero
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                    mset      #'+-|^'             ;;split terms
          #if :nn > 1
                    mdo       2
                    @@~0~     ~{:nn-:mloop+2}.2~  ;;process terms right to left
                    mloop     :nn
                    @@~0~     ~1~                 ;;the first term without operator
                    mdo       2
            #ifparm ~{:mloop}.1.1~ = +
                    #Message  Add~text,~
                    !jsr      StackAdd~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = -
                    #Message  Sub~text,~
                    !jsr      StackSub~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = |
                    #Message  Or~text,~
                    !jsr      StackOr~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = ^
                    #Message  Xor~text,~
                    !jsr      StackXor~text,~
;                   #spadd    -{~text,~/8}
            #endif
                    mloop     :nn
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                    mset      #'*\/&><'           ;;split factors
          #if :nn > 1
                    mdo       2
                    @@~0~     ~{:nn-:mloop+2}.2~  ;;process factors right to left
                    mloop     :nn
                    @@~0~     ~1~                 ;;the first factor without operator
                    mdo       2
            #ifparm ~{:mloop}.1.1~ = *
                    #Message  Mul~text,~
                    !jsr      StackMul~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = /
                    #Message  Div~text,~
                    !jsr      StackDiv~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = \
                    #Message  Mod~text,~
                    !jsr      StackMod~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = &
                    #Message  And~text,~
                    !jsr      StackAnd~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = >
                    #Message  Shr~text,~
                    !jsr      StackShr~text,~
;                   #spadd    -{~text,~/8}
            #endif
            #ifparm ~{:mloop}.1.1~ = <
                    #Message  Shl~text,~
                    !jsr      StackShl~text,~
;                   #spadd    -{~text,~/8}
            #endif
                    mloop     :nn
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~1.1.4~~1.{:1}~ = NEG()         ;;do NEG(ate) function
                    @@~0~     ~1.5.{:1-5}~
                    #Message  Neg~text,~
                    !jsr      StackNegate~text,~
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~1.1.4~~1.{:1}~ = ABS()         ;;do ABS(olute) function
                    @@~0~     ~1.5.{:1-5}~
                    #Message  Abs~text,~
                    !jsr      StackAbs~text,~
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~1.1.4~~1.{:1}~ = SQR()         ;;do SQR() function -- square
                    @@~0~     ~1.5.{:1-5}~
                    #Message  Sqr~text,~(TOS)
                    tsx
                    !jsr      StackLoad~text,~
                    !jsr      StackMul~text,~
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifparm ~1.1.1~~1.{:1}~ = ()            ;;process parenthesized sub-expression
                    @~0~      ~1.2.{:1-2}~
                    mexit
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
                    mset      9                   ;;assume signed (when SIGNED)
          #ifparm ~1.1.1~~1.{:1}~ = []
                    mset      9,unsigned
                    mset      1,~1.2.{:1-2}~
          #endif
          ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
          #ifnum ~#1~
                    mset      1,#~#1~             ;;numerics use immediate mode
          #endif
          #ifparm ~1.1.2~ = -:
                    mset      1,#~1~              ;;negative internal symbol use immediate
          #endif
          #ifparm ~1.1.1~ = :
                    mset      1,#~1~              ;;positive internal symbol use immediate
          #endif
          #ifb ~,1~
            #ifdef ~#1~
              #ifz ::~#1~
                    mset      1,#~#1~             ;;named constants use immediate
              #endif
            #endif
          #endif
          #ifparm ~1.1.1~ = .                     ;;pointers ...
            #if ~text','2~/8 <> ~text,~/8
                    @@_?sei_  ~1~
                    @@pushv   ~1~ ~text','2~/8    ;;are pushed and then resized
                    @@_?cli_  ~1~
                    @ResizeTOS #~text','2~/8\,#~text,~/8\,\,~9~
                    mexit
            #endif
          #endif
          #ifnb ~9~
                    mset      1,[~1~]
          #endif
                    @Load~text,~ ~1~              ;;anything else, load as is
                    endm
;-------------------------------------------------------------------------------
Eval8               macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval16              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval24              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval32              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval40              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval48              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval56              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
Eval64              macro
                    mset      #
                    @_Eval_   ~0.5~\,~1~
                    endm
;-------------------------------------------------------------------------------
StrMath             macro
          #ifnb ~1~
            #ifdef ~1,~
              #ifnz ::~1,~
                    @_DoStr   {::~1,~*8}\,~1~\,~2~
                    mexit
              #endif
            #endif
          #endif
                    mset      #
                    mtrim     1
                    @@_StkMthMax_ ~1,~
                    #temp     :mexit
                    @_DoStr   {:temp}\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str16               macro     [Number],[ResultString]
                    @_DoStr   16\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str24               macro     [Number],[ResultString]
                    @_DoStr   24\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str32               macro     [Number],[ResultString]
                    @_DoStr   32\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str40               macro     [Number],[ResultString]
                    @_DoStr   40\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str48               macro     [Number],[ResultString]
                    @_DoStr   48\,~1~\,~2~
                    endm
;-------------------------------------------------------------------------------
Str64               macro     [Number],[ResultString]
                    @_DoStr   64\,~1~\,~2~
                    endm

;*******************************************************************************
; Show the current value and size of the specified symbols

ShowSym             macro     Symbol[,Symbol]*
                    #Message  --------------------------------------------------
                    mdo
                    mswap     1,:mloop
          #ifdef ~1~
                    #Message  Symbol \@~1~\@~'...................'.1.{20-:1}~ {~1~} (size: {::~1~})
          #endif
                    mloop     :n
                    endm

;*******************************************************************************
; Adds FCC string to end of ROM going backwards

fcc                 macro     String
                    mset      #','
          #ifdef STRING_TOP
                    mdef      0,{STRING_TOP}
          #else
                    mdef      0,{ROM_END}
          #endif
                    #push
                    #DATA
                    #ppc
                    #temp     :n
                    mdo
                    org       ~text~
          #ifstr ~{:temp}.~
                    org       *-{:{:temp}-2}
          #else
                    org       *-1
          #endif
                    mset      0,{:pc}
                    fcc       ~{:temp}.~
                    #temp     :temp-1
                    mloop     :n
                    org       :ppc
                    #pull
                    mexit     ~text~
                    endm

;*******************************************************************************
; Adds FCS string to end of ROM going backwards

fcs                 macro     String
                    mset      #
                    @fcc      ~1~,0
                    endm

;*******************************************************************************
; BSET for Pin names

bset                macro     PinName             ;BSET for Pin names
                    mreq      1:PinName
          #ifz ]~1~
                    !bset     ~1~,#~1~.
          #else
                    pshx
                    ldx       #~1~
                    bset      ,x,#~1~.
                    pulx
          #endif
                    endm

;*******************************************************************************
; BCLR for Pin names

bclr                macro     PinName             ;BCLR for Pin names
                    mreq      1:PinName
          #ifz ]~1~
                    !bclr     ~1~,#~1~.
          #else
                    pshx
                    ldx       #~1~
                    bclr      ,x,#~1~.
                    pulx
          #endif
                    endm

;*******************************************************************************
;BRSET for Pin names

brset               macro     PinName,Address     ;BRSET for Pin names
                    mreq      1,2:PinName,Address
          #ifparm ~3~
                    !brset    ~@~
                    mexit
          #endif
          #ifz ]~1~
                    !brset    ~1~,#~1~.,~2~
                    mexit
          #endif
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    psha
                    lda       ~1~
                    anda      #~1~.
                    cmpa      #~1~.
                    pula
                    beq       ~2~
                    endm

;*******************************************************************************
; BRCLR for Pin names

brclr               macro     PinName,Address     ;BRCLR for Pin names
                    mreq      1,2:PinName,Address
          #ifparm ~3~
                    !brclr    ~@~
                    mexit
          #endif
          #ifz ]~1~
                    !brclr    ~1~,#~1~.,~2~
                    mexit
          #endif
          #ifparm ~2~ = *
                    mset      2,{*}
          #endif
                    psha
                    lda       ~1~
                    anda      #~1~.^$FF
                    pula
                    beq       ~2~
                    endm

;*******************************************************************************
; Adds a new OS call to table

AddOS               macro     Name[,AlternateLocalName]
                    mdef      2,?~1~
                    #push
                    #ROM
          #ifdef XROM
                    #XROM
          #endif
                    mdef      0,{:pc}
          #if :mindex = 1
                    org       :pc+{MAX_OS_CALLS*2} ;;make room for so many OS calls
          #endif
                    #ppc
                    org       ~text~
#ifndef OSCommands
OSCommands          exp       *
#endif
f~1~                exp       :mindex-1
                    dw        ~2~
                    mset      0,{:pc}
NUMBER_OF_OS_CALLS  set       :mindex             ;Number of opcodes defined (so far)
                    org       :ppc
          #if :mindex > 256
                    #Warning  Double byte opcodes, OS => OSW
          #endif
          #if :mindex > MAX_OS_CALLS
                    merror    {MAX_OS_CALLS} MAX_OS_CALLS, need {:mindex} (+{:mindex-MAX_OS_CALLS})
          #endif
                    #pull
;-------------------------------------------------------------------------------
                    endm

;*******************************************************************************
; Define FLAG names (FLAG for port, FLAG. for pin number, and FLAG_ for mask)
; to be used for defining bit (boolean) flags.

Flag                macro     [FlagBaseName (needed first time or to change name)]
          #ifb ~text~
            #ifb ~1~
                    mreq      1:[FlagBaseName (needed first time or to change name)]
            #endif
          #endif
          #ifnb ~1~
                    mset      0,~1~,1,0           ;;flag base name definition (starts with suffix 1)
            #ifb ~label~
                    mexit                         ;;exit if no label with flag name definition
            #endif
          #endif
                    #temp     ~text','3~
          #ifndef ~text,~~text','2~
                    #push
                    #RAM
~text,~~text','2~   rmb       1
                    #pull
          #endif
~label~             set       ~text,~~text','2~
~label~.            equ       1<~text','3~
                    #temp     :temp+1
          #if :temp < 8
                    mset      0,~text,~,~text','2~,{:temp}
          #else
                    #temp     ~text','2~+1
                    mset      0,~text,~,{:temp},0
          #endif
                    endm

;*******************************************************************************
; ASCII CONTROL CODES

NULL                def        0                  ;null
NUL                 def        0                  ;Second spelling for NULL
ASCII_SOH           def        1                  ;Start of Heading
ASCII_STX           def        2                  ;Start of Text
ASCII_ETX           def        3                  ;End of Text
EOT                 def        4                  ;End of Transmission
ASCII_ENQ           def        5                  ;Enquiry
ACK                 def        6                  ;Acknowledge
BELL                def        7                  ;Bell
BS                  def        8                  ;Backspace
TAB                 def        9                  ;Horizontal Tab
LF                  def       10                  ;Line Feed
ASCII_VT            def       11                  ;Vertical Tab
ASCII_FF            def       12                  ;Form Feed
CR                  def       13                  ;Carriage Return
ASCII_SO            def       14                  ;Shift Out
ASCII_SI            def       15                  ;Shift In
ASCII_DLE           def       16                  ;Data Link Escape
ASCII_DC1           def       17                  ;Device Control 1
XON                 def       17                  ;XON
ASCII_DC2           def       18                  ;Device Control 2
ASCII_DC3           def       19                  ;Device Control 3
XOFF                def       19                  ;XOFF
ASCII_DC4           def       20                  ;Device Control 4
ASCII_NAK           def       21                  ;Negative acknowledge
ASCII_SYN           def       22                  ;Synchronous idle
ASCII_ETB           def       23                  ;End of Transmission Block
ASCII_CAN           def       24                  ;Cancel
ASCII_EM            def       25                  ;End of Medium
ASCII_SUB           def       26                  ;CTRL-Z
CTRL_Z              def       26                  ; -//-
ESC                 def       27                  ;Escape
ASCII_FS            def       28                  ;File Separator
ASCII_GS            def       29                  ;Group Separator
ASCII_RS            def       30                  ;Record Separator
ASCII_US            def       31                  ;Unit Separator

DDR                 def       1                   ;standard DDR offset
;-------------------------------------------------------------------------------
;                             Miscellaneous
;-------------------------------------------------------------------------------
SPEED_SIZE          def       2                   ;1 = SPEED, 2 = SIZE optimization
                    @ConstMinMax SPEED_SIZE,1,2

BYTE                def       1                   ;8-bit quantity
WORD                def       2                   ;16-bit quantity
POINTER             def       3                   ;24-bit (paged) pointer
DWORD               def       4                   ;32-bit quantity
QWORD               def       8                   ;64-bit quantity

S_BYTE              def       0,1                 ;size of 8-bit quantity
S_WORD              def       0,2                 ;size of 16-bit quantity
S_POINTER           def       0,2                 ;size of pointer: 16-bit
S_DWORD             def       0,4                 ;size of 32-bit quantity
S_QWORD             def       0,8                 ;size of 64-bit quantity

MAXERROR            def       0                   ;used for error codes

ERASED_STATE        def       -1                  ;default [E]EPROM/Flash Erased State
REQUIRED_STACK      def       50                  ;required stack bytes

MAX_OS_CALLS        def       200                 ;default room for OS calls (68HC11)

;*******************************************************************************
; Various Checks

          #!ifnz REGS&$0FFF
                    #Error    REGS ({REGS(h)}) not in format $X000 (edit ?11E?.INC)
          #endif

?                   macro     FromAddress,ToAddress
                    mdef      2,STACKTOP-REQUIRED_STACK
                    #VARIABLE ~1~ ~2~
                    #Message  Variable space in ~1~~' :'.{:1-2}~ {~1~(h)}-{~2~(h)} ({~2~-~1~+1} bytes)
                    endm

          #ifdef RAM1
                    @?        RAM1,RAM1_END
          #endif
          #ifdef XRAM
            #if STACKTOP > XRAM
                    @?        RAM,RAM_END
                    @?        XRAM
            #else
                    @?        RAM
                    @?        XRAM,XRAM_END
            #endif
          #else
                    @?        RAM
          #endif
          #ifdef VERSION
                    #Message  Firmware v{VERSION(2)}
          #endif
                    #Message  MCU Operating Voltage (VDD): {VDD(3)}V

          #ifz KHZ\1000
                    #Message  {KHZ/100(1)} MHz ({BUS_KHZ/100(1)} MHz bus) Cycle: {10000000/BUS_KHZ(1)} nsec
          #else
                    #Message  {KHZ(3)} MHz ({BUS_KHZ(3)} MHz bus) Cycle: {10000000/BUS_KHZ(1)} nsec
          #endif

?                   macro
          #ifdef ~1~
                    #Message  ~1~~'...:'.{:1-2}~ {~1~(h)}-{~1~_END(h)} ({~1~_END-~1~+1} bytes)
          #endif
                    endm

                    @?        RAM
          #ifdef XRAM
                    @?        XRAM
          #endif
                    @?        EEPROM
                    @?        ROM

          #ifnz REQUIRED_STACK
                    #Message  STACK : {STACKTOP-REQUIRED_STACK+1(h)}-{STACKTOP(h)} [REQUIRED_STACK: {REQUIRED_STACK} bytes]
          #else
                    #Message  REQUIRED_STACK: None!
          #endif

          #ifdef DEBUG
                    #Message  DEBUG/SIMULATOR mode (do NOT distribute)
          #endif
