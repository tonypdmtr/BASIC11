;*******************************************************************************
; Tom PitIL_tman's 6800 Tiny BASIC
; Reverse analyzed from (buggy) hexdump (TB68R1.tiff and TB68R2.tiff) at
; http://www.ittybittycomputers.com/IttyBitty/TinyBasic/index.htm
; by Holger Veit
;
; Note this might look like valid assembler, but possibly isn't.
; For reference only
;*******************************************************************************

                    #CaseOn

RAM_BASIC           def       $2000               ; stub to remove errors [DEBUG]
CR                  equ       13                  ; Carriage Return
LF                  equ       10                  ; Line Feed
PIA_REG             equ       $8004
ACIA_STATUS         equ       $FCF4
ACIA_DATA           equ       $FCF5

;*******************************************************************************
                    #RAM
;*******************************************************************************
                    org       32

start_prgm          rmb       2                   ; start of BASIC text (0x900)
end_ram             rmb       2                   ; end of available RAM
end_prgm            rmb       2                   ; end of BASIC text
top_of_stack        rmb       2                   ; top of return stack pointer location
basic_lineno        rmb       2                   ; save for current line number to be executed
il_pc               rmb       2                   ; program counter for IL code
basic_ptr           rmb       2                   ; pointer to currently executed BASIC byte
basicptr_save       rmb       2                   ; temporary save for basic_ptr
expr_stack          rmb       80                  ; lowest byte of expr_stack (0x30)
rnd_seed            rmb       2                   ; used as seed value for RND function
          ;-------------------------------------- ; note this is actually top of predecrementing expr_stack
var_tbl             rmb       52                  ; variables (A-Z), 26 words
LS_end              rmb       2                   ; used to store addr of end of LS listing,
          ;-------------------------------------- ; start of list is in basic_ptr
bp_save             rmb       2                   ; another temporary save for basic_ptr
x_save              rmb       2                   ; temporary save for X
IL_temp             rmb       2                   ; temporary for various IL operations
          ;-------------------------------------- ; used for branch to IL handler routine for opcode
lead_zero           rmb       1                   ; flag for number output and negative sign in DV
column_cnt          rmb       1                   ; counter for output columns (required for TAB in PRINT)
          ;-------------------------------------- ; if bit 7 is set, suppress output (XOFF)
run_mode            rmb       1                   ; run mode
                                                  ; = 0 direct mode
                                                  ; <> 0 running program
expr_stack_low      rmb       1                   ; low addr byte of expr_stack (should be 0x30)
expr_stack_x        rmb       1                   ; high byte of expr_stack_top (==0x00, used with X register)
expr_stack_top      rmb       1                   ; low byte of expr_stack_top (used in 8 bit comparisons)
il_pc_save          rmb       2                   ; save of IL program counter

;*******************************************************************************
                    #ROM
;*******************************************************************************
                    org       $100                ; cold start vector

CV                  proc
                    jsr       COLD_S              ; Do cold start initialization
WV                  jmp       WARM_S              ; do warm start (warm start vector)
IN_V                bra       *                   ; vector: get a character from input device into A (unimplemented)
OUT_V               bra       *                   ; print a character in A to output device (unimplemented)
BV                  nop                           ; test for break from input device, set C=1 if break
                                                  ; unimplemented - jump to break routine
                    clc                           ; note: at the end of the program, there are two
                    rts                           ; sample implementations for MIKBUG and MINIBUG

;*******************************************************************************
; Some standard constants
;*******************************************************************************

BSC                 fcb       $5F                 ; backspace code (should be 0x7f, but actually is '_')
LSC                 fcb       $18                 ; line cancel code (CTRL-X)
PCC                 fcb       $83                 ; CRLF padding characters
                                                  ; low 7 bits are number of NUL/0xFF
                                                  ; bit7=1: send 0xFF, =0, send NUL
TMC                 fcb       $80
SSS                 fcb       $20                 ; reserved bytes at end_prgm (to prevent return stack
                                                  ; underflow (spare area)
;******************************************************************************
; utility routines for BASIC (not called in interpreter code)
;******************************************************************************

;*******************************************************************************
; get the byte pointed to by X into B:A
;*******************************************************************************

peek                proc
                    lda       ,x
                    clrb
                    rts

;*******************************************************************************
; Put the byte in A into cell pointed to by X
;*******************************************************************************

poke                proc
                    sta       ,x
                    rts

;*******************************************************************************
; Interpreter jump table
;*******************************************************************************

il_jumptable        fdb       IL_BBR              ; 0x40-0x5f: backward branch
                    fdb       IL_FBR              ; 0x60-0x7f: forward_branch
                    fdb       IL_BC               ; 0x80-0x9f: string match branch
                    fdb       IL_BV               ; 0xa0-0xbf: branch if not variable
                    fdb       IL_BN               ; 0xc0-0xdf: branch if not number
                    fdb       IL_BE               ; 0xe0-0xff: branch if not eoln
                    fdb       IL_NO               ; 0x08: no operation
                    fdb       IL_LB               ; 0x09: push literal byte to TOS
                    fdb       IL_LN               ; 0x0a: push literal word to TOS
                    fdb       IL_DS               ; 0x0b: duplicate stack top
                    fdb       IL_SP               ; 0x0c: pop TOS into A:B
                    fdb       expr_pop_byte       ; 0x0d: undocumented: pop byte into
                    fdb       sub_177             ; 0x0e: undocumented: push TOS on return stack
                    fdb       sub_180             ; 0x0f: undocumented: pop return stack into TOS
                    fdb       IL_SB               ; 0x10: save BASIC pointer
                    fdb       IL_RB               ; 0x11: restore BASIC pointer
                    fdb       IL_FV               ; 0x12: fetch variable
                    fdb       IL_SV               ; 0x13: store variable
                    fdb       IL_GS               ; 0x14: save GOSUB line
                    fdb       IL_RS               ; 0x15: restore saved line
                    fdb       IL_GO               ; 0x16: goto
                    fdb       IL_NE               ; 0x17: negate
                    fdb       IL_AD               ; 0x18: add
                    fdb       IL_SU               ; 0x19: subtract
                    fdb       IL_MP               ; 0x1a: multiply
                    fdb       IL_DV               ; 0x1b: divide
                    fdb       IL_CP               ; 0x1c: compare
                    fdb       IL_NX               ; 0x1d: next BASIC statement
                    fdb       IL_NO               ; 0x1e: reserved
                    fdb       IL_LS               ; 0x1f: list program
                    fdb       IL_PN               ; 0x20: print number
                    fdb       IL_PQ               ; 0x21: print BASIC string
                    fdb       IL_PT               ; 0x22: print tab
                    fdb       IL_NL               ; 0x23: new line
                    fdb       IL_PC               ; 0x24: print literal string
                    fdb       pt_print_spc        ; 0x25: undocumented op for SPC(x) function
                    fdb       IL_NO               ; 0x26: reserved
                    fdb       IL_GL               ; 0x27: get input line
                    fdb       IL_NO               ; 0x28: reserved
                    fdb       IL_NO               ; 0x29: reserved
                    fdb       IL_IL               ; 0x2A: Insert BASIC line
                    fdb       IL_MT               ; 0x2B: mark BASIC program space empty
                    fdb       IL_XQ               ; 0x2C: execute
                    fdb       WARM_S              ; 0x2D: warm start
                    fdb       IL_US               ; 0x2E: machine language subroutine call
                    fdb       IL_RT               ; 0x2F: IL subroutine return

;*******************************************************************************
; undocumented IL instruction (unused)
; will take a value from expression stack
; and put onto processor stack

sub_177             proc
                    bsr       IL_SP               ; pop word into A:B
                    std       IL_temp             ; save into IL_temp
                    jmp       push_payload        ; push value on return stack

;*******************************************************************************
; undocumented IL instruction
; will extract stored value on processor stack
; and push back on expr_stack

sub_180             proc
                    jsr       get_payload         ; extract stored value on return stack
                    ldd       IL_temp             ; get this value
                    bra       expr_push_word      ; push on expr_stack

;*******************************************************************************
; IL instruction: duplicate top of expr_stack

IL_DS               proc
                    bsr       IL_SP               ; pop top of expr_stack into A:B
                    bsr       expr_push_word      ; push A:B twice on expr_stack
;                   bra       expr_push_word

;*******************************************************************************
; push A:B on expr_stack

expr_push_word      proc
                    ldx       expr_stack_x        ; get expr_stack top
                    dex                           ; make space for another byte
                    stb       ,x                  ; store byte (low)
                    bra       ?expr_push_byte     ; push A byte

;*******************************************************************************
; push A on expr_stack

expr_push_byte      proc
                    ldx       expr_stack_x        ; get expr_stack top
?expr_push_byte     dex                           ; make space for another byte
                    sta       ,x                  ; save A as new TOS (top of stack value)
                    stx       expr_stack_x        ; set new stack top
                    psha                          ; save A
                    lda       expr_stack_low      ; get stack bottom
                    cmpa      expr_stack_top      ; stack overflow?
                    pula                          ; restore A
                    bcs       IL_NO               ; no, exit

j_error             jmp       CommonError         ; error: stack overflow

;*******************************************************************************
; pop the TOS word off stack, result in A:B

IL_SP               proc
                    bsr       expr_pop_byte       ; pop a byte into B
                    tba                           ; put into A (high byte)
;                   bra       expr_pop_byte

;*******************************************************************************
; pop the TOS byte off stack into B

expr_pop_byte       proc
                    ldb       #1                  ; verify stack is not empty: has 1 byte
;                   bra       pop_byte

;*******************************************************************************

pop_byte            proc
                    addb      expr_stack_top      ; next position on stack
                    cmpb      #$80                ; is it > 0x80?
                    bhi       j_error             ; yes, stack underflow error
                    ldx       expr_stack_x        ; get address of stack top
                    inc       expr_stack_top      ; pop stack
                    ldb       ,x                  ; get TOS byte in B
                    rts

;*******************************************************************************
; IL instruction US: machine language subroutine call

IL_US               proc
                    bsr       us_do               ; call machine language routine
                    bsr       expr_push_byte      ; return here when ML routine does RTS
          ;-------------------------------------- ; push A:B on stack
                    tba
                    bra       expr_push_byte

;*******************************************************************************

us_do               proc
                    lda       #6                  ; verify that stack has at least 6 bytes
                    tab
                    adda      expr_stack_top
                    cmpa      #rnd_seed           ; at end of expr_stack?
                    bhi       j_error             ; yes, error
                    ldx       expr_stack_x        ; load argument list
                    sta       expr_stack_top      ; drop 6 bytes from stack

Loop@@              lda       5,x                 ; push 5 bytes to return stack
                    psha
                    dex
                    decb
                    bne       Loop@@              ; loop until done
                    tpa                           ; push status
                    psha
          ;--------------------------------------
          ; Stack frame is
          ; return address IL_US+2 (caller of bsr us_do)
          ; B
          ; A
          ; X
          ; X
          ; address
          ; address
          ; PSW
          ;--------------------------------------
                    rti                           ; use RTI to branch to routine

;*******************************************************************************
; IL instruction push byte

IL_LB               proc
                    bsr       fetch_il_op         ; get next byte from sequence
                    bra       expr_push_byte      ; push single byte

;*******************************************************************************
; IL instruction push word

IL_LN               proc
                    bsr       fetch_il_op         ; get next two bytes into A:B
                    psha
                    bsr       fetch_il_op
                    tab
                    pula
                    bra       expr_push_word      ; push on stack

;*******************************************************************************
; part of IL linterpreter loop, handle SX instruction

handle_il_SX        proc
                    adda      expr_stack_top      ; opcode is 0..7, add to current stack ptr
                    sta       IL_temp+1           ; make word pointer 0x00SP+opcode
                    clr       IL_temp
                    bsr       expr_pop_byte       ; drop to byte into B
                    ldx       IL_temp             ; get index
                    lda       ,x                  ; get old byte
                    stb       ,x                  ; store byte from TOS there
                    bra       expr_push_byte      ; store old byte on TOS

;*******************************************************************************
; get the next IL opcode and increment the IL PC

fetch_il_op         proc
                    ldx       il_pc               ; get IL PC
                    lda       ,x                  ; read next opcode
                    inx                           ; advance to next byte
                    stx       il_pc               ; save IL PC
IL_NO               tsta                          ; set flags
                    rts

;*******************************************************************************

IL_baseaddr         fdb       start_of_il         ; only used address where IL code starts

;*******************************************************************************
; Cold start entry point
;*******************************************************************************

COLD_S              proc
                    ldx       #RAM_BASIC          ; initialize start of BASIC
                    stx       start_prgm
Loop@@              inx                           ; point to next address
                    com       1,x                 ; complement following byte
                    lda       1,x                 ; load byte
                    com       1,x                 ; complement byte
                    cmpa      1,x                 ; compare with value, should be different, if it is RAM
                    bne       Loop@@              ; if different, advance, until no more RAM cells found
                    stx       end_ram             ; use topmost RAM cell
;                   bra       IL_MT

;*******************************************************************************
; IL instruction MT: clear program

IL_MT               proc
                    ldd       start_prgm          ; load start area
                    addb      SSS                 ; add spare area after end of program
                    adca      #0
                    std       end_prgm            ; save as end of program
                    ldx       start_prgm          ; get addr of start of program
                    clr       ,x                  ; clear line number (means end)
                    clr       1,x
;                   bra       WARM_S

;*******************************************************************************
; Warm start entry point
;*******************************************************************************

WARM_S              proc
                    lds       end_ram             ; set return stack to end of RAM
                                                  ; enters here to start IL loop;
                                                  ; return here after error stop
restart_il          jsr       crlf                ; emit a CRLF

restart_il_nocr     ldx       IL_baseaddr         ; load pointer to IL
                    stx       il_pc               ; store as current IL PC
                    ldx       #rnd_seed           ; set expr_stack top to 0x0080
                    stx       expr_stack_x
                    ldx       #$30                ; set run_mode = 0 (no program)
          ;-------------------------------------- ; set expr_stack_low = 0x30
                    stx       run_mode

il_rs_return        sts       top_of_stack        ; save current stack position

Loop@@              bsr       fetch_il_op         ; fetch next IL opcode
                    bsr       exec_il_opcode      ; execute current IL instruction
                    bra       Loop@@              ; next instruction (il_mainloop)

;*******************************************************************************
; Trick programming here:
; This location is entered in IL_RS by incrementing the return address of
; exec_il_opcode so that it skips over the 'BRA il_mainloop' above

il_rs_target        proc
                    cpx       #$2004              ; this might mask a BRA *+4, which however would
                                                  ; then point into exec_il_opcode+2, which is a TBA
                                                  ; which could then be used for a synthetic
                                                  ; exec_il_opcode...
                                                  ; frankly: this is possibly either a remainder
                                                  ; from old code or a hidden serial number
                                                  ; the 6502 code has a similar anachronism in this
                                                  ; place, so it might be a serial number.
                    bra       il_rs_return        ; enforce storing the stack pointer and do il_mainloop

;*******************************************************************************
; with IL opcode in A, decode opcode and
; branch to appropriate routine

exec_il_opcode      proc
                    ldx       #il_jumptable-4     ; preload address of opcode table - 4
                    stx       IL_temp
                    cmpa      #$30                ; is opcode below 0x30?
                    bcc       handle_30_ff        ; no, skip to handler for higher opcodes
                    cmpa      #8                  ; is it below 8?
                    bcs       handle_il_SX        ; yes, skip to handler for SX instructions
                    asla                          ; make word index
                    sta       IL_temp+1           ; store as offset
                    ldx       IL_temp
                    ldx       $17,x               ; load handler address via offset
                    jmp       ,x                  ; jump to handler

;*******************************************************************************
; Common error routine

CommonError         proc
                    jsr       crlf                ; emit CRLF
                    lda       #'!'
                    sta       expr_stack_low      ; lower stack bottom a bit to avoid another stack fault
          ;-------------------------------------- ; SNAFU already; may overwrite some variables
                    jsr       OUT_V               ; emit exclamation mark
                    lda       #rnd_seed           ; reinitialize stack top
                    sta       expr_stack_top
                    ldd       il_pc               ; load IL PC into A:B
                    subb      IL_baseaddr+1       ; subtract origin of interpreter
                    sbca      IL_baseaddr
                    jsr       emit_number         ; print instruction of IL
                    lda       run_mode            ; is not in program?
                    beq       error_no_lineno     ; no, suppress output of line number
                    ldx       #ErrAt@@            ; load error string
                    stx       il_pc
                    jsr       IL_PC               ; print string at il_prgm_cnt, i.e. "AT "
                    lda       basic_lineno        ; get line number
                    ldb       basic_lineno+1
                    jsr       emit_number         ; print it

error_no_lineno     lda       #7                  ; emit BEL (0x07) character
                    jsr       OUT_V
                    lds       top_of_stack        ; restore return stack
                    bra       restart_il          ; restart interpreter after error

ErrAt@@             fcb       ' ','A','T',' ',$80  ; string " AT " + terminator

;*******************************************************************************
; Long branch instruction

IL_BBR              proc
                    dec       IL_temp             ; adjust position for negative jump (effectively 2's complement)

IL_FBR              tst       IL_temp             ; test new position high byte
                    beq       CommonError         ; was displacement 0?
                                                  ; yes, this is an error condition
il_goto             ldx       IL_temp             ; get new IL target address
                    stx       il_pc               ; do the jump
                    rts

;*******************************************************************************
; part of interpreter loop: handle opcode 0x30-3f

handle_30_ff        proc
                    cmpa      #$40                ; above or equal 0x40?
                    bcc       handle_40_ff        ; yes, handle elsewhere
          ;-------------------------------------- ; handle the J/JS instructions
                    psha                          ; save opcode
                    jsr       fetch_il_op         ; get next byte of instruction (low address)
                    adda      IL_baseaddr+1       ; add to IL interpreter base
                    sta       IL_temp+1
                    pula                          ; restore opcode
                    tab                           ; save into B for later
                    anda      #7                  ; mask out addressbits
                    adca      IL_baseaddr         ; add to base address
                    sta       IL_temp             ; save in temporary
                    andb      #8                  ; mask J/JS bit
                    bne       il_goto             ; if set, is GOTO
                    ldx       il_pc               ; get current IL PC
                    sta       il_pc               ; save new IL PC
                    ldb       IL_temp+1
                    stb       il_pc+1
                    stx       IL_temp             ; save old in temporary
                    jmp       push_payload        ; put on return stack

;*******************************************************************************
; handle the opcodes >=0x40

handle_40_ff        proc
                    tab                           ; save opcode for later
                    lsra:4                        ; get opcode high nibble
                    anda      #$E                 ; make 0x04,0x06,...0x0e
                    sta       IL_temp+1           ; make index into opcode jump table
                    ldx       IL_temp
                    ldx       $17,x               ; X points to handler routine
                    clra                          ; preload A=0 for null displacement (error indicator)
                    cmpb      #$60                ; is it BBR?
                    andb      #$1F                ; mask out displacement bits
                    bcc       _@@                 ; was not backward branch
                    orb       #$E0                ; make displacement negative
_@@                 beq       Cont@@              ; displacement is zero? yes, skip
                    addb      il_pc+1             ; add displayement to current IL_PC
                    stb       IL_temp+1
                    adca      il_pc
Cont@@              sta       IL_temp             ; store high byte of new address
                                                  ; if displayement=0, store high byte=0
                                                  ; -> invalid IL address, will lead to error
                    jmp       ,x                  ; jump to handler routine

;*******************************************************************************
; IL instruction string match branch
; jump forward if string was not matched

IL_BC               proc
                    ldx       basic_ptr           ; save pointer to current BASIC character
                    stx       bp_save

Loop@@              bsr       get_nchar           ; skip spaces
                    bsr       fetch_basicchar     ; consume char
                    tab                           ; save into B
                    jsr       fetch_il_op         ; get next char from IL stream
                    bpl       LastChar@@          ; if positive (not end of string), match further
                    orb       #$80                ; no, make basic char also bit7 set

LastChar@@          cba                           ; compare bytes
                    bne       NoMatch@@           ; do not match, skip
                    tsta                          ; more chars to match?
                    bpl       Loop@@              ; yes, loop
                    rts                           ; that string matched! continue

NoMatch@@           ldx       bp_save             ; restore BASIC pointer
                    stx       basic_ptr
j_FBR               bra       IL_FBR              ; and branch forward

;*******************************************************************************
; IL instruction: jump if not end of line

IL_BE               proc
                    bsr       get_nchar           ; get current BASIC char
                    cmpa      #CR                 ; is it a CR?
                    bne       j_FBR               ; no, jump forward
                    rts                           ; continue

;*******************************************************************************
; IL instruction: branch if not variable
; if variable, push 2*ASCII to expr_stack
; (0x41..0x5A => 0x82...0xB4
; == offset to var table into zero page)

IL_BV               proc
                    bsr       get_nchar           ; get current BASIC char
                    cmpa      #'Z'                ; is it an alphanumeric?
                    bgt       j_FBR               ; no, jump forward
                    cmpa      #'A'
                    blt       j_FBR
                    asla                          ; yes, double the ASCII code
                                                  ; (make it a word index into var table
                    jsr       expr_push_byte      ; push it on the stack
;                   bra       fetch_basicchar     ; ...and consume this character

;*******************************************************************************
; get next BASIC char from program or line
; return in A, Z=1 if CR
;*******************************************************************************

fetch_basicchar     proc
                    ldx       basic_ptr           ; get address of current BASIC byte
                    lda       ,x                  ; get byte
                    inx                           ; advance to next character
                    stx       basic_ptr           ; save it
                    cmpa      #CR                 ; check if 0x0d (end of line)
                    rts

;*******************************************************************************
; get next BASIC char (without advance)
; C=1 if digit

get_nchar           proc
                    bsr       fetch_basicchar     ; get next char
                    cmpa      #' '                ; is it a space?
                    beq       get_nchar           ; yes, skip that
                    dex                           ; is no space, point back to this char
                    stx       basic_ptr
                    cmpa      #'0'                ; is it a digit?
                    clc
                    blt       Done@@              ; no, return C=0
                    cmpa      #':'                ; return C=1 if number
Done@@              rts

;*******************************************************************************
; IL instruction: branch if not number
; if digit, convert this and following digits to number
; and push on expr_stack

IL_BN               proc
                    bsr       get_nchar           ; get BASIC char
                    bcc       j_FBR               ; if not digit, do forward branch
                    clrx                          ; clear temporary for number
                    stx       IL_temp

Loop@@              bsr       fetch_basicchar     ; get and consume this char
                    psha                          ; save it
                    lda       IL_temp             ; multiply TEMP by 10
                    ldb       IL_temp+1
                    aslb                          ; temp*2
                    rola
                    aslb                          ; (temp*2)*2 = temp*4
                    rola
                    addb      IL_temp+1           ; (temp*4)+temp = temp*5
                    adca      IL_temp
                    aslb                          ; (temp*5)*2 = temp*10
                    rola
                    stb       IL_temp+1
                    pulb                          ; restore digit
                    andb      #$F                 ; mask out low nibble (0...9)
                    addb      IL_temp+1           ; add into temp
                    adca      #0
                    std       IL_temp
                    bsr       get_nchar           ; get next char
                    bcs       Loop@@              ; loop as long as digit found
                    lda       IL_temp             ; push A:B on expr_stack (B is still low byte)
                    jmp       expr_push_word

;*******************************************************************************
; IL instruction: divide

IL_DV               proc
                    bsr       expr_check_4bytes   ; validate 2 args on stack; discard 1 byte
                    lda       2,x                 ; high byte dividend
                    asra                          ; put sign into carry
                    rola
                    sbca      2,x                 ; A=0xFF if sign=1, 0x00 if sign=0
                    sta       IL_temp             ; sign extend dividend into 32bit (IL_temp=high word)
                    sta       IL_temp+1
                    tab                           ; if negative, subtract 1 from dividend
                    addb      3,x                 ; 0x0000...0x7fff stays positive
                                                  ; 0x8000 becomes positive
                                                  ; 0x8001...0xffff stays negative
                    stb       3,x
                    tab
                    adcb      2,x
                    stb       2,x
                    eora      ,x                  ; compare with sign of divisor
                    sta       lead_zero           ; store result sign (negative if different, positive if same)
                    bpl       loc_389             ; if different sign, complement divisor
          ;--------------------------------------
          ; i.e. NEG/NEG -> do nothing
          ; NEG/POS -> NEG/NEG + lead_zero <0
          ; POS/NEG -> POS/POS + lead_zero <0
          ; POS/POS -> do nothing
          ;--------------------------------------
                    bsr       negate              ; negate operand

loc_389             ldb       #$11                ; loop counter (16+1 iterations)
                    lda       ,x                  ; is divisor = 0?
                    ora       1,x
                    bne       dv_loop             ; no, do division
                    jmp       CommonError

dv_loop             lda       IL_temp+1           ; subtract divisor from 32bit dividend
                    suba      1,x
                    psha                          ; remember result
                    lda       IL_temp
                    sbca      ,x
                    psha
                    eora      IL_temp
                    bmi       dv_smaller          ; subtract result was <0 ?
                    pula                          ; no, can subtract, remember a 1 bit (sec)
                    sta       IL_temp             ; and store new result
                    pula
                    sta       IL_temp+1
                    sec
                    bra       dv_shift

dv_smaller          pula                          ; discard subtraction
                    pula
                    clc                           ; remember 0 bit

dv_shift            rol       3,x                 ; shift 32bit dividend left
                    rol       2,x                 ; shift in result bit into lowest bit of dividend
                    rol       IL_temp+1
                    rol       IL_temp
                    decb                          ; decrement loop
                    bne       dv_loop             ; subtract divisor from 32bit dividend
                    bsr       j_expr_pop_byte     ; drop a byte (other one was already removed above)
          ;-------------------------------------- ; X points to result in (former) dividend at 2,X
                    tst       lead_zero           ; operand signs were different?
                    bpl       locret_3CC          ; no, we are done
;                   bra       IL_NE

;*******************************************************************************
; IL instruction: negate top of stack

IL_NE               proc
                    ldx       expr_stack_x        ; point to TOS
negate              neg       1,x                 ; negate low byte
                    bne       ne_nocarry          ; not zero: no carry
                    dec       ,x                  ; propagate carry into high byte
ne_nocarry          com       ,x                  ; complement high byte
locret_3CC          rts

;*******************************************************************************
; IL instruction: subtract TOS from NOS -> NOS

IL_SU               proc
                    bsr       IL_NE               ; negate TOS. A-B is A+(-B)

;*******************************************************************************
; IL instruction: add TOS and NOS -> NOS

IL_AD               proc
                    bsr       expr_check_4bytes   ; verify 4 bytes on stack
                    ldb       3,x                 ; add TOS and NOS into AB
                    addb      1,x
                    lda       2,x
                    adca      ,x

expr_save_pop       std       2,x                 ; store A:B in NOS and pop further byte
j_expr_pop_byte     jmp       expr_pop_byte

;*******************************************************************************
; validate stack contains at least 4 bytes, pop 1 byte

expr_check_4bytes   proc
                    ldb       #4
expr_check_nbytes   jmp       pop_byte            ; pop a byte

;*******************************************************************************
; multiply TOS with NOS -> NOS
; I think this this routine is broken for negative numbers

IL_MP               proc
                    bsr       expr_check_4bytes   ; validate 2 args
                    lda       #$10                ; bit count (16 bits)
                    sta       IL_temp
                    clrd                          ; clear bottom 16 bits of result

Loop@@              aslb                          ; shift 1 bit left
                    rola
                    asl       1,x                 ; shift 1st operand
                    rol       ,x
                    bcc       Cont@@              ; is top bit = 1?
                    addb      3,x                 ; yes, add 2nd operand into A:B
                    adca      2,x

Cont@@              dec       IL_temp             ; decrement counter
                    bne       Loop@@              ; loop 16 times
                    bra       expr_save_pop       ; save result

;*******************************************************************************
; IL instruction: fetch variable

IL_FV               proc
                    bsr       j_expr_pop_byte     ; get byte (variable address into zero page)
                    clra
                    std       IL_temp             ; make pointer into var table
                    ldx       IL_temp
                    ldd       ,x                  ; get word indexed by X into A:B
                    jmp       expr_push_word      ; push it onto expr_stack

;*******************************************************************************
; IL instruction: save variable

IL_SV               proc
                    ldb       #3
                    bsr       expr_check_nbytes   ; validate stack contains var index byte
                                                  ; and data word. drop single byte
                    ldd       ,x                  ; data in D
                    clr       1,x                 ; clear this to build word index to var
                    ldx       1,x                 ; load index into variable table
                    std       ,x                  ; save A:B into variable

j_IL_SP             jmp       IL_SP               ; pop word off stack

;*******************************************************************************
; IL instruction compare
; stack: TOS, MASK, NOS
; compare TOS with NOS
; MASK is bit0 = less
; bit1 = equal
; bit2 = greater
; if compare reslut AND    mask return <>0, next IL op is skipped

IL_CP               proc
                    bsr       j_IL_SP             ; pop TOS into A:B
                    pshb                          ; save low byte
                    ldb       #3
                    bsr       expr_check_nbytes   ; verify still 3 bytes on stack,
                                                  ; drop one byte
                    inc       expr_stack_top      ; drop more bytes
                    inc       expr_stack_top
                    pulb                          ; restore low byte of TOS
                    subb      2,x                 ; compare with 1st arg
          ;--------------------------------------
          ; note this subtraction is inverted
          ; thus BGT means BLT, and vice versa
          ;--------------------------------------
                    sbca      1,x
                    bgt       cp_is_lt            ; if less, skip
                    blt       cp_is_gt            ; if greater, skip
                    tstb                          ; is result 0?
                    beq       cp_is_eq
                    bra       cp_is_lt

cp_is_gt            asr       ,x                  ; shift bit 2 into carry
cp_is_eq            asr       ,x                  ; shift bit 1 into carry
cp_is_lt            asr       ,x                  ; shift bit 0 into carray
                    bcc       locret_461          ; not matched: exit, continue new IL op
                    jmp       fetch_il_op         ; skip one IL op before continuing

;*******************************************************************************
; IL instruction: advance to next BASIC line

IL_NX               proc
                    lda       run_mode            ; run mode = 0?
                    beq       loc_46A             ; yes, continue program
Loop@@              jsr       fetch_basicchar     ; get char from program
                    bne       Loop@@              ; is not CR, loop
                    bsr       save_lineno         ; store line number
                    beq       j1_error            ; is 0000 (end of program) -> error
;                   bra       go_found_line

;*******************************************************************************
; enters here from a GOTO,
; basic pointer points to new line

go_found_line       proc
                    bsr       do_runmode          ; set run mode = running
                    jsr       BV                  ; test for BREAK
                    bcs       do_break            ; if C=1, do break
                    ldx       il_pc_save          ; restore IL_PC which was saved in XQ or GO
                    stx       il_pc

locret_461          rts

do_break            ldx       IL_baseaddr         ; restart interpreter
                    stx       il_pc

j1_error            jmp       CommonError         ; and emit break error

;*******************************************************************************
; fragment of code for IL_NX

loc_46A             proc
                    lds       top_of_stack        ; reload stack
                    sta       column_cnt          ; clear column count (A was 0)
                    jmp       restart_il_nocr     ; restart interpreter

;*******************************************************************************
; save current linenumber

save_lineno         proc
                    jsr       fetch_basicchar     ; get char from program code
                    sta       basic_lineno        ; save as high lineno
                    jsr       fetch_basicchar     ; get char from program code
                    sta       basic_lineno+1      ; save as low lineno
                    ldx       basic_lineno        ; load line number for later
                    rts

;*******************************************************************************
; IL instruction: execute program

IL_XQ               proc
                    ldx       start_prgm          ; set current start of program
                    stx       basic_ptr
                    bsr       save_lineno         ; save current line number
                    beq       j1_error            ; if zero, error
                    ldx       il_pc               ; save current IL_PC
                    stx       il_pc_save

do_runmode          tpa                           ; will load non zero value (0xc0) into A - tricky!
                    sta       run_mode            ; set run_mode = "running"
                    rts

;*******************************************************************************
; IL instruction GO

IL_GO               proc
                    jsr       find_line           ; find line which lineno is on stack
                    beq       go_found_line       ; found? yes, skip

go_error            ldx       IL_temp             ; set requested lineno as current
                    stx       basic_lineno
                    bra       j1_error            ; error - line not found

;*******************************************************************************
; IL instruction: restore saved line

IL_RS               proc
                    bsr       get_payload         ; get saved line 2 levels off stack
                    tsx                           ; point to caller of exec_il_opcode
                    inc:2     1,x                 ; hack: adjust return from exec_il_mainloop
                                                  ; that it points to il_rs_target just below
                                                  ; il_mainloop
                    jsr       find_line1          ; find the basic line
                    bne       go_error            ; line not found? -> error
                    rts

;*******************************************************************************
; IL instruction return from IL call

IL_RT               proc
                    bsr       get_payload         ; get saved IL PC address
                    stx       il_pc               ; restore it to IL_PC
                    rts

;*******************************************************************************
; IL instruction save BASIC pointer

IL_SB               proc
                    ldx       #basic_ptr          ; get address of basic pointer
                    bra       loc_4B3             ; continue in IL_RB common code

;*******************************************************************************
; IL instruction: restore BASIC pointer

IL_RB               proc
                    ldx       #basicptr_save
loc_4B3             lda       1,x                 ; is it into the input line area?
                    cmpa      #$80
                    bcc       swap_bp
                    lda       ,x
                    bne       swap_bp             ; no, do swap with save location
                    ldx       basic_ptr
                    stx       basicptr_save       ; store old basic pointer into save
                    rts

;*******************************************************************************

swap_bp             proc
                    ldx       basic_ptr           ; get basic pointer
                    lda       basicptr_save       ; move saved pointer to basic ptr
                    sta       basic_ptr
                    lda       basicptr_save+1
                    sta       basic_ptr+1
                    stx       basicptr_save       ; store old basic pointer into save
                    rts

;*******************************************************************************
; IL instruction gosub

IL_GS               proc
                    tsx
                    inc:2     1,x                 ; adjust return address to il_rs_target
                    ldx       basic_lineno        ; get line number of GOSUB
                    stx       IL_temp             ; store it in temp
;                   bra       push_payload        ; and  inject temp into return stack

;*******************************************************************************
; insert IL_temp into return stack

; stack holds (low to high addresses)
; SP->
; return address of exec_il_opcode
; other data

; afterwards
; SP ->
; return address of exec_il_opcode
; payload
; other data

push_payload        proc
                    des:2                         ; reserve 2 bytes on processor stack
                    tsx                           ; get address in X
                    lda       2,x                 ; duplicate return address
                    sta       ,x
                    lda       3,x
                    sta       1,x
                    lda       IL_temp             ; insert return address for JS instruction in stack
                    sta       2,x
                    lda       IL_temp+1
                    sta       3,x
                    ldx       #end_prgm           ; address of end of program
                    sts       IL_temp             ; save current stack in temporary
                    lda       1,x                 ; check that stack does not run into program code
                    suba      IL_temp+1
                    lda       ,x
                    sbca      IL_temp
                    bcs       Done@@              ; is still space available?
                                                  ; yes, exit
?CommonError        jmp       CommonError         ; no error
Done@@              equ       :AnRTS

;*******************************************************************************
; return payload in X

; stack:
; X
; 0  returnaddr    caller of get_payload
; 1  returnaddr    caller of get_payload
; 2  returnaddr    caller of exec_il_opcode
; 3  returnaddr    caller of exec_il_opcode
; 4  payload
; 5  payload

get_payload         proc
                    tsx                           ; copy return stack addr to X
                    inx:3                         ; pointing to return address
                                                  ; skip over return address and 2 more bytes
                                                  ; point to index 3
                    cpx       end_ram             ; stack underflow?
                    beq       ?CommonError        ; yes, error
                    ldx       1,x                 ; get payload into X
                    stx       IL_temp             ; save it
                    tsx                           ; point to return address
                    pshb                          ; save B
                    ldb       #4                  ; move 4 bytes above

Loop@@              lda       3,x
                    sta       5,x
                    dex
                    decb
                    bne       Loop@@              ; loop until done
                    pulb                          ; restore B
                    ins:2                         ; drop 1 word (duplicate return address)
                    ldx       IL_temp             ; get payload
                    rts                           ; done

;*******************************************************************************
; find BASIC line whose lineno is on stack
; discard from stack
; return found line in basic_ptr
; Z=1 if line is matched exactly

find_line           proc
                    jsr       IL_SP               ; pop word into A:B
                    std       IL_temp             ; save in temporary
                    ora       IL_temp+1           ; check if zero (invalid)
                    beq       ?CommonError        ; if so, error
;                   bra       find_line1

;*******************************************************************************
; find BASIC line whose lineno is in IL_temp

find_line1          proc
                    ldx       start_prgm          ; set BASIC pointer to start
                    stx       basic_ptr

Loop@@              jsr       save_lineno         ; save current lineno
                                                  ; note: X = lineno
                    beq       Done@@              ; if zero, skip to end
                    ldb       basic_lineno+1      ; compare line number with current line
                    lda       basic_lineno
                    subb      IL_temp+1
                    sbca      IL_temp
                    bcc       Done@@              ; if above, exit

ToEoL@@             jsr       fetch_basicchar     ; get next char
                    bne       ToEoL@@             ; not CR? loop
                    bra       Loop@@              ; check next line

Done@@              cpx       IL_temp             ; compare current linenumber with searched one
                    rts

;*******************************************************************************
; emit number in A:B

emit_number         proc
                    jsr       expr_push_word      ; push number on stack
;                   bra       IL_PN

;*******************************************************************************

IL_PN               proc
                    ldx       expr_stack_x        ; get address of stack top
                    tst       ,x                  ; is number negative?
                    bpl       Go@@                ; no, skip
                    jsr       IL_NE               ; negate top of stack
                    lda       #'-'                ; emit negative sign
                    bsr       emit_char

Go@@                clra                          ; push 0 (end of digits)
                    psha
                    ldd       #$1A0F
                    psha                          ; counter for 10's (0x1A)
                    pshb                          ; counter for 100's (0x0F)
                    psha                          ; counter for 1000's, (0x1A)
                    pshb                          ; counter for 10000's (0x0f)
                    jsr       IL_SP               ; pop TOS into A:B
                    tsx                           ; point to the constants 0xF, 0x1A....

_10000@@            inc       ,x                  ; increment counter for 10000's
                    subb      #[10000             ; subtract 10000 until negative
                    sbca      #]10000
                    bcc       _10000@@            ; counter for 10000's will become 0x10...0x14

_1000@@             dec       1,x                 ; is now negative value, subtract until positive again
                    addb      #[1000              ; add 1000 until positive again
                    adca      #]1000              ; decrement counter for 1000's
                    bcc       _1000@@             ; counter for 1000's will become 0x19...0x10

_100@@              inc       2,x                 ; is positive value now
                    subb      #100                ; subtract 100 until negative
                    sbca      #0
                    bcc       _100@@              ; counter for 100's becomes 0x10...0x19

_10@@               dec       3,x                 ; is now negative
                    addb      #10                 ; add 10 until positive again
                    bcc       _10@@               ; counter for 10's become 0x10..0x19
          ;-------------------------------------- ; B contains remaining 1's digits
                    clr       lead_zero           ; clear flag to suppress leading zeroes

EmitDigits@@        pula                          ; restore counter 10000
                    tsta                          ; was zero?
                    beq       Done@@              ; yes, last digit to emit, this one is in B
                    bsr       emit_digit          ; emit digit in A, suppress leading zeroes
                    bra       EmitDigits@@        ; guarantee last digit is printed.

Done@@              tba                           ; last digit is in B
;                   bra       emit_digit

;*******************************************************************************

emit_digit          proc
                    cmpa      #$10                ; check if '0' (note range is 0x10..19 if not last digit)
                    bne       Go@@                ; no, must emit
          ;--------------------------------------
          ; note for last digit, any value will be emitted,
          ; because it can't be 0x10 (is 0...9)
          ;--------------------------------------
                    tst       lead_zero           ; already emitted a digit?
                    beq       Done@@              ; no, exit (leading zero)

Go@@                inc       lead_zero           ; notify digit print
                    ora       #'0'                ; make it a real ASCII '0'...'9'
;                   bra       emit_char           ; and print it
Done@@              equ       :AnRTS

;*******************************************************************************
; emit a character in A

emit_char           proc
                    inc       column_cnt          ; advance to column 1
                    bmi       StopEmit@@          ; if at column 128, stop emit
                    stx       x_save              ; save X
                    pshb                          ; save B
                    jsr       OUT_V               ; emit character
                    pulb                          ; restore B
                    ldx       x_save              ; restore X
                    rts                           ; done

StopEmit@@          dec       column_cnt          ; if column = 0x80, don't advance further
                    rts

;*******************************************************************************
; IL instruction print string

pc_loop             proc
                    bsr       emit_char           ; emit a character and continue
                                                  ; with PC instruction
IL_PC               jsr       fetch_il_op         ; get next byte of instruction
                    bpl       pc_loop             ; if positive, skip
                    bra       emit_char           ; was last char, emit it and terminate

;*******************************************************************************
; IL instruction PQ

loop_pq             proc
                    cmpa      #'"'                ; is character string terminator?
                    beq       Done@@              ; yes, exit
                    bsr       emit_char           ; otherwise emit char
          ;-------------------------------------- ; and redo PQ instruction
IL_PQ               jsr       fetch_basicchar     ; get next char from BASIC text
                    bne       loop_pq             ; if not CR, loop
                    jmp       CommonError         ; error - unterminated string
Done@@              equ       :AnRTS

;*******************************************************************************
; IL instruction print tab

IL_PT               proc
                    ldb       column_cnt          ; column counter
                    bmi       Done@@              ; if overflow, exit
                    orb       #$F8                ; make 7...0
                    negb
                    bra       ?print_spc          ; jump to space printer
Done@@              equ       :AnRTS

;*******************************************************************************

pt_print_spc        proc
                    jsr       IL_SP               ; drop A:B off stack
?print_spc
Loop@@              decb                          ; decrement low byte
                    blt       Done@@              ; < 0, exit
                    lda       #' '                ; emit a space
                    bsr       emit_char
                    bra       Loop@@              ; loop
Done@@              equ       :AnRTS

;*******************************************************************************
; IL Instruction List BASIC source

IL_LS               proc
                    ldx       basic_ptr           ; save current BASIC pointer
                    stx       bp_save
                    ldx       start_prgm          ; default start: begin of program
                    stx       basic_ptr
                    ldx       end_prgm            ; default end: load X with end of program
                    bsr       ls_getlineno        ; if argument to list given, make this new end
          ;--------------------------------------
          ; note "LIST start,end", so the first result
          ; popped off stack is the end
          ;--------------------------------------
                    beq       Loop@@              ; no more argument on stack
                    bsr       ls_getlineno        ; save first position in LS_begin
          ;-------------------------------------- ; get another argument into basic_ptr, if any
Loop@@              lda       basic_ptr           ; compare start and end of listing
                    ldb       basic_ptr+1
                    subb      LS_end+1
                    sbca      LS_end
                    bcc       ls_exit             ; start > end? yes, exit: nothing (more) to list
                    jsr       save_lineno         ; save lineno of current line
                    beq       ls_exit             ; is end of program (line 0)? yes, exit
                    lda       basic_lineno        ; get current line number
                    ldb       basic_lineno+1
                    jsr       emit_number         ; print line number
                    lda       #' '                ; print a space

_@@                 bsr       j_emitchar
                    jsr       BV                  ; check for break
                    bcs       ls_exit             ; if break, exit
                    jsr       fetch_basicchar     ; get next char from line
                    bne       _@@                 ; if not CR, loop output
                    bsr       IL_NL               ; emit a CRLF
                    bra       Loop@@              ; loop with next line

;*******************************************************************************
; called with an address into BASIC code
; return Z=1 if no argument

ls_getlineno        proc
                    inx                           ; increment X
                    stx       LS_end              ; store as default end of listing
                    ldx       expr_stack_x        ; get expr_stack ptr
                    cpx       #$80                ; is stack empty?
                    beq       Done@@              ; yes, no arg given...done
                    jsr       find_line           ; find the line (after the lineno) that was given on
                                                  ; stack (start line number)
                                                  ; result in X=basic_ptr
;                   bra       ls_to_linestart
Done@@              equ       :AnRTS

;*******************************************************************************

ls_to_linestart     proc
                    ldx       basic_ptr           ; point back to lineno that was found
                    dex:2
                    stx       basic_ptr
                    rts

;*******************************************************************************

ls_exit             proc
                    ldx       bp_save             ; restore old BASIC pointer
                    stx       basic_ptr
                    rts

;*******************************************************************************
; IL instruction: emit new line

IL_NL               proc
                    lda       column_cnt          ; if column > 127, suppress output
                    bmi       Done@@
;                   bra       crlf
Done@@              equ       :AnRTS

;*******************************************************************************
; do a CRLF

crlf                proc
                    lda       #CR                 ; emit carriage return character
                    bsr       emit_char_at_0
                    ldb       PCC                 ; get padding mode
                    aslb                          ; shift out bit 7
                    beq       Cont@@              ; if no padding bytes, skip

Loop@@              pshb                          ; save padding count
                    bsr       emit_nul_padding    ; emit padding
                    pulb                          ; restore count
                    decb                          ; decrement twice (because above
                    aslb                          ; multiplied *2)
                    decb
                    bne       Loop@@              ; loop until done

Cont@@              lda       #LF                 ; emit line feed character
                    bsr       j_emitchar          ; emit character (with increment column count)
;                   bra       emit_nul_padding

;*******************************************************************************
; depending on PCC bit 7 emit either NUL or DEL (0xff) byte

emit_nul_padding    proc
                    clra                          ; padding byte
                    tst       PCC                 ; check if bit 7 of PCC:
                                                  ; =0, emit NUL bytes
                                                  ; =1, emit 0xFF bytes
                    bpl       emit_char_at_0      ; emit a NUL byte
                    coma
;                   bra       emit_char_at_0

;*******************************************************************************
; emit a char in A and clear column count/XOFF mode

emit_char_at_0      proc
                    clr       column_cnt          ; reset column to 0
j_emitchar          jmp       emit_char

;*******************************************************************************

do_xon              proc
                    lda       TMC                 ; get XOFF flag
                    bra       ?SaveXOnOff

;*******************************************************************************

do_xoff             proc
                    clra
?SaveXOnOff         sta       column_cnt          ; save column count
                    bra       gl_loop

;*******************************************************************************
; IL instruction: get input line
; uses expr_stack as buffer

IL_GL               proc
                    ldx       #expr_stack         ; store floor of expr_stack as BASIC pointer
                    stx       basic_ptr
                    stx       IL_temp             ; save pointer to char input into buffer
                    jsr       expr_push_word      ; save A:B for later (may be variable address, or alike)

Loop@@              eora      rnd_seed            ; use random A to create some entropy
                    sta       rnd_seed
                    jsr       IN_V                ; get a char from input device
                    anda      #$7F                ; make 7bit ASCII
                    beq       Loop@@              ; if NUL, ignore
                    cmpa      #$7F                ; if 0xFF/0x7F, ignore
                    beq       Loop@@
                    cmpa      #LF                 ; if LF, done
                    beq       do_xon
                    cmpa      #$13                ; if DC3 (XOFF) handle XOFF
                    beq       do_xoff
                    ldx       IL_temp             ; get buffer pointer
                    cmpa      LSC                 ; line cancel?
                    beq       CtrlX@@
                    cmpa      BSC                 ; is it "_" ? (back character)
                    bne       ChkEnd@@            ; no, skip
                    cpx       #expr_stack         ; at start of buffer?
                    bne       Backspace@@         ; no, do a backspace

CtrlX@@             ldx       basic_ptr           ; reset pointer to input char
                    lda       #CR
                    clr       column_cnt          ; do XON

ChkEnd@@            cpx       expr_stack_x        ; is end of buffer reached?
                    bne       SaveChar@@          ; no, skip
                    lda       #7                  ; emit BEL character (line overflow)
                    bsr       j_emitchar
                    bra       Loop@@

SaveChar@@          sta       ,x                  ; save char in buffer
                    inx:2                         ; advance

Backspace@@         dex
                    stx       IL_temp             ; !!! error in dump, was 0F
          ;-------------------------------------- ; save new ptr to input
                    cmpa      #CR                 ; was char CR?
                    bne       Loop@@              ; no, get another char
                    jsr       IL_NL               ; end of input reached
                    lda       IL_temp+1           ; get buffer line
                    sta       expr_stack_low      ; save as new expr_stack bottom
          ;-------------------------------------- ; (should not overwrite buffer)
                    jmp       IL_SP               ; pop old value off stack

gl_loop             equ       Loop@@

;*******************************************************************************
; IL instruction: insert BASIC line

IL_IL               proc
                    jsr       swap_bp             ; basicptr_save = 0x80 (input buffer)
                                                  ; basic_ptr = invalid
                    jsr       find_line           ; search for line with lineno from stack
                                                  ; if found: address of BASIC text in basic_ptr
                                                  ; if not: address of next line or end of program
                    tpa                           ; save status, whether line was found
                    jsr       ls_to_linestart     ; adjust line back to lineno
                    stx       bp_save             ; save this in bp_save as well.
                                                  ; basic_ptr==bp_save is position where to enter
                                                  ; new line (if same lineno, overwrite)
                    ldx       IL_temp             ; save lineno to be handled in LS_end
                    stx       LS_end
                    clrb                          ; preload length of stored line with 0
                                                  ; for line not found (must grow)
                    tap                           ; restore status of find_line
                    bne       LineNotFound@@      ; skip if lineno not matched
                                                  ; hey, this line already exists!
                    jsr       save_lineno         ; save lineno where we are currently in basic_lineno
                    ldb       #$FE                ; advance to end of line,
          ;-------------------------------------- ; B is negative length of line
FindEol@@           decb
                    jsr       fetch_basicchar
                    bne       FindEol@@           ; loop until end of line
          ;-------------------------------------- ; B now contains negative sizeof(stored line)
LineNotFound@@      clrx                          ; B is 0, if line does not yet exist
                    stx       basic_lineno        ; clear lineno
                    jsr       swap_bp             ; basic_ptr = 0x80 (input buffer)
          ;--------------------------------------
          ; basicptr_save = at end of position to insert
          ; (i.e. either before following line, or at end of
          ; line to be grown/shrunk)
          ;-------------------------------------- ; calculate sizeof(input buffer)
                    lda       #CR                 ; load EOLN char
                    ldx       basic_ptr           ; start at input buffer (after line number)
                    cmpa      ,x                  ; is it eoln?
                    beq       Delete@@            ; yes, skip - this is an empty line: must delete
                    addb      #3                  ; no, reserve 3 bytes for lineno and CR

ToEoL@@             incb                          ; increment B for every byte in current line <> eoln
                    inx
                    cmpa      ,x                  ; advance and check for EOLN
                    bne       ToEoL@@             ; loop until eoln found
          ;--------------------------------------
          ; all in all, B contains the difference of line lengths:
          ; -sizeof(stored line)+sizeof(input buffer)
          ; if negative: stored is longer  than new -> shrink program
          ; if zero: stored is same size
          ; if positive: stored is shorter than new -> grow program
          ;--------------------------------------
                    ldx       LS_end              ; restore current lineno
          ;-------------------------------------- ; is non-null: there is a line to add
                    stx       basic_lineno

Delete@@            ldx       bp_save             ; IL_temp = start of area to insert line
                    stx       IL_temp
                    tstb                          ; check number of bytes
          ;--------------------------------------
          ; negative: shrink program
          ; zero: nothing to move
          ; positive: grow program
          ;--------------------------------------
                    beq       SameSize@@          ; same size, just copy
                    bpl       GrowLine@@          ; stored line is longer -> shrink
                    lda       basicptr_save+1     ; bp_save = end_of_insert - bytes to shrink
                    aba
                    sta       bp_save+1
                    lda       basicptr_save
                    adca      #$FF
                    sta       bp_save             ; bp_save < basicptr_save < end_pgrm < top_of_stack (hopefully)

Shrink@@            ldx       basicptr_save       ; copy from end of insert addr to bp_save addr
                    ldb       ,x
                    cpx       end_prgm            ; until end of program
                    beq       EndOfProg@@
                    cpx       top_of_stack        ; or until top_of_stack
                    beq       EndOfProg@@         ; leave, when done
                    inx                           ; advance
                    stx       basicptr_save
                    ldx       bp_save
                    stb       ,x                  ; save the byte
                    inx
                    stx       bp_save
                    bra       Shrink@@            ; loop until done

GrowLine@@          addb      end_prgm+1          ; make space after end of program for B bytes
                    stb       basicptr_save+1
                    clra
                    adca      end_prgm
                    sta       basicptr_save       ; basicptr_save = new end of program
                    subb      top_of_stack+1
                    sbca      top_of_stack        ; verify it's below top_of_RAM
                    bcs       DoGrow@@            ; ok, continue
                    dec       il_pc+1             ; point back to IL instruction
                    jmp       CommonError         ; overflow error

DoGrow@@            ldx       basicptr_save       ; bp_save is new end of program
                    stx       bp_save

Grow@@              ldx       end_prgm            ; get byte from old end of program
                    lda       ,x
                    dex                           ; advance back
                    stx       end_prgm
                    ldx       basicptr_save       ; store byte at new end of program
                    sta       ,x
                    dex
                    stx       basicptr_save
                    cpx       IL_temp
                    bne       Grow@@              ; loop until done

EndOfProg@@         ldx       bp_save             ; adjust new end of program
                    stx       end_prgm

SameSize@@          ldx       basic_lineno        ; now there is space at position for the new line
          ;-------------------------------------- ; check lineno:  is 0 if delete
                    beq       Done@@              ; nothing to copy (gap is already closed)
                    ldx       IL_temp             ; start of area to insert into (the gap)
                    ldd       basic_lineno        ; store the line number into this area
                    std       ,x
                    inx
          ;--------------------------------------
MoveLine@@          inx
                    stx       IL_temp             ; position of gap
                    jsr       fetch_basicchar     ; get char from input buffer
                    ldx       IL_temp             ; put it into gap
                    sta       ,x
                    cmpa      #CR                 ; until EOLN
                    bne       MoveLine@@

Done@@              lds       top_of_stack        ; finished with IL
          ;-------------------------------------- ; reload stack pointer
                    jmp       restart_il_nocr     ; and re-enter BASIC loop

;*******************************************************************************
; Break routine for Motorola MINIBUG

minibug_break       proc
                    lda       ACIA_STATUS         ; ACIA control status
                    asra                          ; check bit0: receive buffer full
                    bcc       Done@@              ; no, exit, carry clear
                    lda       ACIA_DATA           ; load ACIA data
                    bne       Done@@              ; if not NUL, return carry set
                    clc                           ; was NUL, ignore, retun carry clear
Done@@              rts

;*******************************************************************************
; Input/Echo routine for Motorola MINIBUG

minibug_inoutput    proc
                    lda       ACIA_STATUS         ; get ACIA status
                    asra                          ; check bit: receiver buffer empty?
                    bcc       minibug_inoutput    ; yes, wait for char
                    lda       ACIA_DATA           ; get ACIA data
                    psha                          ; save it for later
Loop@@              lda       ACIA_STATUS         ; get ACIA status
                    anda      #2                  ; check bit1: transmit buf empty?
                    beq       Loop@@              ; no, wait until transmitted
                    pula                          ; restore char
                    sta       ACIA_DATA           ; echo data just entered
                    rts

;*******************************************************************************
; test break routine for MIKBUG

mikbug_chkbreak     proc
                    lda       PIA_REG             ; check bitbang input of PIA
                    clc
                    bmi       Done@@              ; if 1, exit: no input
Loop@@              lda       PIA_REG             ; is zero, wait until 1
                    bpl       Loop@@
                    bsr       Emit@@              ; emit byte 0xFF twice
Emit@@              lda       #$FF                ; emit 0xFF
                    jsr       OUT_V
                    sec
Done@@              rts

;*******************************************************************************
; The IL interpreter commented
;*******************************************************************************

start_of_il         fcb       $24,':',$11+$80     ; PL : print literal ":",XON
                    fcb       $27                 ; GL : get input line
                    fcb       $10                 ; SB : save BASIC pointer
                    fcb       $E1                 ; BE 01: if not eoln, branch to il_test_insert
                    fcb       $59                 ; BR 19: branch to start_of_il
il_test_insert      fcb       $C5                 ; BN 05: if not number, branch to il_test_let
                    fcb       $2A                 ; IL : insert BASIC line
                    fcb       $56                 ; BR 16: branch to start_of_il
il_run              fcb       $10                 ; SB : save BASIC pointer
                    fcb       $11                 ; RB : restore BASIC pointer
                    fcb       $2C                 ; XC : execute
il_test_let         fcb       $8B,'LE','T'+$80    ; BC 0B: if not "LET", branch to il_test_go
                    fcb       $A0                 ; BV 00: if not variable, error
                    fcb       $80,'='+$80         ; BC 00: if not "=", error
il_let              fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $13                 ; SV : store variable
                    fcb       $1D                 ; NX : next BASIC statement
il_test_go          fcb       $94,'G','O'+$80     ; BC 14: if not "GO", branch to il_test_10
                    fcb       $88,'T','O'+$80     ; BC 08: if not "TO", branch to il_test_sub
                    fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $10                 ; SB : save BASIC pointer
                    fcb       $11                 ; RB : restore BASIC pointer
                    fcb       $16                 ; GO : GOTO
il_test_sub         fcb       $80,'SU','B'+$80    ; BC 00: if not "SUB", error
                    fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $14                 ; GS : GOSUB save
                    fcb       $16                 ; GO : GOTO
il_test_pr          fcb       $90,'P','R'+$80     ; BC 10: if not "PR", branch to il_jump1
                    fcb       $83,'IN','T'+$80    ; BC 03: if not "INT", branch to il_print
il_print            fcb       $E5                 ; BE 05: if not eoln, branch to il_pr_test_dq
                    fcb       $71                 ; BR 31: branch to il_pr_must_eoln
il_pr_test_semi     fcb       $88,';'+$80         ; BC 08: if not ";", branch to il_pr_test_com
il_pr_eoln          fcb       $E1                 ; BE 01: if not eoln, branch to il_pr_test_dq
                    fcb       $1D                 ; NX : next BASIC statement
il_pr_test_dq       fcb       $8F,'"'+$80         ; BC 0F: if not dblquote, branch to il_pr_expr
                    fcb       $21                 ; PQ : print BASIC string
                    fcb       $58                 ; BR 18: branch to il_test_semi
il_jump1            fcb       $6F                 ; BR 2F: branch to il_test_if
il_pr_test_com      fcb       $83,','+$80         ; BC 03: if not ",", branch to il_test_colon
                    fcb       $22                 ; PT : print TAB
                    fcb       $55                 ; BR 15: branch to il_pr_eoln
il_test_colon       fcb       $83,':'+$80         ; BC 03: if not ":", branch to il_pr_must_eoln
                    fcb       $24,$13+$80         ; PR : print literal XOFF
il_pr_must_eoln     fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $23                 ; NL : new line
                    fcb       $1D                 ; NX : next statement
il_pr_expr          fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $20                 ; PN : print number
                    fcb       $48                 ; BR 08: branch to il_pr_test_semi
                    fcb       $91,'I','F'+$80     ; BC 11: if not "IF", branch to il_test_input
il_test_if          fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $31,$34             ; JS 134: call il_cmpop
                    fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $84,'THE','N'+$80   ; BC 04: if not "THEN", branch to il_test_input
                    fcb       $1C                 ; CP : compare
                    fcb       $1D                 ; NX : next BASIC statement
                    fcb       $38,$D              ; J 00D: jump il_test_let
il_test_input       fcb       $9A,'INPU','T'+$80  ; BC 1A: if not "INPUT", branch to il_test_return
il_in_more          fcb       $A0                 ; BV 00: if not variable, error
                    fcb       $10                 ; SB : save BASIC pointer
                    fcb       $E7                 ; BE 07: if not eoln, branch to il_in_test_com
il_in_query         fcb       $24,'?',' ',$11+$80 ; PR : print literal "? ",XON
                    fcb       $27                 ; GL : get input line
                    fcb       $E1                 ; BE 01: if not eoln, branch to il_in_test_com
                    fcb       $59                 ; BR 19: branch to il_in_query
il_in_test_com      fcb       $81,','+$80         ; BC 01: if not ",", branch to il_in_get
                    fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $13                 ; SV : store variable
                    fcb       $11                 ; RB : restore BASIC pointer
                    fcb       $82,','+$80         ; BC 02: if not ",", branch il_in_done
                    fcb       $4D                 ; BR 0D: branch to il_in_more
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $1D                 ; NX : next BASIC statement
il_test_return      fcb       $89,'RETUR','N'+$80 ; BC 09: if not "RETURN", branch to il_test_end
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $15                 ; RS : restore saved line
                    fcb       $1D                 ; NX : next BASIC statement
il_test_end         fcb       $85,'EN','D'+$80    ; BC 05: if not "END", branch to il_test_list
                    fcb       $E0                 ; BE 00: if not eoln, error
                    fcb       $2D                 ; WS : stop
il_test_list        fcb       $98,'LIS','T'+$80   ; BC 18: if not "LIST", branch to il_test_run
                    fcb       $EC                 ; BE 0C: if not eoln, branch to il_li_line
il_li_newline       fcb       $24,0,0,0,0,LF,0+$80 ; PR : print literal NUL,NUL,NUL,NUL,LF,NUL
                    fcb       $1F                 ; LS : list the program
                    fcb       $24,$13+$80         ; PR : print literal XOFF
                    fcb       $23                 ; NL : newline
                    fcb       $1D                 ; NX : next BASIC statement
il_li_line          fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $E1                 ; if not eoln, branch to il_li2
                    fcb       $50                 ; BR 10: branch to il_li_newline
                    fcb       $80,','+$80         ; BC 00: if not ",", error
                    fcb       $59                 ; BR 19: branch to il_li_line
il_test_run         fcb       $85,'RU','N'+$80    ; BC 05: if not "RUN", branch to il_test_clear
                    fcb       $38,$0A             ; J 00A: branch to il_run
il_test_clear       fcb       $86,'CLEA','R'+$80  ; BC 06: if not "CLEAR", branch to il_test_rem
                    fcb       $2B                 ; MT : mark basic program space empty
il_test_rem         fcb       $84,'RE','M'+$80    ; BC 04: if not "REM, branch to il_assign
                    fcb       $1D                 ; NX : next BASIC statement
                    fcb       $A0                 ; BV 00: if not variable, error
il_assign           fcb       $80,'='+$80         ; BC 00: if not "=", error
                    fcb       $38,$14             ; J 014: branch to il_let
il_expr             fcb       $85,'-'+$80         ; if not "-", branch to il_expr_plus
                    fcb       $30,$D3             ; JS 0D3: call il_term
                    fcb       $17                 ; NE : negate
                    fcb       $64                 ; BR 24: branch to il_expr1
il_expr_plus        fcb       $81,'+'+$80         ; BC 01: if not "+", branch to il_expr0
il_expr0            fcb       $30,$D3             ; JS 0D3: call il_term
il_expr1            fcb       $85,'+'+$80         ; BC 05: if not "+", branch to il_expr2
                    fcb       $30,$D3             ; JS 0D3: call il_term
                    fcb       $18                 ; AD : add
                    fcb       $5A                 ; BR 1A: branch to il_expr1
il_expr2            fcb       $85,'-'+$80         ; BC 05: if not "-", branch to il_term
                    fcb       $30,$D3             ; JS 0D3: call il_term
                    fcb       $19                 ; SU : subtract
                    fcb       $54                 ; BR 14: branch to il_expr1
il_expr3            fcb       $2F                 ; RT : return
il_term             fcb       $30,$E2             ; JS 0E2: call il_fact
il_term0            fcb       $85,'*'+$80         ; BC 05: if not "*", branch to il_term1
                    fcb       $30,$E2             ; JS 0E2: call il_factor
                    fcb       $1A                 ; MP : multiply
                    fcb       $5A                 ; BR 1A: branch to il_term0
il_term1            fcb       $85,'/'+$80         ; if not "/", branch to il_term2
                    fcb       $30,$E2             ; JS 0E2: call il_factor
                    fcb       $1B                 ; DV : divide
                    fcb       $54                 ; BR 14: branch to il_term0
il_term2            fcb       $2F                 ; RT : return
il_factor           fcb       $98,'RN','D'+$80    ; BC 18: if not RND, branch to il_factor1
                    fcb       $A,$80,$80          ; LN : push literal 0x8080
                    fcb       $12                 ; FV : fetch variable rnd_seed
                    fcb       $A,$09,$29          ; LN : push literal 0x0929
                    fcb       $1A                 ; MP : multiply
                    fcb       $A,$1A,$85          ; LN : push literal 0x1A85
                    fcb       $18                 ; AD : add
                    fcb       $13                 ; SV : store variable rnd_seed
                    fcb       9,$80               ; LB : push literal byte 0x80
                    fcb       $12                 ; FV : fetch variable rnd_seed
                    fcb       1                   ; SX 01: stack exchange
                    fcb       $B                  ; DS : duplicate stack top
                    fcb       $31,$30             ; JS 130: call il_rn_paren
                    fcb       $61                 ; BR 21: branch to il_factor2
il_factor1          fcb       $72                 ; BR 32: branch to il_usr
il_factor2          fcb       $B                  ; DS : duplicate stack top
                    fcb       4                   ; SX 04: stack exchange
                    fcb       2                   ; SX 02: stack exchange
                    fcb       3                   ; SX 03: stack exchange
                    fcb       5                   ; SX 05: stack exchange
                    fcb       3                   ; SX 03: stack exchange
                    fcb       $1B                 ; DV : divide
                    fcb       $1A                 ; MP : multiply
                    fcb       $19                 ; SU : subtract
                    fcb       $B                  ; DS : duplicate stack top
                    fcb       9,$06               ; LB : push literal byte 0x06
                    fcb       $A,$00,$00          ; LN : push literal number 0x0000
                    fcb       $1C                 ; CP : compare
                    fcb       $17                 ; NE : negate
                    fcb       $2F                 ; RT : return
il_usr              fcb       $8F,'US','R'+$80    ; BC 0F: if not "USR", branch to il_factor3
                    fcb       $80                 ; BC 00: if not "(", error
                    fcb       $A8                 ; if not variable, branch to il_usr1
                    fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $31,$2A             ; JS 12A: call il_us_test_com
                    fcb       $31,$2A             ; JS 12A: call il_us_test_com
                    fcb       $80,')'+$80         ; BC 00: if not ")", error
il_usr1             fcb       $2E                 ; US : machine language call
                    fcb       $2F                 ; RT : return
il_factor3          fcb       $A2                 ; BV 02: if not variable, branch to il_factor4
                    fcb       $12                 ; FV : fetch variable
                    fcb       $2F                 ; RT : return
il_factor4          fcb       $C1                 ; BN 01: if not number, branch to il_lparen
                    fcb       $2F                 ; RT : return
                    fcb       $80,'('+$80         ; BC 00: if not "(", error
il_factor5          fcb       $30,$BC             ; JS 0BC: call il_expr
                    fcb       $80,')'+$80         ; BC 00: if not ")", error
                    fcb       $2F                 ; RT : return
il_us_test_com      fcb       $83,','+$80         ; BC 03: if not ",", branch to il_us_dup
                    fcb       $38,$BC             ; J 0BC: branch to il_expr
il_us_dup           fcb       $B                  ; DS : duplicate stack top
                    fcb       $2F                 ; RT : return
il_rn_paren         fcb       $80,'('+$80         ; BC 00: if not "(", error
                    fcb       $52                 ; BR 12: branch to il_factor5
                    fcb       $2F                 ; RT : return
il_cmpop            fcb       $84,'='+$80         ; if not "=", branch to il_cmpop1
                    fcb       9,$02               ; LB : push literal byte 0x02
                    fcb       $2F                 ; RT ; return
il_cmpop1           fcb       $8E,'<'+$80         ; BR 0E: if not "<", branch to il_cmpop4
                    fcb       $84,'='+$80         ; BR 04: if not "=", branch to il_cmpop2
                    fcb       9,$93               ; LB : push literal byte 0x93
                    fcb       $2F                 ; RT : return
il_cmpop2           fcb       $84,'>'+$80         ; BR 04: if not ">", branch to il_cmpop3
                    fcb       9,$05               ; LB : push literal byte 0x05
                    fcb       $2F                 ; RT : return
il_cmpop3           fcb       9,$91               ; LB : push literal byte 0x91
                    fcb       $2F                 ; RT : return
il_cmpop4           fcb       $80,'>'+$80         ; BR 00: if not ">", error
                    fcb       $84,'='+$80         ; BR 04: if not "=", branch to il_cmpop5
                    fcb       9,$06               ; LB : push literal byte 0x06
                    fcb       $2F                 ; RT : return
il_cmpop5           fcb       $84,'<'+$80         ; BR 04: if not "<", branch to il_cmpop6
                    fcb       9,$95               ; LB : push literal byte 0x95
                    fcb       $2F                 ; RT : return
il_cmpop6           fcb       9,$04               ; LB : push literal byte 0x04
                    fcb       $2F                 ; RT :return
                    fcb       0
                    fcb       0

;*******************************************************************************
; not called: reference code for break check for MIKBUG/MINIBUG monitors
;*******************************************************************************
;                   jmp       minibug_chkbreak
                    jmp       mikbug_chkbreak
