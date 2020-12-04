;*******************************************************************************
;* Include   : MCU.INC
;* Programmer: Tony Papadimitriou <tonyp@acm.org>
;* Purpose   : General MCU selection for ASM11
;* Language  : Motorola/Freescale/NXP 68HC11 Assembly Language (aspisys.com/ASM11)
;* Status    : FREEWARE Copyright (c) 2020 by Tony Papadimitriou <tonyp@acm.org>
;* Original  : http://www.aspisys.com/code/hc08/mcu.html
;*******************************************************************************

                    #Uses     macros.inc

; MCU specific include selector
; This file will decide (based on user conditionals) which MCU #INCLUDE to use.

          #ifdef F1
                    #Uses     exp-f1.inc
          #else ifdef NOICE
MHZ                 def       16
                    #Uses     noice.inc
          #else ifdef E2
MHZ                 def       8
                    #Uses     811e2.inc
          #else ifdef E9
MHZ                 def       8
                    #Uses     711e9.inc
          #else ifdef WSI
                    #Uses     wsi.inc
          #else ifdef EVBU
MHZ                 def       8
                    #Uses     evbu.inc
          #else ifdef RAMF1
MHZ                 def       16
                    #Uses     ram-f1.inc
          #else
                    #Uses     exp-f1.inc          ;default MCU
          #endif
