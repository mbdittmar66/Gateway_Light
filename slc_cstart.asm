;
;    Startup code automatically generated and updated by:
;    TASKING Tools for 8051 v7.2r6 (build version: 1.191)
;    Do not edit unless switching off the automatic generation
;    checkbox under:
;        Project | Project Options | Processor | Startup Code
;
;    Copyright 1999-2004 Altium BV
;
$CASE
$DEBUG
        NAME     _CSTART

        EXTRN    CODE(_?main)
        EXTRN    CODE(__LK_B_INITIALIZE)
        EXTRN    CODE(__ENDINIT)
        EXTRN    CODE(LARGE)

        PUBLIC   __START
        PUBLIC   __STKSTART
        PUBLIC   __EXIT

?STACK  SEGMENT IDATA
        RSEG ?STACK
__STKSTLAB:
        DS      100
__STKSTART      IDATA __STKSTLAB - 1

        PUBLIC  __HEAPSTART
        PUBLIC  __HEAPLENGTH
        PUBLIC  __HEAPEND

?HEAP SEGMENT XDATA
        RSEG ?HEAP
__HEAPSTART:
        DS      50
__HEAPEND:
__HEAPLENGTH   XDATA  __HEAPEND - __HEAPSTART

        PUBLIC __TOP_OF_VIRT_STACK
?VIRT_STACK    SEGMENT XDATA
        RSEG    ?VIRT_STACK
        DS      1000
__TOP_OF_VIRT_STACK:

CSEG AT 0x0000

        LCALL __START

STARTUP SEGMENT CODE
        RSEG STARTUP
__START:
        CLR     A
        MOV     PSW, A

        CLR     EA
        MOV     SP, #LOW(__STKSTART)

        EXTRN   DATA(__SP)
        PUSH    0A8H
        CLR     0AFH
        MOV     __SP, #HIGH(__TOP_OF_VIRT_STACK)
        MOV     __SP+1, #LOW(__TOP_OF_VIRT_STACK)
        POP     0A8H 

        LCALL   __LK_B_INITIALIZE

        MOV     PSW, #(0*8)

        USING   0
        LCALL   _?main

__EXIT:
__STOP:
        SJMP    __STOP

        END
