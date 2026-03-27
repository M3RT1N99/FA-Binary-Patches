// ==========================================================================
// HGetTargetPos.cpp — Three-point hook for GetTargetPos (0x5AD8B0)
//
// Hook 0 (Prolog 0x5AD8B0, 6B): Save ECX (navigator) to global
// Hook 1 (Epilog A 0x5AD935, 7B): Formation-leader return — apply offset
// Hook 2 (Epilog B 0x5AD9B1, 9B): Cell-target return — apply offset
// ==========================================================================

#include "../define.h"

asm(R"(

.section h0
.set h0, 0x5AD8B0

    jmp GetTargetPosPrologue
    .byte 0x90

.section h1
.set h1, 0x5AD935

    jmp GetTargetPosEpilogueA
    .byte 0x90, 0x90

.section h2
.set h2, 0x5AD9B1

    jmp GetTargetPosEpilogueB
    .byte 0x90, 0x90, 0x90, 0x90

)");
