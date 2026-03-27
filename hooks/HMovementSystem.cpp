#include "../define.h"
asm(
  ".section h0; .set h0,0x5D32BD;"
  "jmp " QU(OnSteeringTickWrapper) ";"
  "nop;"
  "nop;"
  "nop;"
  "nop;"
  "nop;"
);
