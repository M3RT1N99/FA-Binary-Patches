// ==========================================================================
// HFriendlyNoCollide.cpp — Skip collision for same-army/allied units
//
//   Patch 1: CheckCollisions yield list (steering layer)
//   Patch 2: DoCollisionsFor army+alliance check (physics layer)
// ==========================================================================

#define SECTION(index, address) ".section h"#index"; .set h"#index","#address";"
#include "../define.h"

asm(
    // Patch 1: Skip yield list loop in CheckCollisions (steering layer)
    SECTION(0, 0x5D3A7F)
    ".byte 0xEB;"

    // Patch 2: Redirect DoCollisionsFor to army-check trampoline (physics layer)
    SECTION(1, 0x597EBF)
    "jmp " QU(FriendlyCollisionCheck) ";"
    "nop; nop; nop; nop; nop; nop; nop;"

    // Patch 3: auskommentiert (path-prediction skip)
    // SECTION(2, 0x597800)
    // "jmp " QU(FriendlyPathCheck) ";"
    // "nop; nop;"
);
