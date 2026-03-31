// ==========================================================================
// FriendlyNoCollide.cpp — Skip collision for friendly + allied units
//
// Two trampolines:
//
//   1. FriendlyCollisionCheck — hooks Sim::DoCollisionsFor (0x597EBF)
//      Skips AddImpulse for same-army/allied units (physics layer)
//
//   2. FriendlyPathCheck — hooks sub_597800 prologue (0x597800)
//      Skips path-prediction collision for same-army/allied units (steering layer)
//      This prevents units from seeing friendlies as path blockades.
//
// Both use: Unit+0x154 = mArmy (CArmyImpl*), CArmyImpl+0x08 = army index / IArmy*
// IArmy::IsAlly (0x5BD630): __fastcall, ECX=armyIndex, EDX=IArmy*
// ==========================================================================

#include "CObject.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"

static const uint32_t s_IsSourceUnit = 0x62EEA0;
static const uint32_t s_IsAlly = 0x5BD630;

// ======================== Trampoline 1: DoCollisionsFor =======================
// Hook site: 0x597EBF (12 bytes)
// EBP = our unit, ESI = other unit

void FriendlyCollisionCheck()
{
    asm(
        "mov eax, [ebp+0x154];"
        "cmp eax, [esi+0x154];"
        "jz 10f;"

        "push edx;"
        "push ecx;"
        "mov edx, [esi+0x154];"
        "mov ecx, [edx+0x8];"
        "lea edx, [eax+0x8];"
        "call dword ptr [%[ally]];"
        "pop ecx;"
        "pop edx;"
        "test eax, eax;"
        "jnz 10f;"

        "mov ebx, 2;"
        "mov edi, ebp;"
        "call dword ptr [%[isrc]];"
        "jmp 0x597ECB;"

        "10:"
        "mov al, 1;"
        "jmp 0x597ECB;"
        :
        : [ally] "m"(s_IsAlly),
          [isrc] "m"(s_IsSourceUnit)
        :
    );
}

// ======================== Trampoline 2: sub_597800 (path prediction) ==========
// Hook site: 0x597800 (7 bytes: sub esp,0x98; push ebx)
// Params: a1=EAX (steering), a2=EDX (unit1), a3=ECX (unit2), a4=[esp+4]
// Return: ret (caller cleans stack)

void FriendlyPathCheck()
{
    asm(
        // Quick same-army check
        "push eax;"
        "mov eax, [edx+0x154];"
        "cmp eax, [ecx+0x154];"
        "jz 20f;"

        // Alliance check (preserves EDX/ECX)
        "push edx;"
        "push ecx;"
        "mov ecx, [ecx+0x154];"     // a3 army ptr
        "mov ecx, [ecx+0x8];"       // a3 army index
        "lea edx, [eax+0x8];"       // a2 IArmy*
        "call dword ptr [%[ally]];"  // IsAlly
        "pop ecx;"
        "pop edx;"
        "test eax, eax;"
        "jnz 20f;"                  // allied → skip

        // Enemy → execute original prologue, continue
        "pop eax;"
        "sub esp, 0x98;"
        "push ebx;"
        "jmp 0x597807;"             // after prologue

        // Friendly/allied → return early
        "20:"
        "pop eax;"
        "ret;"                      // caller cleans stack
        :
        : [ally] "m"(s_IsAlly)
        :
    );
}
