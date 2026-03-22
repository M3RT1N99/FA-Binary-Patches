#include <windows.h>
#include <cstdio>
#include <cstdint>
#include <cmath>
#include "../include/engine.h"

extern void Log(const char* fmt, ...);

// Original function pointer (trampoline)
void* Trampoline_CalculateForces = nullptr;

// We will implement our custom steering logic here
extern "C" int Hook_CalculateForces_C(void* entity1, void* entity2) {
    // 1. Call the original Collision Math
    int result = 0;
    void* tramp = Trampoline_CalculateForces;
    
    // Call the trampoline passing entity1 in EAX and entity2 on the stack
    asm volatile (
        "mov %1, %%eax\n"
        "push %2\n"            // Push param_1 to stack
        "call *%3\n"
        "add $4, %%esp\n"      // Clean up pushed parameter
        "mov %%eax, %0\n"
        : "=r" (result)
        : "r" (entity1), "r" (entity2), "r" (tramp)
        : "eax", "ecx", "edx", "memory"
    );

    // 2. Here we would add custom RVO / Separation vectors.
    static int logCount = 0;
    if (logCount < 50) {
        Log("[SteeringHook] ResolvePossibleCollision: Entity1=%08X Entity2=%08X\n", (uint32_t)entity1, (uint32_t)entity2);
        logCount++;
    }

    // Example logic ready for integration:
    // CBlueprint* bp = unit->GetBlueprint();
    // float maxForce = bp->Physics.MaxSteerForce;
    // Iterate Sim->units, find nearby, calculate vector away from them, add to steering accel.

    return result;
}

// Naked wrapper to capture EAX and stack argument
__attribute__((naked)) void Hook_CalculateForces_Naked() {
    asm volatile (
        "push 4(%esp)\n"         // Push param_1 (arg2)
        "push %eax\n"            // Push EAX (arg1)
        "call _Hook_CalculateForces_C\n"
        "add $8, %esp\n"         // Clean up our pushed C arguments
        "ret\n"
    );
}

// Memory patching utility
void PatchJmp(uint32_t targetAddr, uint32_t hookAddr, int bytesToNop = 0) {
    DWORD oldProtect;
    VirtualProtect((void*)targetAddr, 5 + bytesToNop, PAGE_EXECUTE_READWRITE, &oldProtect);
    
    uint8_t* pTarget = (uint8_t*)targetAddr;
    pTarget[0] = 0xE9; // JMP
    *(uint32_t*)(pTarget + 1) = hookAddr - targetAddr - 5;
    
    for (int i = 0; i < bytesToNop; i++) {
        pTarget[5 + i] = 0x90; // NOP
    }
    
    VirtualProtect((void*)targetAddr, 5 + bytesToNop, oldProtect, &oldProtect);
}

// Install the detour
void InstallSteeringHook() {
    uint32_t targetAddr = 0x00596F30; // ResolvePossibleCollision
    
    // The first 5 bytes of 0x00596F30 are:
    // 83 EC 60    SUB ESP, 0x60
    // 53          PUSH EBX
    // 55          PUSH EBP
    uint8_t origBytes[5] = { 0x83, 0xEC, 0x60, 0x53, 0x55 };
    
    // Verify bytes match to avoid crashing
    if (memcmp((void*)targetAddr, origBytes, 5) != 0) {
        Log("[SteeringHook] ERROR: Signature mismatch at 0x00596F30. Hook aborted.\n");
        return;
    }

    // Allocate 10 bytes for trampoline (5 original bytes + 5 byte JMP)
    Trampoline_CalculateForces = VirtualAlloc(NULL, 10, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    
    // Copy original bytes
    memcpy(Trampoline_CalculateForces, origBytes, 5);
    
    // Write JMP back to original function + 5
    uint8_t* tramp = (uint8_t*)Trampoline_CalculateForces;
    tramp[5] = 0xE9; // JMP
    *(uint32_t*)(tramp + 6) = (targetAddr + 5) - (uint32_t)(tramp + 5) - 5;

    // Apply the detour jump
    PatchJmp(targetAddr, (uint32_t)&Hook_CalculateForces_Naked, 0); // 0 byte NOP needed
    
    Log("[SteeringHook] Successfully detoured ResolvePossibleCollision!\n");
}
