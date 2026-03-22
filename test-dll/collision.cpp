#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstdint>

extern "C" void Log(const char* fmt, ...);

struct Vector3 { float x, y, z; };

// Separation parameters
static const float SEPARATION_RADIUS  = 3.0f;
static const float MIN_DISTANCE       = 7.0f;
static const float PUSH_STRENGTH      = 0.8f;
static const float LATERAL_STRENGTH   = 0.5f;
static const float MAX_VELOCITY_ADD   = 0.3f;
static const float PROXIMITY_RADIUS   = 12.0f;

// Hook addresses
static uintptr_t g_HookAddr = 0x005AD8B0; // Moho::CAiPathNavigator::GetTargetPos
uint8_t g_TrampolineBuf[16];

static bool IsSafePtr(uintptr_t p) {
    return p >= 0x00400000 && p <= 0x7FFFFFFF;
}

static bool IsValidUnit(uintptr_t entity) {
    if (!IsSafePtr(entity)) return false;
    // Check vtable range
    uintptr_t vt = *(uintptr_t*)entity;
    if (vt < 0x00E10000 || vt > 0x01200000) return false;
    // Check position is sane (Unit + 0x168 mapped to mLastTrans.pos.x)
    Vector3* pos = (Vector3*)(entity + 0x168);
    if (std::isnan(pos->x) || std::abs(pos->x) > 20000.0f) return false;
    if (std::isnan(pos->z) || std::abs(pos->z) > 20000.0f) return false;
    if (pos->x == 0.0f && pos->z == 0.0f) return false;
    return true;
}

// Called AFTER GetTargetPos logic - modifies outPos transiently
extern "C" void ApplyTransientSeparation(uintptr_t navigator, Vector3* outPos) {
    static bool simReady = false;
    if (!simReady) {
        uintptr_t gs = *(uintptr_t*)0x010a63f0;
        if (!gs || !IsSafePtr(gs)) return;
        uintptr_t nd = *(uintptr_t*)(gs + 0xa60);
        uintptr_t sn = gs + 0xa5c;
        if (nd == sn || nd == 0) return;
        simReady = true;
        Log("v29: Sim ready - transient separation (GetTargetPos) active.\n");
    }

    // Throttle: only run separation every 3rd call per navigator
    // This prevents O(n²) explosion with many units
    static int throttleCount = 0;
    if (++throttleCount % 3 != 0) return;

    static int callCount2 = 0;
    if (++callCount2 % 1000 == 0) {
        Log("v29: hook calls=%d nav=0x%08X outPos=0x%08X\n", 
            callCount2, (uint32_t)navigator, (uint32_t)outPos);
    }

    if (callCount2 % 5000 == 1) {
        // Trace the pointer chain
        uintptr_t pf_test = IsSafePtr(navigator) ? 
                            *(uintptr_t*)(navigator + 0x10) : 0;
        uintptr_t unit_test = IsSafePtr(pf_test) ? 
                              *(uintptr_t*)(pf_test + 0x18) : 0;
        Log("v29: nav=0x%08X pf=0x%08X unit=0x%08X valid=%d\n",
            (uint32_t)navigator, (uint32_t)pf_test,
            (uint32_t)unit_test, IsValidUnit(unit_test) ? 1 : 0);
    }

    if (!IsSafePtr(navigator)) return;
    
    // Get Unit via Navigator -> mPathFinder (+0x10) -> mUnit (+0x18)
    uintptr_t pf = *(uintptr_t*)(navigator + 0x10);
    if (!IsSafePtr(pf)) return;
    uintptr_t unitA = *(uintptr_t*)(pf + 0x18);
    if (!IsValidUnit(unitA)) return;

    static int callCount = 0;
    int thisCall = ++callCount;

    Vector3* pPosA = (Vector3*)(unitA + 0x168);

    // Search global entity list for nearby units
    uintptr_t g_Sim = *(uintptr_t*)0x010a63f0;
    if (!g_Sim) return;

    uintptr_t node     = *(uintptr_t*)(g_Sim + 0xa60);
    uintptr_t sentinel = g_Sim + 0xa5c;
    if (node == sentinel || node == 0) return;

    Vector3 delta = {0, 0, 0};
    int safety = 0;
    int neighborCount = 0;
    uintptr_t curr = node;

    while (curr != 0 && curr != sentinel && safety < 1500) {
        if (!IsSafePtr(curr)) break;
        uintptr_t unitB = curr - 0x68;
        curr = *(uintptr_t*)(curr + 4);
        if (curr == node) break;
        safety++;

        if (unitB == unitA) continue;
        if (!IsValidUnit(unitB)) continue;

        Vector3* pPosB = (Vector3*)(unitB + 0x168);
        float dx = pPosA->x - pPosB->x;
        float dz = pPosA->z - pPosB->z;
        float distSq = dx*dx + dz*dz;

        if (distSq > PROXIMITY_RADIUS * PROXIMITY_RADIUS) continue;
        if (distSq < 0.001f) continue;

        if (distSq < MIN_DISTANCE * MIN_DISTANCE) {
            neighborCount++;
            if (neighborCount > 8) break; // cap at 8 neighbors

            float dist    = std::sqrt(distSq);
            float overlap = MIN_DISTANCE - dist;
            float nx = dx / dist;
            float nz = dz / dist;
            float push = overlap * PUSH_STRENGTH * 0.5f;
            float lx = -nz;
            float lz =  nx;
            float side = overlap * LATERAL_STRENGTH * 0.5f;
            delta.x += nx * push + lx * side;
            delta.z += nz * push + lz * side;
        }
    }

    float magSq = delta.x*delta.x + delta.z*delta.z;
    if (magSq > 0.0001f) {
        float mag = std::sqrt(magSq);
        float clamped = (mag > MAX_VELOCITY_ADD) ? MAX_VELOCITY_ADD : mag;
        float scale = clamped / mag;

        // Modify output vector provided by GetTargetPos
        if (IsSafePtr((uintptr_t)outPos)) {
            // Direct separation nudge with hard cap
            float nudgeX = delta.x * scale;
            float nudgeZ = delta.z * scale;
            // Hard cap per axis: max 0.3 units per tick
            if (nudgeX >  0.3f) nudgeX =  0.3f;
            if (nudgeX < -0.3f) nudgeX = -0.3f;
            if (nudgeZ >  0.3f) nudgeZ =  0.3f;
            if (nudgeZ < -0.3f) nudgeZ = -0.3f;
            outPos->x += nudgeX;
            outPos->z += nudgeZ;

            if (thisCall % 5000 == 1) {
                Log("[TRANSIENT.V29 #%d] unit=0x%08X x=%.2f -> %.2f\n",
                    thisCall, (uint32_t)unitA, 
                    outPos->x - nudgeX, outPos->x);
            }
        }
    }
}

volatile int g_HookCallCount = 0;

// Proxy Naked for GetTargetPos hook
// ecx = navigator, eax = outPos
extern "C" __attribute__((naked)) void Hook_GetTargetPos() {
    asm (
        ".intel_syntax noprefix\n"
        "inc dword ptr [_g_HookCallCount]\n"
        
        // Save navigator (ECX) and outPos (EAX) on stack
        "push eax\n"              // [ESP+4] = outPos
        "push ecx\n"              // [ESP+0] = navigator
        
        // Call original via trampoline - ECX must be navigator (__thiscall)
        // ECX is already navigator, EAX is outPos - just call trampoline
        "call _g_TrampolineBuf\n"
        
        // After original: restore saved values
        "pop ecx\n"               // ECX = navigator
        "pop eax\n"               // EAX = outPos
        
        // Call our separation logic
        "push eax\n"              // arg2: outPos
        "push ecx\n"              // arg1: navigator  
        "call _ApplyTransientSeparation\n"
        "add esp, 8\n"
        
        // Return (GetTargetPos return value is in EAX from original call)
        "ret\n"
        ".att_syntax\n"
    );
}

static void MakeJmp(uintptr_t addr, void* target) {
    DWORD old;
    // Prepare trampoline: 55 8B EC 83 E4 C0 (6 bytes) + JMP back
    VirtualProtect(g_TrampolineBuf, 16, PAGE_EXECUTE_READWRITE, &old);
    uint8_t* pOrig = (uint8_t*)addr;
    for(int i=0; i<6; i++) g_TrampolineBuf[i] = pOrig[i];
    g_TrampolineBuf[6] = 0xE9; // JMP
    *(uint32_t*)&g_TrampolineBuf[7] = (addr + 6) - (uintptr_t)&g_TrampolineBuf[6] - 5;
    
    VirtualProtect((void*)addr, 6, PAGE_EXECUTE_READWRITE, &old);
    *(uint8_t*)addr        = 0xE9;
    *(uint32_t*)(addr + 1) = (uintptr_t)target - addr - 5;
    *(uint8_t*)(addr + 5)  = 0x90; 
    VirtualProtect((void*)addr, 6, old, &old);
}

static DWORD WINAPI HookReachThread(LPVOID) {
    Sleep(3000); // Wait for simulation to settle
    while(true) {
        Log("v29: hook_reach_count=%d\n", g_HookCallCount);
        Sleep(2000);
    }
    return 0;
}

extern "C" void InstallSteeringHook() {
    Log("Applying Transient Separation Hook v29 (GetTargetPos)...\n");
    MakeJmp(g_HookAddr, (void*)Hook_GetTargetPos);
    
    // Verify patch was written
    uint8_t* p = (uint8_t*)g_HookAddr;
    Log("Hook at 0x%08X: bytes=%02X %02X %02X %02X %02X %02X\n",
        g_HookAddr, p[0], p[1], p[2], p[3], p[4], p[5]);
    
    Log("Expected JMP (E9) at first byte.\n");

    // Background thread to log reachability
    CreateThread(nullptr, 0, HookReachThread, nullptr, 0, nullptr);
}
