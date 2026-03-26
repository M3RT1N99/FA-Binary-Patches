#include <windows.h>
#include <cstdint>
#include <cmath>

extern "C" void Log(const char* fmt, ...);

struct Vector3 {
    float x, y, z;
};

// --- Game Offsets & Addresses ---
static const uintptr_t ADDR_G_SIM = 0x010A63F0;        // Global pointer to Moho::CSim
static const uintptr_t OFF_SIM_UPDATE_LIST = 0xA5C;    // CSim + 0xA5C: Head of mUpdateEntities circular list (TDatList)
static const uintptr_t OFF_NODE_TO_ENTITY = 0x60;      // Offset from TDatListItem (node) back to Entity start (mUpdateChain)
static const uintptr_t OFF_ENTITY_POS_X = 0x160;       // Entity + 0x160: float mLastTrans.pos.x
static const uintptr_t OFF_ENTITY_POS_Z = 0x168;       // Entity + 0x168: float mLastTrans.pos.z
static const uintptr_t OFF_NAV_PATHFINDER = 0x10;      // CAiPathNavigator + 0x10: Pointer to CAiPathFinder
static const uintptr_t OFF_PF_UNIT = 0x18;             // CAiPathFinder + 0x18: Pointer to Moho::Unit

// --- Separation Parameters ---
static const float MIN_DISTANCE_SQ = 36.0f;           // 6.0f separation radius squared
static const float MAX_NUDGE = 0.2f;                  // Maximum displacement per axis per tick

// --- Safety Checks ---

// Verifies if a pointer is within the valid Moho object heap and 4-byte aligned
inline bool IsSafePtr(uintptr_t ptr) {
    return (ptr >= 0x00400000 && ptr <= 0x3F000000 && (ptr % 4 == 0));
}

// Validates if an entity pointer actually points to a living Moho::Unit
inline bool IsValidUnit(uintptr_t unit) {
    if (!IsSafePtr(unit)) return false;

    // Check VTable range to ensure it's a valid scriptable object (Entity/Unit/Projectile)
    uintptr_t vt = *(uintptr_t*)unit;
    if (vt < 0x00900000 || vt > 0x01200000) return false;

    // Sanity check coordinates (skip null entities and NaNs)
    float x = *(float*)(unit + OFF_ENTITY_POS_X);
    float z = *(float*)(unit + OFF_ENTITY_POS_Z);
    if (x == 0.0f && z == 0.0f) return false;
    if (x != x || z != z) return false; // NaN check

    return true;
}

#if 0 // Unit Separation Hook - Set to 1 to enable for testing
// --- Separation Logic ---

// Injected logic to apply a 'soft' separation nudge to the unit's immediate target position
extern "C" void ApplyTransientSeparation(uintptr_t navigator, Vector3* outPos) {
    if (!outPos || !IsSafePtr(navigator)) return;

    // Navigate from Navigator -> PathFinder -> Owner Unit
    uintptr_t pf = *(uintptr_t*)(navigator + OFF_NAV_PATHFINDER);
    if (!IsSafePtr(pf)) return;

    uintptr_t unitA = *(uintptr_t*)(pf + OFF_PF_UNIT);
    if (!IsValidUnit(unitA)) return;

    // Get the global simulation state
    uintptr_t gs_ptr = *(uintptr_t*)ADDR_G_SIM;
    if (!IsSafePtr(gs_ptr)) return;

    float ax = *(float*)(unitA + OFF_ENTITY_POS_X);
    float az = *(float*)(unitA + OFF_ENTITY_POS_Z);

    // Access the simulation's update list (sentinel node for circular list)
    uintptr_t sentinel = gs_ptr + OFF_SIM_UPDATE_LIST;
    uintptr_t curr = *(uintptr_t*)sentinel;

    int checked = 0;
    int safety = 0;

    // Iterate through units in the update chain to find neighbors
    while (curr != sentinel && safety++ < 1000 && checked < 2) {
        uintptr_t nextNode = *(uintptr_t*)curr;
        uintptr_t unitB = curr - OFF_NODE_TO_ENTITY;

        if (unitB != unitA && IsValidUnit(unitB)) {
            float bx = *(float*)(unitB + OFF_ENTITY_POS_X);
            float bz = *(float*)(unitB + OFF_ENTITY_POS_Z);
            float dx = ax - bx;
            float dz = az - bz;
            float d2 = dx*dx + dz*dz;

            // If a neighbor is within 6.0f radius, calculate a separation nudge
            if (d2 < MIN_DISTANCE_SQ && d2 > 0.01f) {
                float dist = sqrtf(d2);
                float push = (6.0f - dist) * 0.4f; // Push strength factor
                float nudgeX = (dx / dist) * push;
                float nudgeZ = (dz / dist) * push;

                // Clamp to prevent wild jumping (max 0.2 units per tick)
                if (nudgeX >  MAX_NUDGE) nudgeX =  MAX_NUDGE;
                if (nudgeX < -MAX_NUDGE) nudgeX = -MAX_NUDGE;
                if (nudgeZ >  MAX_NUDGE) nudgeZ =  MAX_NUDGE;
                if (nudgeZ < -MAX_NUDGE) nudgeZ = -MAX_NUDGE;

                outPos->x += nudgeX;
                outPos->z += nudgeZ;
                checked++;
            }
        }
        curr = nextNode;
    }
}

// --- Hook Infrastructure ---

extern "C" uint8_t g_Trampoline[16];
uint8_t g_Trampoline[16]; // Hosts original 6 bytes + JMP back

// Naked assembly hook for Moho::CAiPathNavigator::GetTargetPos at 0x005AD8B0
// Calling Convention: Custom/Fast (EAX = Result Buffer, ECX = this/Navigator)
__attribute__((naked)) void Hook_GetTargetPos() {
    __asm__ volatile (
        ".intel_syntax noprefix\n"
        "push eax\n"                // [esp+4] = original buffer pointer (Vector3*)
        "push ecx\n"                // [esp]   = original navigator (this)

        "lea edx, [_g_Trampoline]\n" // Load address of the trampoline
        "call edx\n"                // Call the original function code

        // After original returns:
        "mov ecx, [esp]\n"          // Retrieve original navigator (ECX) from stack
        "mov eax, [esp+4]\n"        // Retrieve original buffer pointer (EAX) from stack
        "push eax\n"                // Push Arg 2: outPos
        "push ecx\n"                // Push Arg 1: navigator
        "call _ApplyTransientSeparation\n"
        "add esp, 8\n"              // Stack cleanup

        "pop ecx\n"                 // Restore original ECX
        "pop eax\n"                 // Restore original EAX as the function return value
        "ret\n"                     // Return to the game's caller
        ".att_syntax\n"
    );
}

// Patches a relative JMP (E9) into the EXE at run-time
void MakeJmp(uintptr_t address, void* target, uint8_t* trampoline) {
    DWORD old;
    VirtualProtect((void*)address, 6, PAGE_EXECUTE_READWRITE, &old);

    // Backup original 6 bytes + append relative JMP back to original+6
    memcpy(trampoline, (void*)address, 6);
    trampoline[6] = 0xE9;
    *(int32_t*)(trampoline + 7) = (int32_t)((address + 6) - ((uintptr_t)trampoline + 11));

    // Write new JMP to our hook function into the original code
    *(uint8_t*)address = 0xE9;
    *(int32_t*)(address + 1) = (int32_t)((uintptr_t)target - (address + 5));
    *(uint8_t*)(address + 5) = 0x90; // NOP filler

    VirtualProtect((void*)address, 6, old, &old);
}
#endif

// --- Initialization & Binary Patches ---

static DWORD WINAPI PatchThread(LPVOID) {
    const char *s1 = "FAIL", *s2 = "FAIL", *s3 = "FAIL", *s4 = "FAIL";
    uintptr_t gs = 0;
    while (!gs) { Sleep(1000); gs = *(uintptr_t*)ADDR_G_SIM; }

    // Patch 1: Collision Resolution (0x00597C83) - Increase prediction granularity (3 -> 1)
    uint8_t* p1 = (uint8_t*)0x00597C83;
    if (p1[0] == 0x83 && p1[1] == 0xC3 && p1[2] == 0x03) {
        DWORD o; VirtualProtect(p1, 3, PAGE_EXECUTE_READWRITE, &o);
        p1[2] = 0x01; VirtualProtect(p1, 3, o, &o);
        s1 = "OK";
    }

    // Patch 2: Collision Filter (0x005D39A9) - Allow group members to avoid each other (jz -> jnz)
    uint8_t* p2 = (uint8_t*)0x005D39A9;
    if (p2[0] == 0x74 && p2[1] == 0x4C) {
        DWORD o; VirtualProtect(p2, 2, PAGE_EXECUTE_READWRITE, &o);
        p2[0] = 0x75; VirtualProtect(p2, 2, o, &o);
        s2 = "OK";
    }

    // Patch 3: Waypoint Arrival Radius (0x00E4F840) - Increase buffer for smoother steering (50 -> 80)
    float* p3 = (float*)0x00E4F840;
    if (*p3 == 50.0f) {
        DWORD o; VirtualProtect(p3, 4, PAGE_EXECUTE_READWRITE, &o);
        *p3 = 80.0f; VirtualProtect(p3, 4, o, &o);
        s3 = "OK";
    }

    // Patch 4: Lookahead Multiplier - Instruction Patch
    // Redirects mulss at 0x005D38AC from [0xE4F714] (0.1f sim-clock!)
    // to [0xE4F9A0] (0.3f steering-only constant)
    // Original: F3 0F 59 05 14 F7 E4 00 (mulss xmm0, [0xE4F714])
    // Patched:  F3 0F 59 05 A0 F9 E4 00 (mulss xmm0, [0xE4F9A0])
    uint8_t* p4 = (uint8_t*)0x005D38AC;
    if (p4[0] == 0xF3 && p4[1] == 0x0F && p4[2] == 0x59 &&
        p4[3] == 0x05 && p4[4] == 0x14 && p4[5] == 0xF7 &&
        p4[6] == 0xE4 && p4[7] == 0x00) {
        DWORD o;
        VirtualProtect(p4, 8, PAGE_EXECUTE_READWRITE, &o);
        p4[4] = 0xA0;  // 0x14 -> 0xA0
        p4[5] = 0xF9;  // 0xF7 -> 0xF9
        VirtualProtect(p4, 8, o, &o);
        s4 = "OK";
    } else {
        Log("[PF] Patch4 FAILED: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            p4[0],p4[1],p4[2],p4[3],p4[4],p4[5],p4[6],p4[7]);
    }

    // Patch 5: Disable moving-check in IsHigherPriorityThan (0x006A8D80)
    // Prevents: "standstill has priority" deadlocks.
    const char* s5 = "FAIL";

    // Patch 5.1 (0x006A8F1E): I'm moving + He's stopped -> I yield (JZ -> 6x NOP)
    uint8_t* p5a = (uint8_t*)0x006A8F1E;
    bool p5a_ok = false;
    if (p5a[0] == 0x0F && p5a[1] == 0x84 && p5a[2] == 0xDE && p5a[3] == 0x00 && p5a[4] == 0x00 && p5a[5] == 0x00) {
        DWORD o; VirtualProtect(p5a, 6, PAGE_EXECUTE_READWRITE, &o);
        memset(p5a, 0x90, 6);
        VirtualProtect(p5a, 6, o, &o);
        p5a_ok = true;
    }

    // Patch 5.2 (0x006A8F3B): I'm stopped + He's moving -> I have priority (JNZ -> 2x NOP)
    uint8_t* p5b = (uint8_t*)0x006A8F3B;
    bool p5b_ok = false;
    if (p5b[0] == 0x75 && p5b[1] == 0xD7) {
        DWORD o; VirtualProtect(p5b, 2, PAGE_EXECUTE_READWRITE, &o);
        memset(p5b, 0x90, 2);
        VirtualProtect(p5b, 2, o, &o);
        p5b_ok = true;
    }
    if (p5a_ok && p5b_ok) s5 = "OK";

    Log("[PF] Done - P1:%s P2:%s P3:%s P4:%s P5:%s\n", s1, s2, s3, s4, s5);
    return 0;
}

extern "C" void InstallSteeringHook() {
    // Start background thread for time-delayed patches
    CreateThread(nullptr, 0, PatchThread, nullptr, 0, nullptr);

#if 0 // Disabled for competitive - toggle for testing
    // Install the global hook for unit target positioning
    DWORD old;
    VirtualProtect(g_Trampoline, 16, PAGE_EXECUTE_READWRITE, &old);
    MakeJmp(0x005AD8B0, (void*)Hook_GetTargetPos, g_Trampoline);
#endif
}
