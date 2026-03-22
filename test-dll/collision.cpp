#include <windows.h>
#include <cstdint>

extern "C" void Log(const char* fmt, ...);

struct Vector3 {
    float x, y, z;
};

static const uintptr_t ADDR_G_SIM = 0x010A63F0;
static const uintptr_t OFF_SIM_UPDATE_LIST = 0xA5C;
static const uintptr_t OFF_NODE_TO_ENTITY = 0x60;
static const uintptr_t OFF_ENTITY_POS_X = 0x160;
static const uintptr_t OFF_ENTITY_POS_Z = 0x168;
static const uintptr_t OFF_NAV_PATHFINDER = 0x10;
static const uintptr_t OFF_PF_UNIT = 0x18;
static const uintptr_t VTABLE_UNIT = 0x00E2A574;

inline bool IsSafePtr(uintptr_t ptr) {
    return (ptr >= 0x00400000 && ptr <= 0x3F000000 && (ptr % 4 == 0));
}

inline bool IsValidUnit(uintptr_t unit) {
    if (!IsSafePtr(unit)) return false;
    uintptr_t* vt = (uintptr_t*)unit;
    // Strict VTable check to prevent crashes with effects/projectiles
    if (IsSafePtr((uintptr_t)vt) && *vt == VTABLE_UNIT) return true;
    return false;
}

// Minimalist Separation Logic
extern "C" void ApplyTransientSeparation(uintptr_t navigator, Vector3* outPos) {
    if (!outPos || !IsSafePtr(navigator)) return;

    // Check if it's a Land Unit navigator
    uintptr_t pf = *(uintptr_t*)(navigator + OFF_NAV_PATHFINDER);
    if (!IsSafePtr(pf)) return;
    
    uintptr_t unitA = *(uintptr_t*)(pf + OFF_PF_UNIT);
    if (!IsValidUnit(unitA)) return;

    uintptr_t gs_ptr = *(uintptr_t*)ADDR_G_SIM;
    if (!IsSafePtr(gs_ptr)) return;

    float ax = *(float*)(unitA + OFF_ENTITY_POS_X);
    float az = *(float*)(unitA + OFF_ENTITY_POS_Z);

    uintptr_t sentinel = gs_ptr + OFF_SIM_UPDATE_LIST;
    uintptr_t curr = *(uintptr_t*)sentinel; 

    int checked = 0;
    int safety = 0;
    const float limitSq = 25.0f; // 5.0f radius

    while (curr != sentinel && safety++ < 50 && checked < 2) {
        uintptr_t next = *(uintptr_t*)curr;
        uintptr_t unitB = curr - OFF_NODE_TO_ENTITY;

        if (unitB != unitA && IsValidUnit(unitB)) {
            float bx = *(float*)(unitB + OFF_ENTITY_POS_X);
            float bz = *(float*)(unitB + OFF_ENTITY_POS_Z);
            float dx = ax - bx;
            float dz = az - bz;
            float d2 = dx*dx + dz*dz;

            if (d2 < limitSq && d2 > 0.01f) {
                // Apply a small separation nudge
                outPos->x += (dx * 0.02f);
                outPos->z += (dz * 0.02f);
                checked++;
            }
        }
        curr = next;
    }
}

extern "C" uint8_t g_Trampoline[16];
uint8_t g_Trampoline[16];

// FIXED HOOK: EAX must be preserved for the original function!
__attribute__((naked)) void Hook_GetTargetPos() {
    __asm__ volatile (
        ".intel_syntax noprefix\n"
        "push eax\n"                // Save buffer pointer
        "push ecx\n"                // Save navigator pointer
        
        "lea edx, [_g_Trampoline]\n" // Use EDX for trampoline jump
        "call edx\n"                // Call original (uses eax+ecx)
        
        "pop ecx\n"                 // Restore navigator
        "pop eax\n"                 // Restore buffer pointer
        
        "pushad\n"                  // Save context for C function
        "sub esp, 4\n"              // Stack alignment (16-byte)
        "push eax\n"                // Arg 2: Vector3* outPos
        "push ecx\n"                // Arg 1: uintptr_t navigator
        "call _ApplyTransientSeparation\n"
        "add esp, 12\n"
        "popad\n"
        
        "ret\n"
        ".att_syntax\n"
    );
}

void MakeJmp(uintptr_t address, void* target, uint8_t* trampoline) {
    DWORD old;
    VirtualProtect((void*)address, 6, PAGE_EXECUTE_READWRITE, &old);
    memcpy(trampoline, (void*)address, 6);
    trampoline[6] = 0xE9;
    *(int32_t*)(trampoline + 7) = (int32_t)((address + 6) - ((uintptr_t)trampoline + 11));
    
    *(uint8_t*)address = 0xE9;
    *(int32_t*)(address + 1) = (int32_t)((uintptr_t)target - (address + 5));
    *(uint8_t*)(address + 5) = 0x90;
    
    VirtualProtect((void*)address, 6, old, &old);
}

static DWORD WINAPI PatchThread(LPVOID) {
    uintptr_t gs = 0;
    while (!gs) { Sleep(1000); gs = *(uintptr_t*)ADDR_G_SIM; }
    
    // P1: Granularity
    uint8_t* p1 = (uint8_t*)0x00597C83;
    if (p1[0] == 0x83) {
        DWORD o; VirtualProtect(p1, 3, PAGE_EXECUTE_READWRITE, &o);
        p1[2] = 0x01; VirtualProtect(p1, 3, o, &o);
    }
    // P2: Group Collision
    uint8_t* p2 = (uint8_t*)0x005D39A9;
    if (p2[0] == 0x74) {
        DWORD o; VirtualProtect(p2, 2, PAGE_EXECUTE_READWRITE, &o);
        p2[0] = 0x75; VirtualProtect(p2, 2, o, &o);
    }
    // P3: Waypoint Arrival Radius
    float* p3 = (float*)0x00E4F840;
    if (*p3 == 50.0f) {
        DWORD o; VirtualProtect(p3, 4, PAGE_EXECUTE_READWRITE, &o);
        *p3 = 80.0f; VirtualProtect(p3, 4, o, &o);
    }
    Log("[PF] Binary Patches and Separation Hook applied successfully\n");
    return 0;
}

extern "C" void InstallSteeringHook() {
    CreateThread(nullptr, 0, PatchThread, nullptr, 0, nullptr);
    DWORD old;
    VirtualProtect(g_Trampoline, 16, PAGE_EXECUTE_READWRITE, &old);
    MakeJmp(0x005AD8B0, (void*)Hook_GetTargetPos, g_Trampoline);
}