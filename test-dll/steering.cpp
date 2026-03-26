// ==========================================================================
// steering.cpp - Steering-Layer Separation Force Hook
//
// Hooks CAiSteeringImpl::CheckCollisions (0x5d3740) to add a separation
// force to CUnitMotion::mForce AFTER the engine collision detection.
// Units dodge each other.
//
// Pipeline: A* path -> Steering waypoints -> CheckCollisions -> *HOOK* -> Physics
//
// Hotkey: Shift+8 Toggle
// ==========================================================================
#include <windows.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" void Log(const char* fmt, ...);

// Extern: NbrCache from collision.cpp
struct NbrEntry { float x, z; uint32_t addr; float size; };
extern NbrEntry g_NbrCache[];
extern volatile LONG g_NbrIdx;

// Feature Flag
static bool g_SepEnabled = true;

// Parameters
#define SEP_STRENGTH      1.0f    // Waypoint offset per neighbor (in world units)
#define SEP_RANGE_MULT    2.5f    // Scan radius = (mySize + nbrSize) * MULT
#define SEP_MAX_OFFSET    3.0f    // Max waypoint displacement (world units)
#define SEP_UNDO_SLOTS    512     // Undo entries for destination rollback
#define SEP_VTABLE_MIN    0x00900000
#define SEP_VTABLE_MAX    0x01200000

// Debug
static LONG g_sepDbgCalls = 0;
static LONG g_sepDbgForced = 0;
static float g_sepDbgMaxForce = 0.0f;
static LONG g_sepOffsetDbg = 0;

// Undo map: stores last offset per steering instance
// so we can reset destination before applying new offset
struct UndoEntry { uint32_t steeringAddr; float lastOffX, lastOffZ; };
static UndoEntry g_UndoMap[SEP_UNDO_SLOTS];

static UndoEntry* findUndo(uint32_t addr) {
    int idx = (addr >> 4) % SEP_UNDO_SLOTS;
    for (int i = 0; i < 8; i++) {
        int slot = (idx + i) % SEP_UNDO_SLOTS;
        if (g_UndoMap[slot].steeringAddr == addr)
            return &g_UndoMap[slot];
        if (g_UndoMap[slot].steeringAddr == 0) {
            g_UndoMap[slot].steeringAddr = addr;
            g_UndoMap[slot].lastOffX = 0;
            g_UndoMap[slot].lastOffZ = 0;
            return &g_UndoMap[slot];
        }
    }
    return nullptr;
}

// Read blueprint size (same function as collision.cpp)
static float GetUnitSize(uint32_t unit) {
    uint32_t vtable = *(uint32_t*)unit;
    if (vtable < SEP_VTABLE_MIN || vtable > SEP_VTABLE_MAX) return 1.0f;
    typedef uint32_t (__thiscall *GetBP_fn)(uint32_t);
    GetBP_fn getBP = (GetBP_fn)(*(uint32_t*)(vtable + 0x1C));
    if (!getBP) return 1.0f;
    uint32_t bp = getBP(unit);
    if (!bp) return 1.0f;
    float sx = *(float*)(bp + 0xAC);
    float sz = *(float*)(bp + 0xB4);
    return (sz > sx) ? sz : sx;
}

// ==========================================================================
// ApplySeparation - Core function
//
// Reads NbrCache, computes separation vector, nudges mVelocity.
// Velocity is recalculated next frame by steering - no permanent
// state corruption, no accumulation.
//
// Called AFTER CheckCollisions with the CAiSteeringImpl* this pointer.
// ==========================================================================
static void __cdecl ApplySeparation(uint32_t steeringPtr) {
    g_sepDbgCalls++;
    if (!g_SepEnabled) return;

    // Unit pointer: Steering+0x20 (from IDA: [edi+20h])
    uint32_t unit = *(uint32_t*)(steeringPtr + 0x20);
    if (!unit) return;
    uint32_t vtable = *(uint32_t*)unit;
    if (vtable < SEP_VTABLE_MIN || vtable > SEP_VTABLE_MAX) return;

    float myX = *(float*)(unit + 0x160);
    float myZ = *(float*)(unit + 0x168);
    float mySize = GetUnitSize(unit);

    // Compute separation vector (away from neighbors)
    float pushX = 0.0f, pushZ = 0.0f;
    int nbrHits = 0;

    LONG nbrCount = g_NbrIdx;
    if (nbrCount > 512) nbrCount = 512;

    for (LONG i = 0; i < nbrCount; i++) {
        if (g_NbrCache[i].addr == unit) continue;

        float dx = myX - g_NbrCache[i].x;
        float dz = myZ - g_NbrCache[i].z;
        float distSq = dx * dx + dz * dz;
        float minDist = (mySize + g_NbrCache[i].size) * SEP_RANGE_MULT;

        if (distSq < minDist * minDist && distSq > 0.01f) {
            float dist = sqrtf(distSq);
            float penetration = 1.0f - dist / minDist;
            float force = SEP_STRENGTH * penetration;
            pushX += (dx / dist) * force;
            pushZ += (dz / dist) * force;
            nbrHits++;
        }
    }

    if (nbrHits == 0) return;

    // Clamp offset
    float pushMag = sqrtf(pushX * pushX + pushZ * pushZ);
    if (pushMag > SEP_MAX_OFFSET) {
        float scale = SEP_MAX_OFFSET / pushMag;
        pushX *= scale;
        pushZ *= scale;
        pushMag = SEP_MAX_OFFSET;
    }

    // Nudge velocity: CUnitMotion+0x38 = mVelocity (Vector3f)
    // Small lateral push per frame. Next frame steering recalculates
    // velocity completely - no accumulation, no state corruption.
    uint32_t motion = *(uint32_t*)(steeringPtr + 0x60);
    if (!motion) return;

    float* velX = (float*)(motion + 0x38);
    float* velZ = (float*)(motion + 0x40); // +0x38=x, +0x3C=y, +0x40=z

    *velX += pushX;
    *velZ += pushZ;

    // Debug
    if (pushMag > g_sepDbgMaxForce) g_sepDbgMaxForce = pushMag;
    g_sepDbgForced++;

    if (g_sepOffsetDbg < 5) {
        Log("[SEP] unit=0x%08X nbrs=%d push=(%.2f,%.2f)\n",
            unit, nbrHits, pushX, pushZ);
        g_sepOffsetDbg++;
    }
}

// ==========================================================================
// Hook Installation: Detour on CheckCollisions (0x5d3740)
//
// Original Prologue (6 bytes):
//   55          push ebp
//   8B EC       mov ebp, esp
//   83 E4 F8    and esp, -8
//
// Calling Convention: __stdcall (retn 4), Parameter: this on stack
//
// Our detour:
//   1. Saves this pointer
//   2. Calls original CheckCollisions via trampoline
//   3. Calls ApplySeparation(this)
//   4. Returns via retn 4
// ==========================================================================
static uint8_t* g_SteerTrampoline = nullptr;  // Original prologue + JMP back
static uint8_t* g_SteerDetour = nullptr;       // Our wrapper

static const uint8_t ORIG_PROLOGUE[] = {0x55, 0x8B, 0xEC, 0x83, 0xE4, 0xF8};

void InstallSeparationHook() {
    uint8_t* hookSite = (uint8_t*)0x005D3740;

    // Verify original bytes
    if (memcmp(hookSite, ORIG_PROLOGUE, 6) != 0) {
        Log("[SEP] FAILED: bytes at 0x5D3740 don't match expected prologue\n");
        Log("[SEP] Got: %02X %02X %02X %02X %02X %02X\n",
            hookSite[0], hookSite[1], hookSite[2],
            hookSite[3], hookSite[4], hookSite[5]);
        return;
    }

    // --- Trampoline: Original prologue + JMP back ---
    g_SteerTrampoline = (uint8_t*)VirtualAlloc(NULL, 4096,
        MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!g_SteerTrampoline) { Log("[SEP] VirtualAlloc trampoline FAILED\n"); return; }

    int off = 0;
    // Original 6 bytes
    memcpy(g_SteerTrampoline + off, ORIG_PROLOGUE, 6); off += 6;
    // JMP to original function + 6 (0x5d3746)
    g_SteerTrampoline[off++] = 0xE9;
    int32_t relBack = (int32_t)(0x005D3746 - (uint32_t)(g_SteerTrampoline + off + 4));
    memcpy(g_SteerTrampoline + off, &relBack, 4); off += 4;

    // --- Detour: Wrapper that calls original, then our function ---
    g_SteerDetour = (uint8_t*)VirtualAlloc(NULL, 4096,
        MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!g_SteerDetour) { Log("[SEP] VirtualAlloc detour FAILED\n"); return; }

    off = 0;
    // Detour is stdcall: [esp+4] = this
    //
    // mov eax, [esp+4]       ; eax = this
    g_SteerDetour[off++] = 0x8B; g_SteerDetour[off++] = 0x44;
    g_SteerDetour[off++] = 0x24; g_SteerDetour[off++] = 0x04;
    // push eax               ; save this (for later)
    g_SteerDetour[off++] = 0x50;
    // push eax               ; arg for original CheckCollisions (stdcall)
    g_SteerDetour[off++] = 0x50;
    // call g_SteerTrampoline ; original CheckCollisions (stdcall, pops arg)
    g_SteerDetour[off++] = 0xE8;
    int32_t relTramp = (int32_t)((uint32_t)g_SteerTrampoline - (uint32_t)(g_SteerDetour + off + 4));
    memcpy(g_SteerDetour + off, &relTramp, 4); off += 4;
    // pop eax                ; restore saved this
    g_SteerDetour[off++] = 0x58;
    // push eax               ; arg for ApplySeparation (cdecl)
    g_SteerDetour[off++] = 0x50;
    // call ApplySeparation
    g_SteerDetour[off++] = 0xE8;
    int32_t relSep = (int32_t)((uint32_t)&ApplySeparation - (uint32_t)(g_SteerDetour + off + 4));
    memcpy(g_SteerDetour + off, &relSep, 4); off += 4;
    // add esp, 4             ; cdecl cleanup
    g_SteerDetour[off++] = 0x83; g_SteerDetour[off++] = 0xC4; g_SteerDetour[off++] = 0x04;
    // retn 4                 ; stdcall return (pops original this arg from caller)
    g_SteerDetour[off++] = 0xC2; g_SteerDetour[off++] = 0x04; g_SteerDetour[off++] = 0x00;

    // --- Patch: JMP from CheckCollisions to our detour ---
    DWORD old;
    if (VirtualProtect(hookSite, 6, PAGE_EXECUTE_READWRITE, &old)) {
        // JMP rel32 (5 bytes) + NOP (1 byte)
        hookSite[0] = 0xE9;
        int32_t relJmp = (int32_t)((uint32_t)g_SteerDetour - (0x005D3740 + 5));
        memcpy(hookSite + 1, &relJmp, 4);
        hookSite[5] = 0x90; // NOP for 6th byte
        VirtualProtect(hookSite, 6, old, &old);
        Log("[SEP] Hook OK at 0x5D3740 -> detour 0x%08X -> tramp 0x%08X\n",
            (uint32_t)g_SteerDetour, (uint32_t)g_SteerTrampoline);
    } else {
        Log("[SEP] VirtualProtect FAILED\n");
    }
}

// Debug log (called from collision.cpp HotkeyThread)
void SeparationDebugLog() {
    Log("[SEP] calls=%d forced=%d maxF=%.2f enabled=%s\n",
        g_sepDbgCalls, g_sepDbgForced, g_sepDbgMaxForce,
        g_SepEnabled ? "ON" : "OFF");
    g_sepDbgCalls = 0;
    g_sepDbgForced = 0;
    g_sepDbgMaxForce = 0.0f;
    g_sepOffsetDbg = 0;
}

// Toggle (Shift+8)
void SeparationToggle() {
    g_SepEnabled = !g_SepEnabled;
    Log("[HOTKEY] Separation = %s\n", g_SepEnabled ? "ON" : "OFF");
}
