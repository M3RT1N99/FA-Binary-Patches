#include <windows.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" void Log(const char* fmt, ...);

// avoidance_tuning.cpp exports
extern void InstallAvoidanceTuning();
extern void AvoidanceTuningToggle();
extern void AvoidanceTuningDebugLog();

// lane_system.cpp exports
extern void ComputeLaneOffset(uint32_t unit, float myX, float myZ,
                              float fwdX, float fwdZ, float fwdLen,
                              float mySize, uint32_t tick,
                              float* outOffX, float* outOffZ);
extern void LaneSystemDebugLog();

// ==========================================================================
// FEATURE FLAGS - Shift+1 / Shift+2 to toggle
// ==========================================================================
static bool g_SpreadEnabled = true;
static bool g_StrogoEnabled = false;

// CON_Execute: Engine console command function (0x41CC90)
typedef void (__cdecl *CON_Execute_t)(const char*);
static CON_Execute_t g_ConExecute = (CON_Execute_t)0x0041CC90;

// ==========================================================================
// Spread v5 - Persistent adaptive offset with collision detection
// ==========================================================================
#define SEP_MAX_UNITS     512
#define SEP_INERTIA       0.92f   // Offset inertia (0.92 = slightly faster, was 0.95)
#define SEP_PUSH_STRENGTH 0.5f    // Base repulsion strength
#define SEP_AVOID_STRENGTH 1.5f   // Extra force on collision ahead (was 1.0)
#define SEP_MAX_OFFSET    3.0f    // Maximum offset (was 5.0 - too much for narrow passes)
#define SEP_FALLOFF_DIST  20.0f   // Falloff distance to target
#define SEP_COL_RANGE     1.5f    // Collision detection range (x minDist, was 3.0)
#define SEP_DEAD_TICKS    100     // Ticks until offset entry expires
#define SEP_EXTRA_MARGIN  0.5f
#define SEP_VTABLE_MIN    0x00900000
#define SEP_VTABLE_MAX    0x01200000

// --- Neighbor Cache (reset every tick) ---
struct NbrEntry { float x, z; uint32_t addr; float size; };
NbrEntry g_NbrCache[SEP_MAX_UNITS];  // not static - lane_system.cpp needs access
volatile LONG g_NbrIdx = 0;

// --- Offset Cache (persistent, never fully reset) ---
struct UnitOffset {
    uint32_t addr;
    float offX, offZ;
    uint32_t lastTick;
};
static UnitOffset g_OffsetMap[SEP_MAX_UNITS];

// --- Tick Counter (deterministic, increments per cache reset) ---
static uint32_t g_TickCounter = 0;

// --- Debug ---
static LONG g_dbgCalls = 0, g_dbgPushed = 0, g_dbgCollisions = 0;
static float g_dbgOffsetSum = 0.0f, g_dbgOffsetMax = 0.0f;
static float g_dbgSideSum = 0.0f;
static float g_dbgAvgNbrDist = 0.0f;  // Average neighbor distance
static LONG g_dbgNbrCount = 0;        // Number of neighbor pairs
static float g_dbgAvgDot = 0.0f;      // Average dot (how frontal the collision is)
static LONG g_dbgHeadOn = 0;          // Head-on collisions (dot > 0.7)
static float g_OrigNavDist = 50.0f;

// Read blueprint size
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

// Hash map: O(1) lookup per unit address
static UnitOffset* findOrCreateOffset(uint32_t addr) {
    int idx = (addr >> 4) % SEP_MAX_UNITS;
    for (int i = 0; i < 8; i++) {
        int slot = (idx + i) % SEP_MAX_UNITS;
        if (g_OffsetMap[slot].addr == addr)
            return &g_OffsetMap[slot];
        if (g_OffsetMap[slot].addr == 0 ||
            g_TickCounter - g_OffsetMap[slot].lastTick > SEP_DEAD_TICKS) {
            g_OffsetMap[slot] = {addr, 0.0f, 0.0f, g_TickCounter};
            return &g_OffsetMap[slot];
        }
    }
    return nullptr;
}

static void __cdecl ApplySpread(float* targetPos, uint32_t pathNav) {
    g_dbgCalls++;

    // Debug log every 3s (GetTickCount only for logging, not for calculation)
    static DWORD dbgLastLog = 0;
    DWORD now = GetTickCount();
    if (now - dbgLastLog > 3000) {
        float avgOff = (g_dbgPushed > 0) ? g_dbgOffsetSum / g_dbgPushed : 0.0f;
        float avgSide = (g_dbgCollisions > 0) ? g_dbgSideSum / g_dbgCollisions : 0.0f;
        int colPct = (g_dbgCalls > 0) ? (g_dbgCollisions * 100 / g_dbgCalls) : 0;
        Log("[SPR5] calls=%d pushed=%d col=%d(%d%%) tick=%u\n",
            g_dbgCalls, g_dbgPushed, g_dbgCollisions, colPct, g_TickCounter);
        float avgNbr = (g_dbgNbrCount > 0) ? g_dbgAvgNbrDist / g_dbgNbrCount : 0.0f;
        float avgDot = (g_dbgCollisions > 0) ? g_dbgAvgDot / g_dbgCollisions : 0.0f;
        Log("[SPR5] off=%.2f/%.2f side=%.2f nbrD=%.2f dot=%.2f headon=%d\n",
            avgOff, g_dbgOffsetMax, avgSide, avgNbr, avgDot, g_dbgHeadOn);
        g_dbgCalls = 0; g_dbgPushed = 0; g_dbgCollisions = 0;
        g_dbgOffsetSum = 0.0f; g_dbgOffsetMax = 0.0f; g_dbgSideSum = 0.0f;
        g_dbgAvgNbrDist = 0.0f; g_dbgNbrCount = 0;
        g_dbgAvgDot = 0.0f; g_dbgHeadOn = 0;
        dbgLastLog = now;
    }

    // Neighbor cache reset (deterministic: once when all units have been processed)
    if (g_NbrIdx > 0 && g_dbgCalls == 1) {
        g_NbrIdx = 0;
        g_TickCounter++;
    }

    if (!g_SpreadEnabled) {
        // Still register in neighbor cache
        uint32_t pf = *(uint32_t*)(pathNav + 0x10);
        if (pf) {
            uint32_t u = *(uint32_t*)(pf + 0x18);
            if (u) {
                LONG idx = g_NbrIdx++;
                if (idx < SEP_MAX_UNITS) {
                    g_NbrCache[idx] = {*(float*)(u+0x160), *(float*)(u+0x168), u, GetUnitSize(u)};
                }
            }
        }
        return;
    }

    // Get unit
    uint32_t pathFinder = *(uint32_t*)(pathNav + 0x10);
    if (!pathFinder) return;
    uint32_t unit = *(uint32_t*)(pathFinder + 0x18);
    if (!unit) return;

    float myX = *(float*)(unit + 0x160);
    float myZ = *(float*)(unit + 0x168);
    float mySize = GetUnitSize(unit);

    // Direction to target
    float fwdX = targetPos[0] - myX;
    float fwdZ = targetPos[2] - myZ;
    float fwdLen = sqrtf(fwdX * fwdX + fwdZ * fwdZ);

    // Close to target -> no spread (so unit can arrive)
    if (fwdLen < 3.0f) {
        LONG idx = g_NbrIdx++;
        if (idx < SEP_MAX_UNITS)
            g_NbrCache[idx] = {myX, myZ, unit, mySize};
        return;
    }

    // === DYNAMIC LANE SYSTEM (Bidirectional Lane Separation) ===
    float laneOffX = 0.0f, laneOffZ = 0.0f;
    ComputeLaneOffset(unit, myX, myZ, fwdX, fwdZ, fwdLen,
                      mySize, g_TickCounter, &laneOffX, &laneOffZ);

    if (laneOffX != 0.0f || laneOffZ != 0.0f) {
        // Falloff near target
        float falloff = fwdLen / SEP_FALLOFF_DIST;
        if (falloff > 1.0f) falloff = 1.0f;

        targetPos[0] += laneOffX * falloff;
        targetPos[2] += laneOffZ * falloff;
        g_dbgPushed++;

        float appliedMag = sqrtf(laneOffX*laneOffX + laneOffZ*laneOffZ) * falloff;
        g_dbgOffsetSum += appliedMag;
        if (appliedMag > g_dbgOffsetMax) g_dbgOffsetMax = appliedMag;
    }

    // Register in neighbor cache
    LONG idx = g_NbrIdx++;
    if (idx < SEP_MAX_UNITS)
        g_NbrCache[idx] = {myX, myZ, unit, mySize};
}

// ==========================================================================
// Hook Installation (same trampoline as v4)
// ==========================================================================
static uint8_t* g_Trampoline = nullptr;

static void InstallSpreadHook() {
    g_Trampoline = (uint8_t*)VirtualAlloc(NULL, 4096,
        MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!g_Trampoline) { Log("[SPR5] VirtualAlloc FAILED\n"); return; }

    uint8_t* t = g_Trampoline;
    int off = 0;
    t[off++] = 0x57; t[off++] = 0x56;
    t[off++] = 0x8B; t[off++] = 0xF8;
    t[off++] = 0x8B; t[off++] = 0xF1;
    t[off++] = 0xE8;
    int32_t r1 = (int32_t)(0x005AD8B0 - (uint32_t)(t + off + 4));
    memcpy(t + off, &r1, 4); off += 4;
    t[off++] = 0x56; t[off++] = 0x57;
    t[off++] = 0xE8;
    int32_t r2 = (int32_t)((uint32_t)&ApplySpread - (uint32_t)(t + off + 4));
    memcpy(t + off, &r2, 4); off += 4;
    t[off++] = 0x83; t[off++] = 0xC4; t[off++] = 0x08;
    t[off++] = 0x8B; t[off++] = 0xC7;
    t[off++] = 0x5E; t[off++] = 0x5F; t[off++] = 0xC3;

    uint8_t* cs = (uint8_t*)0x005A42E2;
    DWORD old;
    if (VirtualProtect(cs, 5, PAGE_EXECUTE_READWRITE, &old)) {
        int32_t nr = (int32_t)((uint32_t)g_Trampoline - (0x005A42E2 + 5));
        memcpy(cs + 1, &nr, 4);
        VirtualProtect(cs, 5, old, &old);
        Log("[SPR5] Hook OK -> 0x%08X\n", (uint32_t)g_Trampoline);
    }
}

static DWORD WINAPI PatchThread(LPVOID) {
    uintptr_t g_Sim = 0;
    int attempts = 0;
    while (!g_Sim && attempts < 120) {
        Sleep(1000);
        g_Sim = *(uintptr_t*)0x010A63F0;
        attempts++;
    }
    if (!g_Sim) { Log("[PF] Sim never ready\n"); return 0; }
    Log("[PF] Sim ready at 0x%08X\n", (uint32_t)g_Sim);

    float* navDist = (float*)0x00E4F840;
    DWORD old;
    if (VirtualProtect(navDist, 4, PAGE_EXECUTE_READWRITE, &old)) {
        g_OrigNavDist = *navDist;
        if (g_StrogoEnabled) {
            *navDist = 9999.0f;
            Log("[PF] Strogo ON (was %.1f)\n", g_OrigNavDist);
        }
        VirtualProtect(navDist, 4, old, &old);
    }

    memset(g_OffsetMap, 0, sizeof(g_OffsetMap));
    InstallSpreadHook();
    InstallAvoidanceTuning();
    Log("[PF] Spread=%s Strogo=%s\n",
        g_SpreadEnabled ? "ON" : "OFF", g_StrogoEnabled ? "ON" : "OFF");
    return 0;
}

static DWORD WINAPI HotkeyThread(LPVOID) {
    bool last1 = false, last2 = false, last3 = false;
    DWORD lastGsLog = 0;
    while (true) {
        Sleep(50);
        bool shift = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
        bool k1 = (GetAsyncKeyState('1') & 0x8000) != 0;
        bool k2 = (GetAsyncKeyState('2') & 0x8000) != 0;
        bool k3 = (GetAsyncKeyState('3') & 0x8000) != 0;
        if (k1 && !last1 && shift) {
            g_SpreadEnabled = !g_SpreadEnabled;
            Log("[HOTKEY] Spread = %s\n", g_SpreadEnabled ? "ON" : "OFF");
        }
        last1 = k1;

        // Shift+3: Toggle all 3 nav debug visualizations
        if (k3 && !last3 && shift) {
            g_ConExecute("dbg navwaypoints");
            g_ConExecute("dbg navpath");
            g_ConExecute("dbg navsteering");
            Log("[HOTKEY] Toggled navwaypoints + navpath + navsteering\n");
        }
        last3 = k3;

        if (k2 && !last2 && shift) {
            g_StrogoEnabled = !g_StrogoEnabled;
            float* navDist = (float*)0x00E4F840;
            DWORD old;
            if (VirtualProtect(navDist, 4, PAGE_EXECUTE_READWRITE, &old)) {
                *navDist = g_StrogoEnabled ? 9999.0f : g_OrigNavDist;
                VirtualProtect(navDist, 4, old, &old);
            }
            Log("[HOTKEY] Strogo = %s (navDist=%.1f)\n",
                g_StrogoEnabled ? "ON" : "OFF",
                g_StrogoEnabled ? 9999.0f : g_OrigNavDist);
        }
        last2 = k2;

        // Shift+9: Toggle Avoidance Tuning
        bool k9 = (GetAsyncKeyState('9') & 0x8000) != 0;
        static bool last9 = false;
        if (k9 && !last9 && shift) AvoidanceTuningToggle();
        last9 = k9;

        // Debug logs every 3s
        DWORD now = GetTickCount();
        if (now - lastGsLog > 3000) {
            AvoidanceTuningDebugLog();
            LaneSystemDebugLog();
            lastGsLog = now;
        }
    }
    return 0;
}

extern "C" void InstallSteeringHook() {
    Log("[PF] Spread v5 (persistent adaptive offset)\n");
    CreateThread(nullptr, 0, PatchThread, nullptr, 0, nullptr);
    CreateThread(nullptr, 0, HotkeyThread, nullptr, 0, nullptr);
}
