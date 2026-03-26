// ==========================================================================
// avoidance_tuning.cpp - Engine Collision Avoidance Tuning
//
// Patches specific instructions in the existing engine avoidance to point
// at our private float variables. The global constants (0.1, 0.707) are
// NOT modified as they have 65-76+ xrefs.
//
// Tuning knobs:
//   A. Scan radius multiplier (CheckCollisions): 0.1 -> 0.3
//      Units detect collisions earlier
//   B. Cone check threshold (ResolvePossibleCollision): 0.707 -> 0.3
//      Units also dodge lateral neighbors, not just frontal ones
//
// Hotkey: Shift+9 Toggle (original vs tuned values)
// ==========================================================================
#include <windows.h>
#include <cstdint>
#include <cstring>
#include <cmath>

extern "C" void Log(const char* fmt, ...);

// ==========================================================================
// Private tuning values (referenced via instruction patches)
// ==========================================================================
static float g_ScanRadiusMult = 0.3f;       // Original: 0.1
static float g_ScanRadiusOriginal = 0.1f;
static float g_ConeThreshold = 0.30f;        // Original: 0.707 (cos 45deg)
static float g_ConeThresholdOriginal = 0.70700002f;
static float g_CollisionExtent = 0.75f;      // Original: 0.25 (OBB half-extent)
static float g_CollisionExtentOriginal = 0.25f;

static bool g_TuningEnabled = true;

// ==========================================================================
// D. GoalSpread with 10-Lane Interleaved System
// Hook on RequestPathfind (0x5AE00B): shifts SNavGoal rect per unit
// so A* computes different paths. Undo after the original call.
// ==========================================================================
#define LANE_OFFSET        10    // Grid cells per lane step
#define LANES_PER_GROUP    3     // 3 lanes per army = 6 lanes total

static LONG g_gsDbgCalls = 0;
static LONG g_gsDbgOffset = 0;
static LONG g_gsDbgSkipped = 0;

// Undo state
static int g_lastOffX = 0, g_lastOffZ = 0;
static uint32_t g_lastGoalPtr = 0;

// Called BEFORE SetNavGoal: shifts goal rect into assigned lane
// Register context: eax = &mGoal (SNavGoal*), ebx = mPathFinder
static void __cdecl ModifyGoal(uint32_t goalPtr, uint32_t pathFinderPtr) {
    g_gsDbgCalls++;
    if (!g_TuningEnabled) { g_gsDbgSkipped++; return; }

    uint32_t unit = *(uint32_t*)(pathFinderPtr + 0x18);
    if (!unit) { g_gsDbgSkipped++; return; }

    // Start position from PathFinder
    short startX = *(short*)(pathFinderPtr + 0x34);
    short startZ = *(short*)(pathFinderPtr + 0x36);

    // Goal center (SNavGoal: 8x int)
    int* pos = (int*)goalPtr;
    int goalCX = (pos[0] + pos[2]) / 2;
    int goalCZ = (pos[1] + pos[3]) / 2;

    // Travel direction
    float dirX = (float)(goalCX - startX);
    float dirZ = (float)(goalCZ - startZ);
    float dirLen = sqrtf(dirX * dirX + dirZ * dirZ);
    if (dirLen < 2.0f) { g_gsDbgSkipped++; return; }

    // Perpendicular - CANONICAL (always in +X half-plane)
    float perpX = -dirZ / dirLen;
    float perpZ =  dirX / dirLen;
    if (perpX < 0.0f || (perpX == 0.0f && perpZ < 0.0f)) {
        perpX = -perpX;
        perpZ = -perpZ;
    }

    // 6-Lane Interleaved: clean pass-through
    // Group 0 (dirX+dirZ>0) -> right side: Lanes +1, +2, +3
    // Group 1 (dirX+dirZ<0) -> left side:  Lanes -1, -2, -3
    // Result: | G1:-3 | G1:-2 | G1:-1 | G0:+1 | G0:+2 | G0:+3 |
    // Opposing traffic drives on separate sides past each other
    int groupId = (dirX + dirZ > 0.0f) ? 0 : 1;
    uint32_t hash = (unit >> 4) * 2654435761u;
    int subLane = (int)(hash % LANES_PER_GROUP) + 1;  // 1, 2, 3
    float laneOffset = (groupId == 0) ? (float)subLane : -(float)subLane;

    int offX = (int)(perpX * laneOffset * (float)LANE_OFFSET);
    int offZ = (int)(perpZ * laneOffset * (float)LANE_OFFSET);

    if (offX == 0 && offZ == 0) { g_gsDbgSkipped++; return; }

    // Shift goal rect (will be undone after original call)
    pos[0] += offX; pos[1] += offZ;
    pos[2] += offX; pos[3] += offZ;
    pos[4] += offX; pos[5] += offZ;
    pos[6] += offX; pos[7] += offZ;

    g_lastOffX = offX; g_lastOffZ = offZ; g_lastGoalPtr = goalPtr;
    g_gsDbgOffset++;

    if (g_gsDbgOffset <= 5) {
        Log("[LANE] unit=0x%08X grp=%d sub=%d off=(%d,%d)\n",
            unit, groupId, subLane, offX, offZ);
    }
}

// Called AFTER SetNavGoal: restores goal rect
static void __cdecl UndoGoalOffset() {
    if (g_lastGoalPtr && (g_lastOffX || g_lastOffZ)) {
        int* pos = (int*)g_lastGoalPtr;
        pos[0] -= g_lastOffX; pos[1] -= g_lastOffZ;
        pos[2] -= g_lastOffX; pos[3] -= g_lastOffZ;
        pos[4] -= g_lastOffX; pos[5] -= g_lastOffZ;
        pos[6] -= g_lastOffX; pos[7] -= g_lastOffZ;
    }
    g_lastGoalPtr = 0; g_lastOffX = 0; g_lastOffZ = 0;
}

static uint8_t* g_GoalTrampoline = nullptr;

// ==========================================================================
// Instruction Patching Helper
// Changes the 4-byte address within an SSE instruction
// ==========================================================================
static bool PatchInstrAddr(uint8_t* instrAddr, int addrOffset,
                           const uint8_t* expectedPrefix, int prefixLen,
                           uint32_t expectedOrigAddr, uint32_t newAddr,
                           const char* name) {
    // Verify prefix bytes
    if (memcmp(instrAddr, expectedPrefix, prefixLen) != 0) {
        Log("[TUNE] FAILED %s: prefix mismatch at 0x%08X\n", name, (uint32_t)instrAddr);
        Log("[TUNE]   expected: ");
        for (int i = 0; i < prefixLen; i++) Log("%02X ", expectedPrefix[i]);
        Log("\n[TUNE]   got:      ");
        for (int i = 0; i < prefixLen; i++) Log("%02X ", instrAddr[i]);
        Log("\n");
        return false;
    }

    // Verify original address
    uint32_t currentAddr = *(uint32_t*)(instrAddr + addrOffset);
    if (currentAddr != expectedOrigAddr) {
        Log("[TUNE] FAILED %s: addr mismatch at 0x%08X+%d (expected 0x%08X, got 0x%08X)\n",
            name, (uint32_t)instrAddr, addrOffset, expectedOrigAddr, currentAddr);
        return false;
    }

    // Patch
    DWORD old;
    if (!VirtualProtect(instrAddr + addrOffset, 4, PAGE_EXECUTE_READWRITE, &old)) {
        Log("[TUNE] FAILED %s: VirtualProtect\n", name);
        return false;
    }
    *(uint32_t*)(instrAddr + addrOffset) = newAddr;
    VirtualProtect(instrAddr + addrOffset, 4, old, &old);

    Log("[TUNE] OK %s: 0x%08X -> 0x%08X (at 0x%08X+%d)\n",
        name, expectedOrigAddr, newAddr, (uint32_t)instrAddr, addrOffset);
    return true;
}

// ==========================================================================
// Installation
// ==========================================================================
void InstallAvoidanceTuning() {
    Log("[TUNE] Installing avoidance tuning patches...\n");

    // --- A. Scan radius multiplier ---
    // CheckCollisions+0x16C (0x5D38AC): mulss xmm0, dword ptr ds:[0xE4F714]
    // Bytes: F3 0F 59 05 [14 F7 E4 00]
    // Address operand at offset +4
    {
        uint8_t prefix[] = {0xF3, 0x0F, 0x59, 0x05};
        PatchInstrAddr((uint8_t*)0x5D38AC, 4, prefix, 4,
                       0x00E4F714, (uint32_t)&g_ScanRadiusMult,
                       "ScanRadius");
    }

    // --- B. Cone check threshold (3 locations) ---
    // ResolvePossibleCollision: comiss xmm3, dword ptr ds:[0xE4F6E0]
    // Bytes: 0F 2F 1D [E0 F6 E4 00]
    // Address operand at offset +3
    {
        uint8_t prefix[] = {0x0F, 0x2F, 0x1D};
        PatchInstrAddr((uint8_t*)0x597003, 3, prefix, 3,
                       0x00E4F6E0, (uint32_t)&g_ConeThreshold,
                       "Cone1");
        PatchInstrAddr((uint8_t*)0x597586, 3, prefix, 3,
                       0x00E4F6E0, (uint32_t)&g_ConeThreshold,
                       "Cone2");
        PatchInstrAddr((uint8_t*)0x5975F9, 3, prefix, 3,
                       0x00E4F6E0, (uint32_t)&g_ConeThreshold,
                       "Cone3");
    }

    // --- C. Collision extent (2 locations in func_UnitsWillCollide) ---
    // OBB half-extent: (SizeX+SizeZ) * 0.25 -> * 0.75
    // mulss xmm0, dword ptr ds:[0xE4F6EC]
    // Bytes: F3 0F 59 05 [EC F6 E4 00]
    // Address operand at offset +4
    {
        uint8_t prefix[] = {0xF3, 0x0F, 0x59, 0x05};
        uint32_t origAddr = 0x00E4F6EC;

        PatchInstrAddr((uint8_t*)0x596C62, 4, prefix, 4,
                       origAddr, (uint32_t)&g_CollisionExtent,
                       "Extent1");
        PatchInstrAddr((uint8_t*)0x596D82, 4, prefix, 4,
                       origAddr, (uint32_t)&g_CollisionExtent,
                       "Extent2");
    }

    // --- D. GoalSpread hook at RequestPathfind (0x5AE00B) ---
    // Redirect CALL sub_5AA120 (SetNavGoal) to our trampoline
    // Trampoline: ModifyGoal -> Original SetNavGoal -> UndoGoalOffset
    {
        g_GoalTrampoline = (uint8_t*)VirtualAlloc(NULL, 4096,
            MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
        if (!g_GoalTrampoline) {
            Log("[TUNE] FAILED GoalSpread: VirtualAlloc\n");
        } else {
            uint8_t* t = g_GoalTrampoline;
            int off = 0;

            // Context at 0x5AE00B: eax=&mGoal, ebx=mPathFinder
            // pushad
            t[off++] = 0x60;
            // push ebx (arg2: pathFinder)
            t[off++] = 0x53;
            // push eax (arg1: goalPtr)
            t[off++] = 0x50;
            // call ModifyGoal
            t[off++] = 0xE8;
            int32_t relMod = (int32_t)((uint32_t)&ModifyGoal - (uint32_t)(t + off + 4));
            memcpy(t + off, &relMod, 4); off += 4;
            // add esp, 8
            t[off++] = 0x83; t[off++] = 0xC4; t[off++] = 0x08;
            // popad
            t[off++] = 0x61;

            // call original sub_5AA120 (SetNavGoal)
            t[off++] = 0xE8;
            int32_t relOrig = (int32_t)(0x005AA120 - (uint32_t)(t + off + 4));
            memcpy(t + off, &relOrig, 4); off += 4;

            // save return value
            t[off++] = 0x50; // push eax
            // call UndoGoalOffset
            t[off++] = 0xE8;
            int32_t relUndo = (int32_t)((uint32_t)&UndoGoalOffset - (uint32_t)(t + off + 4));
            memcpy(t + off, &relUndo, 4); off += 4;
            // restore return value
            t[off++] = 0x58; // pop eax
            // ret
            t[off++] = 0xC3;

            // Patch CALL at 0x5AE00B
            uint8_t* callSite = (uint8_t*)0x005AE00B;
            DWORD old;
            if (VirtualProtect(callSite, 5, PAGE_EXECUTE_READWRITE, &old)) {
                if (callSite[0] == 0xE8) {
                    int32_t newRel = (int32_t)((uint32_t)g_GoalTrampoline - (0x005AE00B + 5));
                    memcpy(callSite + 1, &newRel, 4);
                    VirtualProtect(callSite, 5, old, &old);
                    Log("[TUNE] OK GoalSpread: 0x5AE00B -> 0x%08X\n", (uint32_t)g_GoalTrampoline);
                } else {
                    VirtualProtect(callSite, 5, old, &old);
                    Log("[TUNE] FAILED GoalSpread: expected E8, got %02X\n", callSite[0]);
                }
            }
        }
    }

    Log("[TUNE] ScanRadius=%.2f Cone=%.2f Extent=%.2f LaneOffset=%d\n",
        g_ScanRadiusMult, g_ConeThreshold, g_CollisionExtent, LANE_OFFSET);
}

// ==========================================================================
// Toggle (Shift+9): Switches between original and tuned values
// ==========================================================================
void AvoidanceTuningToggle() {
    g_TuningEnabled = !g_TuningEnabled;
    if (g_TuningEnabled) {
        g_ScanRadiusMult = 0.3f;
        g_ConeThreshold = 0.30f;
        g_CollisionExtent = 0.75f;
    } else {
        g_ScanRadiusMult = g_ScanRadiusOriginal;
        g_ConeThreshold = g_ConeThresholdOriginal;
        g_CollisionExtent = g_CollisionExtentOriginal;
    }
    Log("[HOTKEY] AvoidanceTuning = %s (scan=%.2f cone=%.2f extent=%.2f)\n",
        g_TuningEnabled ? "TUNED" : "ORIGINAL",
        g_ScanRadiusMult, g_ConeThreshold, g_CollisionExtent);
}

// Debug log
void AvoidanceTuningDebugLog() {
    Log("[TUNE] scan=%.2f cone=%.2f extent=%.2f lanes=%d/%d %s\n",
        g_ScanRadiusMult, g_ConeThreshold, g_CollisionExtent,
        g_gsDbgOffset, g_gsDbgCalls, g_TuningEnabled ? "TUNED" : "ORIGINAL");
    g_gsDbgCalls = 0; g_gsDbgOffset = 0; g_gsDbgSkipped = 0;
}
