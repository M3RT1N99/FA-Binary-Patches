#include <windows.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" void Log(const char* fmt, ...);

// ==========================================================================
// Separation Force System
// ==========================================================================
#define SEP_MAX_UNITS    512
#define SEP_MIN_DIST     5.0f   // Distance at which repulsion begins
#define SEP_PUSH_STRENGTH 0.8f  // Repulsion strength (reduced to prevent oscillation)
#define SEP_MAX_NUDGE    1.0f   // Maximum displacement per tick
#define SEP_SMOOTHING    0.3f   // Blend factor: 0=old only, 1=new only (lower = smoother)

struct CachedUnit {
    float x, z;
    uint32_t addr;
    float lastPushX, lastPushZ; // Previous push for smoothing
};

static CachedUnit g_UnitCache[SEP_MAX_UNITS];
static volatile LONG g_CacheIdx = 0;
static DWORD g_LastResetTime = 0;

// Called after GetTargetPos - displaces target away from neighbors
static void __cdecl ApplySeparation(float* targetPos, uint32_t pathNav) {
    // Reset cache every ~100ms (approx. 1 sim tick)
    DWORD now = GetTickCount();
    if (now - g_LastResetTime > 100) {
        g_CacheIdx = 0;
        g_LastResetTime = now;
    }

    // Get unit via PathNavigator chain
    uint32_t pathFinder = *(uint32_t*)(pathNav + 0x10);
    if (!pathFinder) return;
    uint32_t unit = *(uint32_t*)(pathFinder + 0x18);
    if (!unit) return;

    // Own position
    float myX = *(float*)(unit + 0x160);
    float myZ = *(float*)(unit + 0x168);

    // Calculate repulsion from cache
    LONG count = g_CacheIdx;
    if (count > SEP_MAX_UNITS) count = SEP_MAX_UNITS;

    float pushX = 0.0f, pushZ = 0.0f;
    float prevPushX = 0.0f, prevPushZ = 0.0f;
    LONG myIdx = -1;

    // Find previous push (if unit was already in cache)
    for (LONG i = 0; i < count; i++) {
        if (g_UnitCache[i].addr == unit) {
            prevPushX = g_UnitCache[i].lastPushX;
            prevPushZ = g_UnitCache[i].lastPushZ;
            myIdx = i;
            continue;
        }
        float dx = myX - g_UnitCache[i].x;
        float dz = myZ - g_UnitCache[i].z;
        float distSq = dx * dx + dz * dz;

        if (distSq < SEP_MIN_DIST * SEP_MIN_DIST) {
            if (distSq < 0.01f) {
                // Nearly identical position: deterministic side selection
                // Unit with lower address goes +X, higher goes -X
                float side = (unit < g_UnitCache[i].addr) ? 1.0f : -1.0f;
                pushX += side * SEP_PUSH_STRENGTH;
            } else {
                float dist = sqrtf(distSq);
                float force = SEP_PUSH_STRENGTH * (1.0f - dist / SEP_MIN_DIST);
                pushX += (dx / dist) * force;
                pushZ += (dz / dist) * force;
            }
        }
    }

    // Smoothing: blend new push with previous (prevents direction flipping)
    pushX = prevPushX + SEP_SMOOTHING * (pushX - prevPushX);
    pushZ = prevPushZ + SEP_SMOOTHING * (pushZ - prevPushZ);

    // Register/update own position in cache
    LONG idx = (myIdx >= 0) ? myIdx : InterlockedIncrement(&g_CacheIdx) - 1;
    if (idx >= 0 && idx < SEP_MAX_UNITS) {
        g_UnitCache[idx].x = myX;
        g_UnitCache[idx].z = myZ;
        g_UnitCache[idx].addr = unit;
        g_UnitCache[idx].lastPushX = pushX;
        g_UnitCache[idx].lastPushZ = pushZ;
    }

    // Clamp nudge magnitude
    float nudgeSq = pushX * pushX + pushZ * pushZ;
    if (nudgeSq > SEP_MAX_NUDGE * SEP_MAX_NUDGE) {
        float scale = SEP_MAX_NUDGE / sqrtf(nudgeSq);
        pushX *= scale;
        pushZ *= scale;
    }

    // Apply to target position (x = [0], z = [2])
    targetPos[0] += pushX;
    targetPos[2] += pushZ;
}

static uint8_t* g_SepTrampoline = nullptr;

static void InstallSeparationHook() {
    // Trampoline: wrapper that calls GetTargetPos, then ApplySeparation
    g_SepTrampoline = (uint8_t*)VirtualAlloc(NULL, 4096,
        MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE);
    if (!g_SepTrampoline) {
        Log("[PF-SEP] VirtualAlloc FAILED\n");
        return;
    }

    uint8_t* t = g_SepTrampoline;
    int off = 0;

    // push edi; push esi
    t[off++] = 0x57;
    t[off++] = 0x56;
    // mov edi, eax (save outbuf)
    t[off++] = 0x8B; t[off++] = 0xF8;
    // mov esi, ecx (save pathNavigator)
    t[off++] = 0x8B; t[off++] = 0xF1;

    // call GetTargetPos (0x5AD8B0)
    t[off++] = 0xE8;
    int32_t relGetTarget = (int32_t)(0x005AD8B0 - (uint32_t)(t + off + 4));
    memcpy(t + off, &relGetTarget, 4); off += 4;

    // push esi (arg2: pathNavigator)
    t[off++] = 0x56;
    // push edi (arg1: outbuf / targetPos)
    t[off++] = 0x57;

    // call ApplySeparation
    t[off++] = 0xE8;
    int32_t relSep = (int32_t)((uint32_t)&ApplySeparation - (uint32_t)(t + off + 4));
    memcpy(t + off, &relSep, 4); off += 4;

    // add esp, 8
    t[off++] = 0x83; t[off++] = 0xC4; t[off++] = 0x08;
    // mov eax, edi (return outbuf)
    t[off++] = 0x8B; t[off++] = 0xC7;
    // pop esi; pop edi; ret
    t[off++] = 0x5E;
    t[off++] = 0x5F;
    t[off++] = 0xC3;

    // Redirect CALL in TaskTick (0x5A42E2): E8 [rel32]
    uint8_t* callSite = (uint8_t*)0x005A42E2;
    DWORD old;
    if (VirtualProtect(callSite, 5, PAGE_EXECUTE_READWRITE, &old)) {
        int32_t newRel = (int32_t)((uint32_t)g_SepTrampoline - (0x005A42E2 + 5));
        memcpy(callSite + 1, &newRel, 4);
        VirtualProtect(callSite, 5, old, &old);
        Log("[PF-SEP] Hook installed at 0x5A42E2 -> trampoline 0x%08X\n",
            (uint32_t)g_SepTrampoline);
    } else {
        Log("[PF-SEP] VirtualProtect FAILED\n");
    }
}



static DWORD WINAPI PatchThread(LPVOID) {
    // Wait for sim to be ready
    uintptr_t g_Sim = 0;
    int attempts = 0;
    while (!g_Sim && attempts < 120) {
        Sleep(1000);
        g_Sim = *(uintptr_t*)0x010A63F0;
        attempts++;
    }
    if (!g_Sim) {
        Log("[PF] Sim never ready - Aborting patches\n");
        return 0;
    }
    Log("[PF] Sim ready at 0x%08X after %d attempts\n", (uint32_t)g_Sim, attempts);

#if 0 // === All byte patches disabled - only Separation Force active ===
    const char* s1 = "FAIL";
    const char* s2 = "FAIL";
    const char* s3 = "FAIL";
    const char* s4 = "FAIL";
    const char* s5 = "FAIL";
    const char* s6 = "FAIL";

    uint8_t* p1 = (uint8_t*)0x00597C83;
    if (p1[0] == 0x83 && p1[1] == 0xC3 && p1[2] == 0x03) {
        DWORD old;
        if (VirtualProtect(p1, 3, PAGE_EXECUTE_READWRITE, &old)) {
            p1[2] = 0x01;
            VirtualProtect(p1, 3, old, &old);
            s1 = "OK";
            Log("[PF] Patch1 Granularity OK\n");
        }
    } else {
        Log("[PF] Patch1 FAILED: %02X %02X %02X\n", p1[0], p1[1], p1[2]);
    }
    // -------------------------------------------------------------------------
    // Patch 2: Remove PT_2 skip in CheckCollisions (0x005D39A9)
    // Original: 74 4C (jz +0x4C) - skips PT_2 units entirely
    //   -> Groups "don't see" each other, drive through each other
    // Old fix (jz->jnz) was wrong: skipped non-PT_2 instead
    // New fix: NOP (90 90) - ALL units get collision checked
    //   Army/Priority logic downstream filters correctly
    // -------------------------------------------------------------------------
    uint8_t* p2 = (uint8_t*)0x005D39A9;
    if (p2[0] == 0x74 && p2[1] == 0x4C) {
        DWORD old;
        if (VirtualProtect(p2, 2, PAGE_EXECUTE_READWRITE, &old)) {
            p2[0] = 0x90; // nop
            p2[1] = 0x90; // nop
            VirtualProtect(p2, 2, old, &old);
            s2 = "OK";
            Log("[PF] Patch2 RemovePT2Skip OK\n");
        }
    } else {
        Log("[PF] Patch2 FAILED: %02X %02X\n", p2[0], p2[1]);
    }

    // -------------------------------------------------------------------------
    // Patch 3: Waypoint Radius (0x00E4F840: 50.0f -> 80.0f)
    // Smoother curves, less abrupt direction changes at waypoints
    // -------------------------------------------------------------------------
    /*
    float* p3 = (float*)0x00E4F840;
    if (*p3 == 50.0f) {
        DWORD old;
        if (VirtualProtect(p3, 4, PAGE_EXECUTE_READWRITE, &old)) {
            *p3 = 80.0f;
            VirtualProtect(p3, 4, old, &old);
            s3 = "OK";
            Log("[PF] Patch3 WaypointRadius OK\n");
        }
    } else {
        Log("[PF] Patch3 FAILED: %.1f\n", *p3);
    }
    */

    // -------------------------------------------------------------------------
    // Patch 4: Lookahead Multiplier
    // (0x005D38AC: mulss xmm0,[0xE4F714] -> mulss xmm0,[0xE4F9A0])
    // SearchRadius = SplineNodes * MaxSpeed * 0.3 instead of 0.1 -> 3x earlier detection
    // 0xE4F714 = "Seconds Per Tick" (0.1f) - do NOT patch the original!
    // 0xE4F9A0 = 0.3f steering constant (from ResolvePossibleCollision)
    // Original: F3 0F 59 05 14 F7 E4 00
    // Patch:    F3 0F 59 05 A0 F9 E4 00
    // -------------------------------------------------------------------------

    uint8_t* p4 = (uint8_t*)0x005D38AC;
    if (p4[0] == 0xF3 && p4[1] == 0x0F && p4[2] == 0x59 && p4[3] == 0x05 &&
        p4[4] == 0x14 && p4[5] == 0xF7 && p4[6] == 0xE4 && p4[7] == 0x00) {
        DWORD old;
        if (VirtualProtect(p4, 8, PAGE_EXECUTE_READWRITE, &old)) {
            p4[4] = 0xA0;
            p4[5] = 0xF9;
            VirtualProtect(p4, 8, old, &old);
            s4 = "OK";
            Log("[PF] Patch4 LookaheadMultiplier OK\n");
        }
    } else {
        Log("[PF] Patch4 FAILED: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            p4[0], p4[1], p4[2], p4[3], p4[4], p4[5], p4[6], p4[7]);
    }

    // -------------------------------------------------------------------------
    // Patch 5: Disable moving-check in IsHigherPriorityThan (0x006A8D80)
    //
    // Problem: "Standstill has priority" causes deadlocks.
    // When a unit briefly stops (because it's deliberating), it becomes an
    // "immovable obstacle" with highest priority. The other group also
    // stops -> both are standing -> deadlock.
    //
    // Patch 5.1 (0x006A8F1E): jz loc_6A9002 (0F 84 DE 00 00 00) -> 6x NOP
    //   Prevents: "I'm moving + He's stopped -> I yield (return 0)"
    //
    // Patch 5.2 (0x006A8F3B): jnz loc_6A8E14 (75 D7) -> 2x NOP
    //   Prevents: "I'm stopped + He's moving -> I have priority (return 1)"
    //
    // Result: Temporary standstill no longer grants priority.
    // Tiebreaker falls to footprint size or Unit ID (deterministic,
    // multiplayer-safe since Unit IDs are identical on all clients).
    // -------------------------------------------------------------------------

    // Patch 5.1: Disable yield branch (6 bytes)

    /* not very effective, therefore commented out
    uint8_t* p5a = (uint8_t*)0x006A8F1E;
    bool p5a_ok = false;
    if (p5a[0] == 0x0F && p5a[1] == 0x84 && p5a[2] == 0xDE &&
        p5a[3] == 0x00 && p5a[4] == 0x00 && p5a[5] == 0x00) {
        DWORD old;
        if (VirtualProtect(p5a, 6, PAGE_EXECUTE_READWRITE, &old)) {
            p5a[0] = 0x90;
            p5a[1] = 0x90;
            p5a[2] = 0x90;
            p5a[3] = 0x90;
            p5a[4] = 0x90;
            p5a[5] = 0x90;
            VirtualProtect(p5a, 6, old, &old);
            p5a_ok = true;
            Log("[PF] Patch5.1 MovingYield NOP OK\n");
        }
    } else {
        Log("[PF] Patch5.1 FAILED: %02X %02X %02X %02X %02X %02X\n",
            p5a[0], p5a[1], p5a[2], p5a[3], p5a[4], p5a[5]);
    }

    // Patch 5.2 (0x006A8F40): jnz loc_6A8E14 -> 6x NOP
    // Corrected address (was 0x006A8F3B - in the middle of mov ecx,edi)
    uint8_t* p5b = (uint8_t*)0x006A8F40;
    bool p5b_ok = false;
    if (p5b[0] == 0x0F && p5b[1] == 0x85 && p5b[2] == 0xCE &&
        p5b[3] == 0xFE && p5b[4] == 0xFF && p5b[5] == 0xFF) {
        DWORD old;
        if (VirtualProtect(p5b, 6, PAGE_EXECUTE_READWRITE, &old)) {
            p5b[0] = 0x90; p5b[1] = 0x90; p5b[2] = 0x90;
            p5b[3] = 0x90; p5b[4] = 0x90; p5b[5] = 0x90;
            VirtualProtect(p5b, 6, old, &old);
            p5b_ok = true;
            Log("[PF] Patch5.2 MovingPriority NOP OK\n");
        }
    } else {
        Log("[PF] Patch5.2 FAILED: %02X %02X %02X %02X %02X %02X\n",
            p5b[0], p5b[1], p5b[2], p5b[3], p5b[4], p5b[5]);
    }

    if (p5a_ok && p5b_ok) s5 = "OK";

    else if (p5a_ok || p5b_ok) s5 = "PARTIAL";
    */

    // -------------------------------------------------------------------------
    // Patch 6: Shorten stuck timer (0x005AE655: 64h -> 14h)
    // Units detect blockages after 2s instead of 10s -> faster replan
    // Correction: 3-byte version (83 C0 64) instead of 5-byte
    // -------------------------------------------------------------------------

    uint8_t* p6 = (uint8_t*)0x005AE655;
    if (p6[0] == 0x83 && p6[1] == 0xC0 && p6[2] == 0x64) {
        DWORD old;
        if (VirtualProtect(p6, 3, PAGE_EXECUTE_READWRITE, &old)) {
            p6[2] = 0x14; // 64h (100) -> 14h (20)
            VirtualProtect(p6, 3, old, &old);
            s6 = "OK";
            Log("[PF] Patch6 StuckTimer OK\n");
        }
    } else {
        Log("[PF] Patch6 FAILED: %02X %02X %02X\n", p6[0], p6[1], p6[2]);
    }

    // -------------------------------------------------------------------------
    // Patch 7: Force avoidance in ResolvePossibleCollision (0x00597798)
    // Original: 0F 84 19 F9 FF FF (jz loc_5970B7 = skip avoidance)
    // Problem: TryBuildStructureAt checks if the avoidance point is "free".
    //   With 50+ units it's almost always occupied -> unit does NOTHING.
    //   Result: Groups drive straight through each other.
    // Fix: NOP the jump -> avoidance is ALWAYS attempted (SetCol COLTYPE_2).
    //   The pathfinding corrects invalid points on the next tick.
    // -------------------------------------------------------------------------
    const char* s7 = "FAIL";
    uint8_t* p7 = (uint8_t*)0x00597798;
    if (p7[0] == 0x0F && p7[1] == 0x84 && p7[2] == 0x19 &&
        p7[3] == 0xF9 && p7[4] == 0xFF && p7[5] == 0xFF) {
        DWORD old;
        if (VirtualProtect(p7, 6, PAGE_EXECUTE_READWRITE, &old)) {
            p7[0] = 0x90; p7[1] = 0x90; p7[2] = 0x90;
            p7[3] = 0x90; p7[4] = 0x90; p7[5] = 0x90;
            VirtualProtect(p7, 6, old, &old);
            s7 = "OK";
            Log("[PF] Patch7 ForceAvoidance OK\n");
        }
    } else {
        Log("[PF] Patch7 FAILED: %02X %02X %02X %02X %02X %02X\n",
            p7[0], p7[1], p7[2], p7[3], p7[4], p7[5]);
    }

    // -------------------------------------------------------------------------
    // Patch 8: Increase avoidance offset (0x0059728C)
    // Original: addss xmm0, [0xE4F724] (0.5f)
    //   -> Avoidance point only SizeX1+SizeX2+0.5 away (~2.5 for T1)
    //   -> Not enough for large groups
    // Fix: Redirect address to own float (g_AvoidanceOffset = 5.0f)
    //   -> Avoidance point now ~7.0 away for T1 (SizeX1+SizeX2+5.0)
    //   Only THIS one instruction affected, all other 0.5f users untouched
    // -------------------------------------------------------------------------
    const char* s8 = "FAIL";
    uint8_t* p8 = (uint8_t*)0x0059728C;
    if (p8[0] == 0xF3 && p8[1] == 0x0F && p8[2] == 0x58 && p8[3] == 0x05 &&
        p8[4] == 0x24 && p8[5] == 0xF7 && p8[6] == 0xE4 && p8[7] == 0x00) {
        DWORD old;
        if (VirtualProtect(p8, 8, PAGE_EXECUTE_READWRITE, &old)) {
            uint32_t newAddr = (uint32_t)&g_AvoidanceOffset;
            memcpy(p8 + 4, &newAddr, 4); // Redirect address to 5.0f
            VirtualProtect(p8, 8, old, &old);
            s8 = "OK";
            Log("[PF] Patch8 AvoidanceOffset OK (%.1f at 0x%08X)\n",
                g_AvoidanceOffset, (uint32_t)&g_AvoidanceOffset);
        }
    } else {
        Log("[PF] Patch8 FAILED: %02X %02X %02X %02X %02X %02X %02X %02X\n",
            p8[0], p8[1], p8[2], p8[3], p8[4], p8[5], p8[6], p8[7]);
    }

    // -------------------------------------------------------------------------
    // Patch 9: Remove stopped-unit bypass (0x005971EF)
    // Original: 0F 86 A4 FE FF FF (jbe loc_597099 = LABEL_12)
    // Problem: When the neighbor has velocity=0 (standing still), the
    //   collision is DELETED (COLTYPE_0). This means: stopped units
    //   are INVISIBLE to the avoidance system.
    //   On head-on: front units stop -> rear units ignore them
    //   -> push through.
    // Fix: NOP the jump -> avoidance also against standing units.
    // -------------------------------------------------------------------------
    const char* s9 = "FAIL";
    uint8_t* p9 = (uint8_t*)0x005971EF;
    if (p9[0] == 0x0F && p9[1] == 0x86 && p9[2] == 0xA4 &&
        p9[3] == 0xFE && p9[4] == 0xFF && p9[5] == 0xFF) {
        DWORD old;
        if (VirtualProtect(p9, 6, PAGE_EXECUTE_READWRITE, &old)) {
            p9[0] = 0x90; p9[1] = 0x90; p9[2] = 0x90;
            p9[3] = 0x90; p9[4] = 0x90; p9[5] = 0x90;
            VirtualProtect(p9, 6, old, &old);
            s9 = "OK";
            Log("[PF] Patch9 StoppedUnitBypass OK\n");
        }
    } else {
        Log("[PF] Patch9 FAILED: %02X %02X %02X %02X %02X %02X\n",
            p9[0], p9[1], p9[2], p9[3], p9[4], p9[5]);
    }

    Log("[PF] Done (all byte-patches disabled)\n");
#endif // === End of disabled block ===

    // Only Separation Force Hook active
    InstallSeparationHook();

    return 0;
}

extern "C" void InstallSteeringHook() {
    Log("[PF] Initializing Pathfinding Tweaks...\n");
    CreateThread(nullptr, 0, PatchThread, nullptr, 0, nullptr);
}
