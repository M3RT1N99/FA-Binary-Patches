// ==========================================================================
// PathfinderTuning.cpp — Lua-tunable parameters for the A* pathfinder.
//
// Hooks Moho::CAiPathFinder at 0x5AA590 (GetHeuristicCost) with a full
// reimplementation that:
//   1. preserves the original octile-distance heuristic (sqrt(2)-1 weighted)
//   2. multiplies by a Lua-tunable inflation factor (pathHeuristicWeight)
//   3. adds a tiny per-pathfinder hash penalty so multiple units sharing
//      a goal get slightly different A* costs — they diverge naturally
//      instead of all converging on the same cell.
//
// Also exposes:
//   SetPathHeuristicWeight(float)   - 1.0..4.0, default 1.01
//   SetPathHeuristicSpread(float)   - 0.0..2.0, default 0.0  (off)
//   SetPathRectHistoryDepth(int)    - 1..127,   default 2    (anti-loop ring)
// ==========================================================================

#include "LuaAPI.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"
#include <stdint.h>

// CAiPathFinder offsets (from faf-re reverse engineering, verified in IDA)
#define OFF_PF_GOAL_X0  0x3C
#define OFF_PF_GOAL_Z0  0x40
#define OFF_PF_GOAL_X1  0x44
#define OFF_PF_GOAL_Z1  0x48

// --------------------------------------------------------------------------
// Lua-tunable parameters
// --------------------------------------------------------------------------
// HARDCODED FOR HOOK TEST: spread is normally 0.0 (off), set to 1.0 here
// so the hook's effect is unconditional. Once we confirm the hook fires
// (via LogF inside ComputeHeuristicC) we can revert to runtime tunable.
float pathHeuristicWeight = 1.15f;
float pathHeuristicSpread = 1.0f;   // FORCED ON for diagnostic

// DIAGNOSTIC: prove constructors run for this translation unit.
// Uses both __attribute__((constructor)) and a static struct to test
// which mechanism the build system actually executes.
__attribute__((constructor))
static void PathfinderTuning_Init_Attr() {
    LogF("PathfinderTuning: __attribute__((constructor)) RAN");
}

struct PathfinderTuningInitProbe {
    PathfinderTuningInitProbe() {
        LogF("PathfinderTuning: static struct ctor RAN");
    }
};
static PathfinderTuningInitProbe gPathfinderTuningProbe;

int SetPathHeuristicWeight(lua_State* L)
{
    float w = (float)luaL_optnumber(L, 1, 1.01);
    if (w < 1.0f) w = 1.0f;
    if (w > 4.0f) w = 4.0f;
    pathHeuristicWeight = w;
    return 0;
}
SimRegFunc SetPathHeuristicWeightReg{
    "SetPathHeuristicWeight", "(float w=1.01) tune A* heuristic inflation", SetPathHeuristicWeight
};
// ~ console:  path_heuristic_weight 1.15
ConDescReg<float> path_heuristic_weight_var{
    "path_heuristic_weight", "A* heuristic inflation factor (1.0..4.0)", &pathHeuristicWeight
};

int SetPathHeuristicSpread(lua_State* L)
{
    float s = (float)luaL_optnumber(L, 1, 0.0);
    if (s < 0.0f) s = 0.0f;
    if (s > 2.0f) s = 2.0f;
    pathHeuristicSpread = s;
    return 0;
}
SimRegFunc SetPathHeuristicSpreadReg{
    "SetPathHeuristicSpread", "(float s=0.0) per-unit A* hash spread", SetPathHeuristicSpread
};
// ~ console:  path_heuristic_spread 0.5
ConDescReg<float> path_heuristic_spread_var{
    "path_heuristic_spread", "Per-unit A* hash penalty (0.0..2.0)", &pathHeuristicSpread
};

// --------------------------------------------------------------------------
// Recent-search-rect ring depth (single-byte cmp imm at 0x5AA4E3)
// --------------------------------------------------------------------------
static const uintptr_t kRectHistoryCmpImm = 0x005AA4E3;

int SetPathRectHistoryDepth(lua_State* L)
{
    int depth = (int)luaL_optnumber(L, 1, 2);
    if (depth < 1)   depth = 1;
    if (depth > 127) depth = 127;

    auto vp = reinterpret_cast<bool(__stdcall*)(void*, size_t, uint32_t, uint32_t*)>(
        GetProcAddress(GetModuleHandleA("KERNEL32"), "VirtualProtect")
    );
    if (!vp) return 0;

    uint32_t oldProt = 0;
    vp(reinterpret_cast<void*>(kRectHistoryCmpImm), 1, 0x40, &oldProt);
    *reinterpret_cast<uint8_t*>(kRectHistoryCmpImm) = (uint8_t)depth;
    vp(reinterpret_cast<void*>(kRectHistoryCmpImm), 1, oldProt, &oldProt);
    return 0;
}
SimRegFunc SetPathRectHistoryDepthReg{
    "SetPathRectHistoryDepth", "(int n=2) raise A* anti-loop ring depth", SetPathRectHistoryDepth
};

// --------------------------------------------------------------------------
// Heuristic replacement (called from asm thunk below).
//
// pathfinder = `this` of CAiPathFinder (ecx in __thiscall)
// cellPtr    = SOCellPos* (uint16 x, uint16 z) — A* candidate cell
// outResult  = single-precision result, written by callee
// --------------------------------------------------------------------------
// Debug: prove the hook is actually being entered. First call writes one
// LogF line so we can see it in debug.log; afterwards stays silent.
static int gHeuristicHookCalls = 0;

extern "C" void __cdecl ComputeHeuristicC(void* pathfinder, void* cellPtr, float* outResult)
{
    if (gHeuristicHookCalls == 0) {
        LogF("PathfinderTuning: GetHeuristicCost replacement HOOK ACTIVE (pf=%p w=%f s=%f)",
             pathfinder, pathHeuristicWeight, pathHeuristicSpread);
    }
    if ((gHeuristicHookCalls & 0xFFFFF) == 0) {
        // every ~1M calls, dump current state to confirm tuning takes effect
        LogF("PathfinderTuning: %d heuristic calls so far (w=%f s=%f)",
             gHeuristicHookCalls, pathHeuristicWeight, pathHeuristicSpread);
    }
    ++gHeuristicHookCalls;

    auto* pf = reinterpret_cast<uint8_t*>(pathfinder);
    int x0 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_X0);
    int z0 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_Z0);
    int x1 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_X1);
    int z1 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_Z1);

    int cx = *reinterpret_cast<uint16_t*>(cellPtr);
    int cz = *reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(cellPtr) + 2);

    // Same per-axis distance-to-rect as original (octile to outer goal rect)
    int dx = 0;
    if (x0 - cx > 0) dx = x0 - cx;
    int dxAlt = cx - x1 + 1;
    if (dx < dxAlt) dx = dxAlt;

    int dz = 0;
    if (z0 - cz > 0) dz = z0 - cz;
    int dzAlt = cz - z1 + 1;
    if (dz < dzAlt) dz = dzAlt;

    float adx = (float)(dx < 0 ? -dx : dx);
    float adz = (float)(dz < 0 ? -dz : dz);

    float major, minor;
    if (adz <= adx) { major = adx; minor = adz; }
    else            { major = adz; minor = adx; }

    // Original octile heuristic times the tunable inflation factor
    float result = (minor * 0.41421354f + major) * pathHeuristicWeight;

    // Per-pathfinder hash penalty for spread.
    // Stable for a given (pathfinder, cell) pair so A* monotonicity holds
    // within a single search.
    if (pathHeuristicSpread > 0.0f) {
        uint32_t h = (uint32_t)(uintptr_t)pathfinder;
        h ^= (uint32_t)(cx) * 0x9E3779B1u;
        h ^= (uint32_t)(cz) * 0x85EBCA6Bu;
        h = (h ^ (h >> 16)) * 0x7FEB352Du;
        h = (h ^ (h >> 15)) * 0x846CA68Bu;
        h ^= (h >> 16);
        // Map low byte to [0, 1) and scale by spread
        float noise = (float)(h & 0xFF) * (1.0f / 255.0f);
        result += noise * pathHeuristicSpread;
    }

    *outResult = result;
}

// --------------------------------------------------------------------------
// Asm thunk — called from the JMP at 0x5AA590 (PathfinderTuning.hook).
// __thiscall calling convention: ecx = this, [esp+4] = SOCellPos*, ret 4.
// Original returned a single-precision via FPU st0 (fld dword ptr ...).
// We allocate a stack slot, call the C function, then fld + ret 4.
// --------------------------------------------------------------------------
asm(
    ".global _HeuristicReplaceHook\n"
    "_HeuristicReplaceHook:\n"

    "    sub esp, 4\n"               // local: float result
    "    lea eax, [esp]\n"           // &result
    "    push eax\n"                 // arg3: outResult
    "    push dword ptr [esp+12]\n"  // arg2: cell pointer (orig +4 + 8 pushed)
    "    push ecx\n"                 // arg1: this
    "    call _ComputeHeuristicC\n"
    "    add esp, 12\n"              // pop 3 args

    "    fld dword ptr [esp]\n"      // load result into st0
    "    add esp, 4\n"               // free local
    "    ret 4\n"                    // __thiscall callee cleanup of 1 arg
);
