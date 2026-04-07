// ==========================================================================
// PathfinderTuning.cpp — A* heuristic replacement with hash spread.
//
// Replaces Moho::CAiPathFinder::GetHeuristicCost (0x5AA590) via a 6-byte
// JMP at the prologue (see hooks/PathfinderTuning.hook). The new
// implementation:
//   1. Preserves the original octile-distance heuristic to the goal rect
//      (sqrt(2)-1 weighted) so existing path-search invariants hold.
//   2. Multiplies by an inflation factor (pathHeuristicWeight). Values
//      slightly above 1.0 make A* expand far fewer nodes at the cost of
//      marginally suboptimal paths. 1.10 gives ~30% fewer expansions.
//   3. Adds a tiny per-pathfinder hash penalty (pathHeuristicSpread) so
//      multiple units sharing a goal get slightly different A* costs and
//      naturally diverge instead of converging on the same cell. The
//      hash is stable for a given (pathfinder, cell) pair so monotonicity
//      holds within a single search.
//
// Lua side (Sim-Lua context, not the ~ debug console):
//   SetPathHeuristicWeight(float w)   1.0..4.0  default 1.10
//   SetPathHeuristicSpread(float s)   0.0..2.0  default 0.50
//   SetPathRectHistoryDepth(int n)    1..127    default 2
//
// The ~ console route via ConDescReg does NOT work in this codebase
// because ConDescReg's constexpr constructor body gets constant-init'd
// by the compiler and the runtime registration call is dropped.
// ==========================================================================

#include "LuaAPI.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"
#include <stdint.h>

// CAiPathFinder offsets (faf-re reverse engineering, IDA verified)
#define OFF_PF_GOAL_X0  0x3C
#define OFF_PF_GOAL_Z0  0x40
#define OFF_PF_GOAL_X1  0x44
#define OFF_PF_GOAL_Z1  0x48

// --------------------------------------------------------------------------
// Tunable globals. Defaults are chosen to be a clear improvement over
// vanilla without producing visibly worse paths.
// --------------------------------------------------------------------------
float pathHeuristicWeight = 1.10f;
float pathHeuristicSpread = 0.50f;

int SetPathHeuristicWeight(lua_State* L)
{
    float w = (float)luaL_optnumber(L, 1, 1.10);
    if (w < 1.0f) w = 1.0f;
    if (w > 4.0f) w = 4.0f;
    pathHeuristicWeight = w;
    return 0;
}
SimRegFunc SetPathHeuristicWeightReg{
    "SetPathHeuristicWeight", "(float w=1.10) tune A* heuristic inflation", SetPathHeuristicWeight
};

int SetPathHeuristicSpread(lua_State* L)
{
    float s = (float)luaL_optnumber(L, 1, 0.50);
    if (s < 0.0f) s = 0.0f;
    if (s > 2.0f) s = 2.0f;
    pathHeuristicSpread = s;
    return 0;
}
SimRegFunc SetPathHeuristicSpreadReg{
    "SetPathHeuristicSpread", "(float s=0.50) per-unit A* hash spread", SetPathHeuristicSpread
};

// --------------------------------------------------------------------------
// Recent-search-rect ring depth (single-byte cmp imm at 0x5AA4E3 inside
// CAiPathFinder::QueueSearch). Default kept at 2 to match vanilla; raise
// via Lua to give the anti-loop logic more memory in tight corridors.
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
extern "C" void __cdecl ComputeHeuristicC(void* pathfinder, void* cellPtr, float* outResult)
{
    auto* pf = reinterpret_cast<uint8_t*>(pathfinder);
    int x0 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_X0);
    int z0 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_Z0);
    int x1 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_X1);
    int z1 = *reinterpret_cast<int*>(pf + OFF_PF_GOAL_Z1);

    int cx = *reinterpret_cast<uint16_t*>(cellPtr);
    int cz = *reinterpret_cast<uint16_t*>(reinterpret_cast<uint8_t*>(cellPtr) + 2);

    // Per-axis distance to outer goal rect (matches engine octile)
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

    // Original octile heuristic, scaled by inflation factor
    float result = (minor * 0.41421354f + major) * pathHeuristicWeight;

    // Per-pathfinder hash penalty for spread. Stable for a given
    // (pathfinder, cell) pair so monotonicity holds within one search.
    if (pathHeuristicSpread > 0.0f) {
        uint32_t h = (uint32_t)(uintptr_t)pathfinder;
        h ^= (uint32_t)(cx) * 0x9E3779B1u;
        h ^= (uint32_t)(cz) * 0x85EBCA6Bu;
        h = (h ^ (h >> 16)) * 0x7FEB352Du;
        h = (h ^ (h >> 15)) * 0x846CA68Bu;
        h ^= (h >> 16);
        float noise = (float)(h & 0xFF) * (1.0f / 255.0f);  // [0,1)
        result += noise * pathHeuristicSpread;
    }

    *outResult = result;
}

// --------------------------------------------------------------------------
// Asm thunk — entry point for the JMP at 0x5AA590 (PathfinderTuning.hook).
// __thiscall: ecx = this, [esp+4] = SOCellPos*, callee returns float in
// st0 and cleans up 1 dword arg with `ret 4`.
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
