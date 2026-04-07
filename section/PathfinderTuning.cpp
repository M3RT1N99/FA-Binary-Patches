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
#include "PathfinderOccupancy.h"
#include <stdint.h>

// CAiPathFinder offsets (faf-re reverse engineering, IDA verified)
#define OFF_PF_GOAL_X0  0x3C
#define OFF_PF_GOAL_Z0  0x40
#define OFF_PF_GOAL_X1  0x44
#define OFF_PF_GOAL_Z1  0x48
#define OFF_PF_SIM      0x28    // CAiPathFinder + 0x28 → Sim*

// Sim offsets (verified from existing patches and IDA)
#define OFF_SIM_CURTICK 0x900   // Sim + 0x900 → int mCurTick

// --------------------------------------------------------------------------
// Tunable globals. Defaults are chosen to be a clear improvement over
// vanilla without producing visibly worse paths.
// --------------------------------------------------------------------------
float pathHeuristicWeight   = 1.10f;
float pathHeuristicSpread   = 0.50f;
// Cost added per OTHER moving unit in candidate cell's bucket. Two
// safety features keep the penalty well-behaved:
//
//   1. Cap effective count at OCC_COUNT_CAP=8: dense clusters (max observed
//      30+ in one bucket) would otherwise produce explosive costs.
//
//   2. Distance-fade in ComputeHeuristicC: penalty fades to zero in the
//      last 10 cells before the goal so engineers can enter crowded build
//      sites without being deflected.
//
// At 0.50 with cap=8 and fade over 10 cells:
//   wall of 8 at goal_dist=20, 5 cells thick = 5 * 7 * 0.5 * 1.0  = 17.5
//   vs 20-cell base + 10-cell detour                              = 30
//   → direct still wins by 12.5 (penalty visible but not blocking)
//
//   wall of 8 at goal_dist=15, 5 cells thick = 5 * 7 * 0.5 * 1.0  = 17.5
//   vs 15-cell base + 30-cell detour                              = 45
//   → direct wins by 27.5 (long walls still take direct unless very dense)
//
//   engineer crowd at goal_dist=5 (fade=0.5)  = 5 * 7 * 0.5 * 0.5 = 8.75
//   plus base 5 = 13.75  vs 5-cell detour = 10 → detour barely wins
//
//   engineer crowd at goal_dist=2 (fade=0.2)  = 2 * 7 * 0.5 * 0.2 = 1.4
//   → engineers easily enter the destination cluster
//
// Tunable via SetPathOccupancyPenalty(p) at runtime.
float pathOccupancyPenalty  = 0.50f;
#define OCC_COUNT_CAP   8u

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

int SetPathOccupancyPenalty(lua_State* L)
{
    float p = (float)luaL_optnumber(L, 1, 0.75);
    if (p < 0.0f) p = 0.0f;
    if (p > 4.0f) p = 4.0f;
    pathOccupancyPenalty = p;
    return 0;
}
SimRegFunc SetPathOccupancyPenaltyReg{
    "SetPathOccupancyPenalty", "(float p=0.75) cost per moving unit in cell bucket", SetPathOccupancyPenalty
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
// Auto-tuning telemetry (TEMPORARY — removed in cleanup commit).
//
// Each ComputeHeuristicC call optionally records:
//   - sum of base heuristic
//   - sum of occupancy penalty added
//   - histogram of bucket counts encountered
//   - hits/calls ratio
//
// Periodically we log the aggregates AND a recommended pathOccupancyPenalty
// value computed as:
//   ideal = current * (TARGET_RATIO / observed_ratio)
// where TARGET_RATIO is the desired penalty/base ratio (10–15%).
// --------------------------------------------------------------------------
struct HeurStats {
    uint64_t calls;
    uint64_t baseSum_x1000;       // base * 1000 to keep precision in int
    uint64_t penaltySum_x1000;    // penalty * 1000
    uint64_t hits;                // calls where bucket count > 1
    uint64_t bucketCountHist[8];  // index 0=0/1 units, 1=2, 2=3, ... 7=8+
    uint64_t lastLogCall;
};
static HeurStats gHeurStats = {};
static const uint64_t kHeurLogInterval = 500000;   // ~every 500k calls

// Tuning target: per-HIT penalty should be ~1.5 cells of base cost. Hits
// are only ~18% of cells, so global ratio is much smaller. We tune to make
// the penalty meaningful WHEN it triggers, not on average over all calls.
static const float    kTargetPenaltyPerHit = 1.5f;

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

    // Per-tick occupancy penalty: if the candidate cell falls into a
    // bucket that is currently crowded with moving units, add cost so
    // A* prefers a path around the cluster. Determinism: bucket key is
    // pure cell coords; sim tick comes from the engine state.
    float penaltyAdded = 0.0f;
    uint32_t observedCount = 0;
    if (pathOccupancyPenalty > 0.0f) {
        void* sim = *reinterpret_cast<void**>(pf + OFF_PF_SIM);
        if (sim) {
            uint32_t curTick = *reinterpret_cast<uint32_t*>(
                static_cast<uint8_t*>(sim) + OFF_SIM_CURTICK);
            int bcx = cx / OCC_BUCKET_SIZE;
            int bcz = cz / OCC_BUCKET_SIZE;
            uint32_t bucket = PathOcc_HashCell(bcx, bcz);
            uint32_t v = gPathOccupancyHash[bucket];
            if ((v >> 8) == curTick) {
                observedCount = v & 0xFFu;
                // Cap the effective count so single mega-clusters don't
                // produce prohibitively expensive crossings.
                uint32_t effective = observedCount;
                if (effective > OCC_COUNT_CAP) effective = OCC_COUNT_CAP;
                if (effective > 1) {
                    // ---- Wall-detection (A) -------------------------
                    // A "wall" looks like a single dense bucket whose
                    // 4 cardinal neighbors are ALSO crowded — units form
                    // a continuous obstruction. An isolated cluster has
                    // mostly empty neighbors. We sample 4 cells out and
                    // count how many are also crowded; 3+ → wall, double
                    // the per-cell penalty so a real wall becomes
                    // expensive enough that A* prefers a flank.
                    uint32_t wallNeighbors = 0;
                    {
                        uint32_t nb, nv;
                        nb = PathOcc_HashCell(bcx - 1, bcz);
                        nv = gPathOccupancyHash[nb];
                        if ((nv >> 8) == curTick && (nv & 0xFFu) > 1) ++wallNeighbors;

                        nb = PathOcc_HashCell(bcx + 1, bcz);
                        nv = gPathOccupancyHash[nb];
                        if ((nv >> 8) == curTick && (nv & 0xFFu) > 1) ++wallNeighbors;

                        nb = PathOcc_HashCell(bcx, bcz - 1);
                        nv = gPathOccupancyHash[nb];
                        if ((nv >> 8) == curTick && (nv & 0xFFu) > 1) ++wallNeighbors;

                        nb = PathOcc_HashCell(bcx, bcz + 1);
                        nv = gPathOccupancyHash[nb];
                        if ((nv >> 8) == curTick && (nv & 0xFFu) > 1) ++wallNeighbors;
                    }
                    float wallMult = (wallNeighbors >= 3) ? 2.0f
                                   : (wallNeighbors == 2) ? 1.4f
                                   : 1.0f;

                    // Distance-faded penalty: full strength far from goal,
                    // fades to zero as we approach. This is critical for
                    // engineers trying to reach a build site INSIDE a
                    // crowd — the penalty must not block them entering
                    // the destination cluster, only help routing on the
                    // way there. `major` is the octile distance to the
                    // goal rect (already computed above), measured in
                    // cells. Fade to zero over 10 cells.
                    float fade = major * (1.0f / 10.0f);
                    if (fade > 1.0f) fade = 1.0f;
                    // Subtract 1: exclude self-contribution to the bucket.
                    penaltyAdded = (float)(effective - 1) * pathOccupancyPenalty * fade * wallMult;
                    result += penaltyAdded;
                }
            }

            // ---- Reservation hash (Cooperative A*) ----------------------
            // Add penalty for cells already reserved by other units' just-
            // accepted paths. Same fade applies — reservations near the goal
            // don't matter (engineers reaching same build site).
            // Reservations are weighted at 0.6× direct occupancy because
            // they describe FUTURE positions which may or may not happen.
            uint32_t rv = gPathReservationHash[bucket];
            uint32_t rExp = rv >> 8;
            if (rExp > curTick) {
                uint32_t rCnt = rv & 0xFFu;
                if (rCnt > OCC_COUNT_CAP) rCnt = OCC_COUNT_CAP;
                if (rCnt > 0) {
                    float fade2 = major * (1.0f / 10.0f);
                    if (fade2 > 1.0f) fade2 = 1.0f;
                    float resPen = (float)rCnt * pathOccupancyPenalty * 0.6f * fade2;
                    penaltyAdded += resPen;
                    result      += resPen;
                }
            }
        }
    }

    // ---- Telemetry recording (remove in cleanup) ------------------------
    {
        ++gHeurStats.calls;
        gHeurStats.baseSum_x1000    += (uint64_t)((minor * 0.41421354f + major) * 1000.0f);
        gHeurStats.penaltySum_x1000 += (uint64_t)(penaltyAdded * 1000.0f);
        if (observedCount > 1) ++gHeurStats.hits;
        uint32_t bucketBin = (observedCount <= 1) ? 0u
                            : (observedCount >= 8 ? 7u : (observedCount - 1));
        ++gHeurStats.bucketCountHist[bucketBin];

        if (gHeurStats.calls - gHeurStats.lastLogCall >= kHeurLogInterval) {
            gHeurStats.lastLogCall = gHeurStats.calls;

            // Compute averages. All divisions through float to avoid
            // libgcc's __udivdi3 (we build with -nostdlib).
            float fcalls    = (float)gHeurStats.calls;
            float fhits     = (float)gHeurStats.hits;
            float baseAvg   = (float)gHeurStats.baseSum_x1000    / fcalls / 1000.0f;
            float penAvg    = (float)gHeurStats.penaltySum_x1000 / fcalls / 1000.0f;

            // Recommended penalty: target ~1.5 cells of cost PER HIT (not
            // averaged over all calls). When a cell is in a fresh bucket
            // we want the cost bump to be meaningful — comparable to
            // taking a 1-2 cell detour — but not crushing.
            float penPerHit = (fhits > 0.0f)
                            ? ((float)gHeurStats.penaltySum_x1000 / fhits / 1000.0f)
                            : 0.0f;
            float recommended = pathOccupancyPenalty;
            if (penPerHit > 0.0001f) {
                recommended = pathOccupancyPenalty * (kTargetPenaltyPerHit / penPerHit);
                if (recommended < 0.05f) recommended = 0.05f;
                if (recommended > 1.5f)  recommended = 1.5f;
            }
            float hitPct = fhits / fcalls * 100.0f;

            LogF("PathTune: calls=%u  hit_rate=%.1f%%  base_avg=%.2f  pen_per_hit=%.2f  current=%.3f  recommend=%.3f",
                 (uint32_t)gHeurStats.calls, hitPct, baseAvg, penPerHit,
                 pathOccupancyPenalty, recommended);
            LogF("PathTune: bucket_hist  c<=1:%u  c=2:%u  c=3:%u  c=4:%u  c=5:%u  c=6:%u  c=7:%u  c>=8:%u",
                 (uint32_t)gHeurStats.bucketCountHist[0],
                 (uint32_t)gHeurStats.bucketCountHist[1],
                 (uint32_t)gHeurStats.bucketCountHist[2],
                 (uint32_t)gHeurStats.bucketCountHist[3],
                 (uint32_t)gHeurStats.bucketCountHist[4],
                 (uint32_t)gHeurStats.bucketCountHist[5],
                 (uint32_t)gHeurStats.bucketCountHist[6],
                 (uint32_t)gHeurStats.bucketCountHist[7]);
        }
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
