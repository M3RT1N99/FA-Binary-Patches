// ==========================================================================
// PathfinderOccupancy.cpp — Per-tick spatial occupancy hash for soft A*
// collision-avoidance.
//
// Hooks Moho::CAiSteeringImpl::TaskTick (0x5D32B0). For each moving unit,
// hashes the unit's coarse cell position into a 4096-entry table. Each
// bucket carries the sim tick it was last touched, so old data is
// implicitly stale and no clearing pass is needed.
//
// The heuristic side (PathfinderTuning.cpp / ComputeHeuristicC) reads
// this table and adds a soft penalty to A* candidate cells whose bucket
// is crowded in the current tick.
//
// Determinism: the hash uses ONLY cell coordinates (cx, cz). No pointers,
// no memory addresses, no random state. Byte-identical across MP clients.
//
// Hook protocol (entry at 0x5D32B0):
//   - 6-byte prolog overwritten with JMP to OnTickObserveHook
//   - Thunk preserves all registers (pushad), calls OnTickObserveC(this)
//   - Thunk re-executes the overwritten 3 prolog instructions, then
//     jumps back to 0x5D32B6 to continue the original function
// ==========================================================================

#include "moho.h"
#include "global.h"
#include "MovementConfig.h"
#include "PathfinderOccupancy.h"
#include <stdint.h>

// --------------------------------------------------------------------------
// Spatial hash storage. Constants and hash function live in the header.
// --------------------------------------------------------------------------
uint32_t gPathOccupancyHash[OCC_HASH_SIZE] = {0};

// Unit::GetPosition is a virtual method.  vftable slot is at byte offset
// 0x14 = index 5 (per IDA disassembly of TaskTick around 0x5D3503).
// __fastcall(this, dummy_edx) matches __thiscall ABI for a 1-arg member.
// We use the vftable call rather than a direct field read because the
// Unit struct holds multiple position-like vectors (Pos1, Pos2, Pos3, …)
// at non-trivial offsets and OFF_UNIT_POS=0x160 in MovementConfig.h is
// stale. The vftable is the engine's authoritative source.
typedef float* (__fastcall* GetPositionFn)(void* unit, int dummyEdx);
#define UNIT_GETPOS_VFTABLE_SLOT  5

// --------------------------------------------------------------------------
// Diagnostic counters — removed in cleanup commit. Keep until end-to-end
// verification with the heuristic read is done.
// --------------------------------------------------------------------------
static uint32_t gOnTickCallCount = 0;
static uint32_t gLastDiagTick    = 0;

// --------------------------------------------------------------------------
// OnTick observer — called from asm thunk on entry to TaskTick.
//
// ctask = Moho::CAiSteeringImpl_CTask*  (the steering task, "this")
// We resolve unit, sim tick, position via verified field offsets and
// update one bucket in the spatial hash.
// --------------------------------------------------------------------------
extern "C" void __cdecl OnTickObserveC(void* ctask)
{
    if (!ctask) return;

    // ctask + 0x1C → Unit*
    void* unit = *reinterpret_cast<void**>(static_cast<uint8_t*>(ctask) + OFF_STEERING_UNIT);
    if (!unit) return;

    // Unit + 0x150 → Sim*
    void* sim = *reinterpret_cast<void**>(static_cast<uint8_t*>(unit) + OFF_UNIT_SIM);
    if (!sim) return;

    // Sim + 0x900 → int mCurTick
    uint32_t curTick = *reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(sim) + OFF_SIM_CURTICK);

    // Resolve unit position via vftable[5] = Unit::GetPosition
    void** vftable = *reinterpret_cast<void***>(unit);
    if (!vftable) return;
    GetPositionFn getPos = reinterpret_cast<GetPositionFn>(vftable[UNIT_GETPOS_VFTABLE_SLOT]);
    if (!getPos) return;
    float* pos = getPos(unit, 0);
    if (!pos) return;

    // Coarse-bucket cell coordinates
    int cx = static_cast<int>(pos[0]) / OCC_BUCKET_SIZE;
    int cz = static_cast<int>(pos[2]) / OCC_BUCKET_SIZE;

    uint32_t bucket = PathOcc_HashCell(cx, cz);
    uint32_t cur    = gPathOccupancyHash[bucket];
    uint32_t bTick  = cur >> 8;
    uint32_t bCount = cur & 0xFFu;

    if (bTick != curTick) {
        // First touch this tick — reset count
        gPathOccupancyHash[bucket] = (curTick << 8) | 1u;
    } else if (bCount < 255u) {
        gPathOccupancyHash[bucket] = (curTick << 8) | (bCount + 1u);
    }
    // (else: saturated at 255, leave it)

    // ---- diagnostic (remove in cleanup) ---------------------------------
    if (gOnTickCallCount < 5) {
        LogF("PathOcc: OnTick #%u unit=%p pos=(%.1f,%.1f,%.1f) cell=(%d,%d) bucket=%u tick=%u",
             gOnTickCallCount, unit, pos[0], pos[1], pos[2], cx, cz, bucket, curTick);
    }
    if (curTick != gLastDiagTick && (curTick % 100) == 0 && curTick > 1) {
        gLastDiagTick = curTick;
        uint32_t prev = curTick - 1;
        uint32_t live = 0, totalCount = 0, maxCount = 0;
        for (uint32_t i = 0; i < OCC_HASH_SIZE; ++i) {
            uint32_t v = gPathOccupancyHash[i];
            if ((v >> 8) == prev) {
                ++live;
                uint32_t c = v & 0xFFu;
                totalCount += c;
                if (c > maxCount) maxCount = c;
            }
        }
        LogF("PathOcc: prev_tick=%u live_buckets=%u sum=%u max_in_bucket=%u",
             prev, live, totalCount, maxCount);
    }
    ++gOnTickCallCount;
}

// --------------------------------------------------------------------------
// Asm thunk — entry from JMP at 0x5D32B0.
//
// On entry: ecx = ctask (this). Stack as on normal call.
//
// We pushad, push ecx as the C arg, call our handler, clean the arg,
// popad, then re-execute the overwritten prolog and jmp into the rest
// of TaskTick at 0x5D32B6 (sub esp, 58h).
// --------------------------------------------------------------------------
asm(
    ".global _OnTickObserveHook\n"
    "_OnTickObserveHook:\n"

    "    pushad\n"
    "    push ecx\n"               // arg1: ctask
    "    call _OnTickObserveC\n"
    "    add esp, 4\n"
    "    popad\n"

    // Re-execute the original prolog bytes we overwrote (6 bytes total):
    "    push ebp\n"               //   55
    "    mov ebp, esp\n"           //   8B EC
    "    and esp, 0xFFFFFFF8\n"    //   83 E4 F8

    "    jmp 0x005D32B6\n"         // continue at sub esp, 58h
);
