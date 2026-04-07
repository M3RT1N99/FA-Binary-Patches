// ==========================================================================
// RepathTuning.cpp — Tunable repath/stuck thresholds for CAiPathNavigator.
//
// Targets the five hardcoded immediate constants inside
// Moho::CAiPathNavigator::UpdateCurrentPosition (0x5AE2D0). Each constant
// is a single byte, so we runtime-patch via VirtualProtect (same pattern
// as SetPathRectHistoryDepth in PathfinderTuning.cpp).
//
// Engine constants (verified in IDA):
//
//   0x5AE627  add edx, 0Ah          repath cooldown (ticks between requests)
//   0x5AE657  add eax, 64h          formation-sync cooldown (ticks)
//   0x5AEB3D  cmp [ebx+74h], 1Eh    stuck threshold #1 (mNoProgressTickCount)
//   0x5AEBBC  cmp [ebx+74h], 1Eh    stuck threshold #2 (same field, second site)
//   0x5AEC5F  mov [ebx+64h], 0Ah    retry countdown after pathfind failure
//
// Stuck threshold has TWO sites for the same field — both must be patched
// in lockstep. SetPathStuckThreshold() does this atomically.
//
// Lua side (Sim-Lua context):
//   SetPathRepathCooldown(int n)         default 10
//   SetPathFormationSyncCooldown(int n)  default 100
//   SetPathStuckThreshold(int n)         default 30  (raised to 60 at startup)
//   SetPathRetryCountdown(int n)         default 10
//
// At startup we bump the stuck threshold from 30 → 60 ticks via an
// __attribute__((constructor)) — the most impactful tweak, gives units
// twice as long to push through transient blockages before A* gives up.
// ==========================================================================

#include "LuaAPI.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"
#include <stdint.h>

// --------------------------------------------------------------------------
// Byte addresses (single-byte cmp/add/mov immediates inside 0x5AE2D0)
// --------------------------------------------------------------------------
static const uintptr_t REPATH_COOLDOWN_IMM   = 0x005AE627;
static const uintptr_t FORMATION_SYNC_IMM    = 0x005AE657;
static const uintptr_t STUCK_THRESHOLD_1     = 0x005AEB3D;
static const uintptr_t STUCK_THRESHOLD_2     = 0x005AEBBC;
static const uintptr_t RETRY_COUNTDOWN_IMM   = 0x005AEC5F;

// --------------------------------------------------------------------------
// VirtualProtect helper — same pattern as SetPathRectHistoryDepth.
// Looked up at call time so we don't need a static initializer.
// --------------------------------------------------------------------------
static void PatchByte(uintptr_t addr, uint8_t value)
{
    auto vp = reinterpret_cast<bool(__stdcall*)(void*, size_t, uint32_t, uint32_t*)>(
        GetProcAddress(GetModuleHandleA("KERNEL32"), "VirtualProtect")
    );
    if (!vp) return;

    uint32_t oldProt = 0;
    vp(reinterpret_cast<void*>(addr), 1, 0x40 /*RWX*/, &oldProt);
    *reinterpret_cast<uint8_t*>(addr) = value;
    vp(reinterpret_cast<void*>(addr), 1, oldProt, &oldProt);
}

// --------------------------------------------------------------------------
// Lua setters — all clamp to signed-byte range so the cmp imm8 stays valid.
// --------------------------------------------------------------------------
int SetPathRepathCooldown(lua_State* L)
{
    int v = (int)luaL_optnumber(L, 1, 10);
    if (v < 0)   v = 0;
    if (v > 127) v = 127;
    PatchByte(REPATH_COOLDOWN_IMM, (uint8_t)v);
    return 0;
}
SimRegFunc SetPathRepathCooldownReg{
    "SetPathRepathCooldown", "(int n=10) min ticks between repath requests", SetPathRepathCooldown
};

int SetPathFormationSyncCooldown(lua_State* L)
{
    int v = (int)luaL_optnumber(L, 1, 100);
    if (v < 0)   v = 0;
    if (v > 127) v = 127;
    PatchByte(FORMATION_SYNC_IMM, (uint8_t)v);
    return 0;
}
SimRegFunc SetPathFormationSyncCooldownReg{
    "SetPathFormationSyncCooldown", "(int n=100) min ticks between formation syncs", SetPathFormationSyncCooldown
};

int SetPathStuckThreshold(lua_State* L)
{
    int v = (int)luaL_optnumber(L, 1, 30);
    if (v < 1)   v = 1;
    if (v > 127) v = 127;
    // Two sites checking the same field — patch in lockstep.
    PatchByte(STUCK_THRESHOLD_1, (uint8_t)v);
    PatchByte(STUCK_THRESHOLD_2, (uint8_t)v);
    return 0;
}
SimRegFunc SetPathStuckThresholdReg{
    "SetPathStuckThreshold", "(int n=30) ticks of no progress before path abort", SetPathStuckThreshold
};

int SetPathRetryCountdown(lua_State* L)
{
    int v = (int)luaL_optnumber(L, 1, 10);
    if (v < 1)   v = 1;
    if (v > 127) v = 127;
    PatchByte(RETRY_COUNTDOWN_IMM, (uint8_t)v);
    return 0;
}
SimRegFunc SetPathRetryCountdownReg{
    "SetPathRetryCountdown", "(int n=10) wait ticks after pathfind failure", SetPathRetryCountdown
};

// --------------------------------------------------------------------------
// Production defaults applied at startup. Only the stuck threshold is
// touched — the other constants are left at vanilla because they have
// less obvious payoff and more risk of side effects (formation logic,
// retry storms, etc.). The stuck threshold is the highest-leverage tweak
// per the IDA analysis: it directly controls how long an engineer/unit
// will keep trying to push through a transient blockage before A* gives
// up and reports the path failed.
// --------------------------------------------------------------------------
__attribute__((constructor))
static void RepathTuning_ApplyDefaults()
{
    PatchByte(STUCK_THRESHOLD_1, 60);   // vanilla 30 → 60 ticks (~6s @ sim 10)
    PatchByte(STUCK_THRESHOLD_2, 60);
}
