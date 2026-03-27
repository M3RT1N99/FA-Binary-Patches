// ==========================================================================
// GetTargetPosHook.cpp — Phase 2c: SpatialHash + Lane Separation
//
// Three-point hook on GetTargetPos (0x5AD8B0):
//   Prolog:   Save ECX (navigator) to global
//   Epilog A: Apply lane offset on formation-leader return
//   Epilog B: Apply lane offset on cell-target return (EDI clobbered!)
// ==========================================================================

#include "CObject.h"
#include "moho.h"
#include "global.h"
#include "MovementConfig.h"

// Navigator offsets (IDA-verified)
#define OFF_NAV_PATHFINDER  0x10
#define OFF_PF_UNIT         0x18
#define VTABLE_UNIT         0x00E2A574

// Forward declarations (defined in TransientGrid.cpp)
float TGrid_QueryOffset(float px, float pz, float fwdX, float fwdZ,
                        uint32_t myUnitId, uint32_t tick);

// Max offset clamp (world units)
#define GTP_MAX_OFFSET      4.0f

// Saved navigator pointer (written by prolog, read by epilogs)
static uint32_t g_SavedNavigator = 0;

// Debug counters
static int g_GtpCalls = 0;
static int g_GtpApplied = 0;
static float g_GtpMaxOff = 0.0f;
static uint32_t g_GtpLastLogTick = 0;

// Per-unit velocity tracking (previous frame positions)
#define VEL_TRACK_MAX 512
struct VelTrack {
    uint32_t unitAddr;
    float lastX, lastZ;
    float velX, velZ;       // estimated velocity
    uint32_t lastTick;
};
static VelTrack g_VelTrack[VEL_TRACK_MAX];

static VelTrack* FindVelTrack(uint32_t addr, uint32_t tick) {
    int idx = (addr >> 4) % VEL_TRACK_MAX;
    for (int i = 0; i < 4; i++) {
        int slot = (idx + i) % VEL_TRACK_MAX;
        if (g_VelTrack[slot].unitAddr == addr)
            return &g_VelTrack[slot];
        if (g_VelTrack[slot].unitAddr == 0 ||
            tick - g_VelTrack[slot].lastTick > 100) {
            g_VelTrack[slot].unitAddr = addr;
            g_VelTrack[slot].lastX = 0;
            g_VelTrack[slot].lastZ = 0;
            g_VelTrack[slot].velX = 0;
            g_VelTrack[slot].velZ = 0;
            g_VelTrack[slot].lastTick = tick;
            return &g_VelTrack[slot];
        }
    }
    return 0;
}

// SpatialHash query (defined in MovementSystem.cpp)
int MovSys_QueryNeighbors(float x, float z, float radius,
                          SpatialEntry* outResults, int maxResults,
                          uint32_t excludeUnit);

// Lane separation (defined in LaneSeparation.cpp)
extern "C" void ComputeLaneOffset(
    uint32_t unitAddr, float myX, float myZ,
    float fwdX, float fwdZ, float fwdLen,
    float mySize, uint32_t tick,
    SpatialEntry* neighbors, int nbrCount,
    float* outOffX, float* outOffZ);

// ==========================================================================
// ApplyMovementOffset — modifies GetTargetPos output in-place
// ==========================================================================

extern "C" void ApplyMovementOffset(float* outPos) {
    if (!outPos) return;

    uint32_t navigatorAddr = g_SavedNavigator;
    if (!IsValidPtr(navigatorAddr)) return;

    // navigator -> pathfinder -> unit
    uint32_t pfAddr = *(uint32_t*)(navigatorAddr + OFF_NAV_PATHFINDER);
    if (!IsValidPtr(pfAddr)) return;
    uint32_t unitAddr = *(uint32_t*)(pfAddr + OFF_PF_UNIT);
    if (!IsValidPtr(unitAddr)) return;

    // VTable check
    if (*(uint32_t*)unitAddr != VTABLE_UNIT) return;

    uint8_t* unit = (uint8_t*)unitAddr;
    g_GtpCalls++;

    // Read unit position
    float px = *(float*)(unit + OFF_UNIT_POS + 0x00);
    float pz = *(float*)(unit + OFF_UNIT_POS + 0x08);

    // Target position from output buffer (Vector3f: x, y, z)
    float tx = outPos[0];
    float tz = outPos[2];

    // Forward direction = target - position (stable, doesn't oscillate)
    float fwdX = tx - px;
    float fwdZ = tz - pz;
    float fwdLenSq = fwdX * fwdX + fwdZ * fwdZ;

    // No offset when close to target — let unit reach its formation position
    if (fwdLenSq < 16.0f) return;  // < 4.0 world units — too close, no offset

    // Fast sqrt
    float fwdLen;
    {
        float xhalf = 0.5f * fwdLenSq;
        int fi = *(int*)&fwdLenSq;
        fi = 0x5f3759df - (fi >> 1);
        float invSqrt = *(float*)&fi;
        invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
        fwdLen = 1.0f / invSqrt;
    }

    // Get tick
    uint32_t tick = 0;
    uint32_t simAddr = *(uint32_t*)(unit + OFF_UNIT_SIM);
    if (IsValidPtr(simAddr)) {
        tick = *(uint32_t*)(simAddr + OFF_SIM_CURTICK);
    }

    // Query spatial hash for nearby units
    SpatialEntry neighbors[RVO_MAX_NEIGHBORS];
    int nbrCount = MovSys_QueryNeighbors(px, pz, RVO_NEIGHBOR_RADIUS,
                                          neighbors, RVO_MAX_NEIGHBORS,
                                          unitAddr);

    if (nbrCount == 0) return;

    // Separation force: push AWAY from nearby units
    float sepX = 0.0f, sepZ = 0.0f;
    int sepCount = 0;

    for (int i = 0; i < nbrCount; i++) {
        if (neighbors[i].unitAddr == unitAddr) continue;
        float dx = px - neighbors[i].x;  // Vector FROM neighbor TO us
        float dz = pz - neighbors[i].z;
        float distSq = dx * dx + dz * dz;

        if (distSq < 0.01f || distSq > 64.0f) continue;

        float weight = 1.0f / distSq;
        sepX += dx * weight;
        sepZ += dz * weight;
        sepCount++;
    }

    if (sepCount == 0) return;

    float sepMag = sepX * sepX + sepZ * sepZ;
    if (sepMag < 0.0001f) return;

    // Normalize and scale
    float invMag;
    {
        float xhalf = 0.5f * sepMag;
        int fi = *(int*)&sepMag;
        fi = 0x5f3759df - (fi >> 1);
        invMag = *(float*)&fi;
        invMag = invMag * (1.5f - xhalf * invMag * invMag);
    }

    // Target offset: 1.0 world unit
    float targetOffX = sepX * invMag * 1.0f;
    float targetOffZ = sepZ * invMag * 1.0f;

    // Fade near target: full offset > 15 wu, zero < 4
    float fade = 1.0f;
    if (fwdLen < 15.0f) {
        fade = (fwdLen - 4.0f) / 11.0f;
        if (fade < 0.0f) fade = 0.0f;
    }
    targetOffX *= fade;
    targetOffZ *= fade;

    // Damping: smooth offset changes using per-unit state (VelTrack)
    // This prevents left-right oscillation
    VelTrack* vt = FindVelTrack(unitAddr, tick);
    float offX, offZ;
    if (vt && vt->lastTick != 0) {
        // Lerp toward target offset (0.15 = smooth, no jitter)
        offX = vt->velX + (targetOffX - vt->velX) * 0.15f;
        offZ = vt->velZ + (targetOffZ - vt->velZ) * 0.15f;
        vt->velX = offX;
        vt->velZ = offZ;
        vt->lastTick = tick;
    } else {
        // First frame: use target directly
        offX = targetOffX;
        offZ = targetOffZ;
        if (vt) {
            vt->velX = offX;
            vt->velZ = offZ;
            vt->lastX = px;
            vt->lastZ = pz;
            vt->lastTick = tick;
        }
    }

    // Apply smoothed offset to output position
    outPos[0] += offX;
    outPos[2] += offZ;

    g_GtpApplied++;
    float appliedMag = offX * offX + offZ * offZ;
    if (appliedMag > g_GtpMaxOff) g_GtpMaxOff = appliedMag;

    // Periodic log
    if (tick < g_GtpLastLogTick) g_GtpLastLogTick = 0; // new match reset
    if (tick - g_GtpLastLogTick > 50) {
        LogF("[GTP] tick=%u calls=%d applied=%d maxOff=%.2f\n",
             tick, g_GtpCalls, g_GtpApplied, g_GtpMaxOff);
        g_GtpCalls = 0;
        g_GtpApplied = 0;
        g_GtpMaxOff = 0.0f;
        g_GtpLastLogTick = tick;
    }
}

// ==========================================================================
// Prolog hook (0x5AD8B0, 6 bytes)
// Entry: ECX = CAiPathNavigator*, EAX = Vector3f* outPos
// Overwritten: PUSH EBP / MOV EBP,ESP / AND ESP,0xFFFFFFC0
// ==========================================================================

void GetTargetPosPrologue() {
    asm(
        "mov %[nav], ecx;"
        "push ebp;"
        "mov ebp, esp;"
        "and esp, 0xFFFFFFC0;"
        "jmp 0x5AD8B6;"
        :
        : [nav] "m"(g_SavedNavigator)
        :
    );
}

// ==========================================================================
// Epilog A (0x5AD935, 7 bytes) — Formation-leader return
// ==========================================================================

void GetTargetPosEpilogueA() {
    asm(
        "pushad;"
        "push esi;"
        "call %[func];"
        "add esp, 4;"
        "popad;"
        "pop edi;"
        "pop esi;"
        "pop ebx;"
        "mov esp, ebp;"
        "pop ebp;"
        "ret;"
        :
        : [func] "i"(ApplyMovementOffset)
        :
    );
}

// ==========================================================================
// Epilog B (0x5AD9B1, 9 bytes) — Cell-target return
// ==========================================================================

void GetTargetPosEpilogueB() {
    asm(
        "pushad;"
        "push esi;"
        "call %[func];"
        "add esp, 4;"
        "popad;"
        "pop edi;"
        "mov eax, esi;"
        "pop esi;"
        "pop ebx;"
        "mov esp, ebp;"
        "pop ebp;"
        "ret;"
        :
        : [func] "i"(ApplyMovementOffset)
        :
    );
}
