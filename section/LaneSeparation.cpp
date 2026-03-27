// ==========================================================================
// LaneSeparation.cpp — Bidirectional Lane Separation
//
// Assigns each unit a lane on the RIGHT side of its travel direction.
// When oncoming traffic is detected, lanes spread apart dynamically.
// Ported from test-dll/lane_system.cpp to FA_Patcher section format.
//
// Called from MovementSystem.cpp OnSteeringTick().
// Uses SpatialHash neighbors (passed in, not global NbrCache).
// ==========================================================================

#include "CObject.h"
#include "moho.h"
#include "global.h"
#include "MovementConfig.h"

// SpatialEntry defined in MovementConfig.h

// ==========================================================================
// Per-Unit Lane State (persistent between ticks)
// ==========================================================================

struct UnitLaneState {
    uint32_t addr;
    float currentOffset;    // Current lateral offset (world units)
    float targetOffset;     // Target offset (lerp toward this)
    int oncomingCount;      // Detected oncoming neighbors
    uint32_t lastTick;
};

#define LANE_MAX_UNITS  512
#define LANE_SCAN_RADIUS_SQ  2500.0f  // 50^2 detection range
#define LANE_SAME_DOT   0.3f          // Dot threshold for same-direction

static UnitLaneState g_LaneStates[LANE_MAX_UNITS];
static uint32_t g_LaneTick = 0;

// Debug counters
static int g_LaneDbgCalls    = 0;
static int g_LaneDbgOncoming = 0;
static float g_LaneDbgMaxOff = 0.0f;

// ==========================================================================
// Hash lookup for persistent per-unit state
// ==========================================================================

static UnitLaneState* FindLaneState(uint32_t addr) {
    int idx = (addr >> 4) % LANE_MAX_UNITS;
    for (int i = 0; i < 8; i++) {
        int slot = (idx + i) % LANE_MAX_UNITS;
        if (g_LaneStates[slot].addr == addr)
            return &g_LaneStates[slot];
        if (g_LaneStates[slot].addr == 0 ||
            g_LaneTick - g_LaneStates[slot].lastTick > 200) {
            g_LaneStates[slot].addr = addr;
            g_LaneStates[slot].currentOffset = 0.0f;
            g_LaneStates[slot].targetOffset = 0.0f;
            g_LaneStates[slot].oncomingCount = 0;
            g_LaneStates[slot].lastTick = g_LaneTick;
            return &g_LaneStates[slot];
        }
    }
    return 0;
}

// ==========================================================================
// ComputeLaneOffset — main entry point
//
// Computes lateral offset for a unit based on:
// - Fixed lane assignment (hash of unit addr)
// - Dynamic spreading when oncoming traffic detected
// - Smooth lerp between states
//
// outOffX/outOffZ: lateral offset in world coordinates
//   Positive = right of travel direction (Rechtsverkehr)
// ==========================================================================

extern "C" void ComputeLaneOffset(
    uint32_t unitAddr, float myX, float myZ,
    float fwdX, float fwdZ, float fwdLen,
    float mySize, uint32_t tick,
    SpatialEntry* neighbors, int nbrCount,
    float* outOffX, float* outOffZ)
{
    *outOffX = 0.0f;
    *outOffZ = 0.0f;
    g_LaneDbgCalls++;
    g_LaneTick = tick;

    // Need minimum velocity to determine direction
    if (fwdLen < 2.0f) return;

    // Normalized travel direction
    float dirX = fwdX / fwdLen;
    float dirZ = fwdZ / fwdLen;

    // Perpendicular RIGHT of travel direction
    // Army A heading NE -> right is SE
    // Army B heading SW -> right is NW
    // = automatic separation on opposite sides
    float rightX =  dirZ;
    float rightZ = -dirX;

    // Lane assignment via hash (1..LANE_COUNT)
    uint32_t hash = (unitAddr >> 4) * 2654435761u;
    int subLane = (int)(hash % LANE_COUNT) + 1;

    // Base offset: always to the right (positive)
    float baseOffset = (float)subLane * mySize * LANE_BASE_WIDTH;

    // --- Neighbor scan: count oncoming traffic ---
    int oncomingNbrs = 0;

    for (int i = 0; i < nbrCount; i++) {
        if (neighbors[i].unitAddr == unitAddr) continue;

        float dx = neighbors[i].x - myX;
        float dz = neighbors[i].z - myZ;
        float distSq = dx * dx + dz * dz;

        if (distSq > LANE_SCAN_RADIUS_SQ || distSq < 0.01f) continue;

        float dist = 1.0f;
        // Fast inverse sqrt approximation
        {
            float xhalf = 0.5f * distSq;
            int fi = *(int*)&distSq;
            fi = 0x5f3759df - (fi >> 1);
            float invSqrt = *(float*)&fi;
            invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
            dist = 1.0f / invSqrt;
        }

        // Direction neighbor -> us
        float toUsX = -dx / dist;
        float toUsZ = -dz / dist;

        // Dot our direction vs neighbor->us direction
        // Negative = neighbor is coming TOWARD us (oncoming!)
        float dot = dirX * toUsX + dirZ * toUsZ;

        if (dot < LANE_ONCOMING_DOT) {
            oncomingNbrs++;
        }
    }

    // --- Update persistent lane state ---
    UnitLaneState* state = FindLaneState(unitAddr);
    if (!state) {
        *outOffX = rightX * baseOffset;
        *outOffZ = rightZ * baseOffset;
        return;
    }

    state->lastTick = tick;
    state->oncomingCount = oncomingNbrs;

    if (oncomingNbrs > 0) {
        // Spread: lanes get wider
        state->targetOffset = baseOffset * LANE_SPREAD_FACTOR;
        state->currentOffset += (state->targetOffset - state->currentOffset) * LANE_SPREAD_LERP;
        g_LaneDbgOncoming++;
    } else {
        // Contract: back to normal lanes
        state->targetOffset = baseOffset;
        state->currentOffset += (state->targetOffset - state->currentOffset) * LANE_CONTRACT_LERP;
    }

    *outOffX = rightX * state->currentOffset;
    *outOffZ = rightZ * state->currentOffset;

    // Track max offset for debug
    float absOff = state->currentOffset;
    if (absOff < 0.0f) absOff = -absOff;
    if (absOff > g_LaneDbgMaxOff) g_LaneDbgMaxOff = absOff;
}

// ==========================================================================
// Debug logging (called from MovementSystem periodic log)
// ==========================================================================

extern "C" void LaneDebugLog() {
    LogF("[LANE] calls=%d oncoming=%d maxOff=%.1f\n",
         g_LaneDbgCalls, g_LaneDbgOncoming, g_LaneDbgMaxOff);
    g_LaneDbgCalls = 0;
    g_LaneDbgOncoming = 0;
    g_LaneDbgMaxOff = 0.0f;
}
