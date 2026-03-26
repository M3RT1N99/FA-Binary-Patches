// ==========================================================================
// lane_system.cpp - Bidirectional Lane Separation System
//
// Dynamic lane system that detects opposing traffic and spreads lanes.
// 4 phases: Detection -> Spreading -> Interleaved pass-through -> Contracting
//
// Called from collision.cpp ApplySpread() per unit per tick.
// Uses NbrCache for neighbor detection.
// ==========================================================================
#include <windows.h>
#include <cstdint>
#include <cmath>
#include <cstring>

extern "C" void Log(const char* fmt, ...);

// Extern: NbrCache from collision.cpp
struct NbrEntry { float x, z; uint32_t addr; float size; };
extern NbrEntry g_NbrCache[];
extern volatile LONG g_NbrIdx;

// ==========================================================================
// Parameters
// ==========================================================================
#define MAX_LANE_UNITS     512
#define ONCOMING_DOT       -0.5f    // Dot threshold for "opposing" (cos 120deg, stricter)
#define SCAN_RADIUS_SQ     2500.0f  // 50^2 world units detection range (look further ahead)
#define SPREAD_FACTOR      3.0f     // Lanes get this much wider on opposing traffic
#define SPREAD_LERP        0.12f    // Lerp speed for spreading (faster reaction)
#define CONTRACT_LERP      0.04f    // Lerp speed for contracting (slower, more stable)
#define NUM_LANES          3        // Lanes per direction group (more compact, fewer edge issues)
#define LANE_BASE_WIDTH    1.2f     // Base lane width in unit sizes (slightly wider)

// ==========================================================================
// Per-Unit Lane State (persistent between ticks)
// ==========================================================================
struct UnitLaneState {
    uint32_t addr;
    float currentOffset;    // Current lateral offset (world units)
    float targetOffset;     // Target offset
    int oncomingCount;      // Number of detected opposing traffic neighbors
    uint32_t lastTick;
};
static UnitLaneState g_LaneStates[MAX_LANE_UNITS];
static uint32_t g_LaneTick = 0;

// Debug
static LONG g_laneDbgCalls = 0;
static LONG g_laneDbgOncoming = 0;
static float g_laneDbgMaxOffset = 0.0f;

// ==========================================================================
// Hash lookup for persistent lane state
// ==========================================================================
static UnitLaneState* FindLaneState(uint32_t addr) {
    int idx = (addr >> 4) % MAX_LANE_UNITS;
    for (int i = 0; i < 8; i++) {
        int slot = (idx + i) % MAX_LANE_UNITS;
        if (g_LaneStates[slot].addr == addr)
            return &g_LaneStates[slot];
        if (g_LaneStates[slot].addr == 0 ||
            g_LaneTick - g_LaneStates[slot].lastTick > 200) {
            g_LaneStates[slot] = {addr, 0.0f, 0.0f, 0, g_LaneTick};
            return &g_LaneStates[slot];
        }
    }
    return nullptr;
}

// ==========================================================================
// ComputeLaneOffset - Main function (called from collision.cpp)
//
// Computes the lateral offset for a unit based on:
// - Fixed lane assignment (hash)
// - Dynamic spreading on opposing traffic
// - Smooth lerp between states
//
// Parameters:
//   unit      - Unit address
//   myX, myZ  - Current position
//   fwdX, fwdZ, fwdLen - Travel direction (unnormalized + length)
//   mySize    - Unit size (blueprint)
//   tick      - Current tick counter
//
// Return: lateral offset in world units (+ = right, - = left)
// ==========================================================================
void ComputeLaneOffset(uint32_t unit, float myX, float myZ,
                       float fwdX, float fwdZ, float fwdLen,
                       float mySize, uint32_t tick,
                       float* outOffX, float* outOffZ) {
    *outOffX = 0.0f;
    *outOffZ = 0.0f;
    g_laneDbgCalls++;
    g_LaneTick = tick;

    if (fwdLen < 2.0f) return;

    // Normalized direction
    float dirX = fwdX / fwdLen;
    float dirZ = fwdZ / fwdLen;

    // Perpendicular RIGHT of the travel direction (NOT canonical!)
    // Right-hand traffic: each unit dodges to the RIGHT relative to its travel direction
    // Army A heading NE -> "right" is SE
    // Army B heading SW -> "right" is NW
    // = automatically different sides!
    float rightX =  dirZ;  // 90 degrees right of travel direction
    float rightZ = -dirX;

    // Lane assignment per hash (1..NUM_LANES)
    uint32_t hash = (unit >> 4) * 2654435761u;
    int subLane = (int)(hash % NUM_LANES) + 1;  // 1, 2, 3

    // Base offset: ALWAYS to the right (positive)
    // Each unit drives on the right side of its travel direction
    float baseOffset = (float)subLane * mySize * LANE_BASE_WIDTH;

    // --- Neighbor scan: count opposing traffic ---
    LONG nbrCount = g_NbrIdx;
    if (nbrCount > MAX_LANE_UNITS) nbrCount = MAX_LANE_UNITS;

    int oncomingNbrs = 0;
    int sameGroupNbrs = 0;

    for (LONG i = 0; i < nbrCount; i++) {
        if (g_NbrCache[i].addr == unit) continue;

        float dx = g_NbrCache[i].x - myX;
        float dz = g_NbrCache[i].z - myZ;
        float distSq = dx * dx + dz * dz;

        if (distSq > SCAN_RADIUS_SQ) continue;

        // Estimate neighbor direction: vector from neighbor to us
        // (rough approximation: if the neighbor is heading in the opposite
        //  direction, dot(ourDir, neighborDir) < threshold)
        // Since we don't know the neighbor's direction directly, we use
        // the neighbor->us vector as proxy for "they're heading towards us"
        float dist = sqrtf(distSq);
        float toUsX = -dx / dist;  // Direction neighbor->us
        float toUsZ = -dz / dist;

        // Dot our direction vs neighbor->us direction
        // If positive: neighbor is AHEAD of us (same direction or stationary)
        // If negative: neighbor is COMING TOWARDS us (opposing traffic!)
        float dot = dirX * toUsX + dirZ * toUsZ;

        if (dot < ONCOMING_DOT) {
            oncomingNbrs++;
        } else if (dot > 0.3f) {
            sameGroupNbrs++;
        }
    }

    // --- Update lane state ---
    UnitLaneState* state = FindLaneState(unit);
    if (!state) {
        *outOffX = rightX * baseOffset;
        *outOffZ = rightZ * baseOffset;
        return;
    }

    state->lastTick = tick;
    state->oncomingCount = oncomingNbrs;

    // Target offset: spread when opposing traffic, otherwise normal
    if (oncomingNbrs > 0) {
        // Spread: lanes get wider
        state->targetOffset = baseOffset * SPREAD_FACTOR;
        state->currentOffset += (state->targetOffset - state->currentOffset) * SPREAD_LERP;
        g_laneDbgOncoming++;
    } else {
        // Contract: back to normal lane
        state->targetOffset = baseOffset;
        state->currentOffset += (state->targetOffset - state->currentOffset) * CONTRACT_LERP;
    }

    // Output: offset in world coordinates (rightX/Z * magnitude)
    *outOffX = rightX * state->currentOffset;
    *outOffZ = rightZ * state->currentOffset;

    // Debug
    float absOff = fabsf(state->currentOffset);
    if (absOff > g_laneDbgMaxOffset) g_laneDbgMaxOffset = absOff;
}

// ==========================================================================
// Debug + Toggle (called from collision.cpp)
// ==========================================================================
void LaneSystemDebugLog() {
    Log("[LANE] calls=%d oncoming=%d maxOff=%.1f\n",
        g_laneDbgCalls, g_laneDbgOncoming, g_laneDbgMaxOffset);
    g_laneDbgCalls = 0;
    g_laneDbgOncoming = 0;
    g_laneDbgMaxOffset = 0.0f;
}
