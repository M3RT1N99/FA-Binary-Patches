// ==========================================================================
// TransientGrid.cpp — Transient Occupancy Grid for movement avoidance
//
// A lightweight 2D grid that tracks where moving units ARE and where
// they WILL BE (cells ahead of them). Other units query this grid
// to find a clear lateral offset to avoid collisions.
//
// Grid covers the full map at 1-cell-per-world-unit resolution.
// Each cell stores a tick counter — if close to current tick, it's "occupied".
// Cells decay automatically (no explicit clear needed).
//
// Called from:
//   OnSteeringTick  — marks cells occupied by moving units
//   GetTargetPosHook — queries grid to compute avoidance offset
// ==========================================================================

#include "CObject.h"
#include "moho.h"
#include "global.h"
#include "MovementConfig.h"

// Grid dimensions (covers 1024x1024 map, 1 cell = 1 world unit)
// For 2048 maps, units beyond 1024 wrap (modulo). Acceptable tradeoff.
#define TGRID_SIZE      1024
#define TGRID_MASK      (TGRID_SIZE - 1)
#define TGRID_DECAY     15      // Cells valid for 15 ticks (~1.5 sec)
#define TGRID_LOOKAHEAD 5       // Mark 5 cells ahead of travel direction

// Each cell stores the tick it was last marked + the unit ID that marked it
struct TGridCell {
    uint16_t tick;      // Lower 16 bits of sim tick when marked
    uint16_t unitId;    // Unit ID that marked this cell (0 = empty)
};

// The grid (1024*1024 * 4 bytes = 4MB — too big for section!)
// Use 512x512 instead (512*512 * 4 = 1MB — still too big)
// Use 256x256 (256*256 * 4 = 256KB — fits in section)
#undef TGRID_SIZE
#undef TGRID_MASK
#define TGRID_SIZE      256
#define TGRID_MASK      (TGRID_SIZE - 1)
#define TGRID_CELL_SIZE 1.0f    // 1:1 with engine pathfinding grid

static TGridCell g_TGrid[TGRID_SIZE * TGRID_SIZE];

// Convert world position to grid coordinates
static int TGridX(float worldX) { return ((int)(worldX / TGRID_CELL_SIZE)) & TGRID_MASK; }
static int TGridZ(float worldZ) { return ((int)(worldZ / TGRID_CELL_SIZE)) & TGRID_MASK; }
static int TGridIdx(int gx, int gz) { return (gz & TGRID_MASK) * TGRID_SIZE + (gx & TGRID_MASK); }

// ==========================================================================
// Mark cells occupied by a moving unit
// Called from OnSteeringTick for each unit every frame
// ==========================================================================

void TGrid_MarkUnit(float px, float pz, float velX, float velZ,
                    uint32_t unitId, uint32_t tick, int footprint) {
    uint16_t t = (uint16_t)(tick & 0xFFFF);
    uint16_t uid = (uint16_t)(unitId & 0xFFFF);
    if (uid == 0) uid = 1;

    // Mark footprint area (footprint x footprint cells centered on unit)
    int halfFp = footprint / 2;
    int gx0 = TGridX(px);
    int gz0 = TGridZ(pz);
    for (int dx = -halfFp; dx <= halfFp; dx++) {
        for (int dz = -halfFp; dz <= halfFp; dz++) {
            int idx = TGridIdx(gx0 + dx, gz0 + dz);
            g_TGrid[idx].tick = t;
            g_TGrid[idx].unitId = uid;
        }
    }

    // Mark cells ahead (in velocity direction) if moving
    float speed = velX * velX + velZ * velZ;
    if (speed < 0.01f) return;

    float invSpeed;
    {
        float xhalf = 0.5f * speed;
        int fi = *(int*)&speed;
        fi = 0x5f3759df - (fi >> 1);
        invSpeed = *(float*)&fi;
        invSpeed = invSpeed * (1.5f - xhalf * invSpeed * invSpeed);
    }
    float dirX = velX * invSpeed;
    float dirZ = velZ * invSpeed;

    for (int i = 1; i <= TGRID_LOOKAHEAD; i++) {
        float fx = px + dirX * (float)i * TGRID_CELL_SIZE;
        float fz = pz + dirZ * (float)i * TGRID_CELL_SIZE;
        // Mark footprint at lookahead position too
        int gxA = TGridX(fx);
        int gzA = TGridZ(fz);
        for (int ddx = -halfFp; ddx <= halfFp; ddx++) {
            for (int ddz = -halfFp; ddz <= halfFp; ddz++) {
                int idx = TGridIdx(gxA + ddx, gzA + ddz);
                g_TGrid[idx].tick = t;
                g_TGrid[idx].unitId = uid;
            }
        }
    }
}

// ==========================================================================
// Query: find the best lateral offset to avoid occupied cells
//
// Samples cells to the left and right of the travel direction.
// Returns which side has fewer occupied cells (and how much offset).
// ==========================================================================

float TGrid_QueryOffset(float px, float pz, float fwdX, float fwdZ,
                        uint32_t myUnitId, uint32_t tick) {
    uint16_t t = (uint16_t)(tick & 0xFFFF);
    uint16_t myUid = (uint16_t)(myUnitId & 0xFFFF);
    if (myUid == 0) myUid = 1;

    // Perpendicular: right = (fwdZ, -fwdX)
    float rightX =  fwdZ;
    float rightZ = -fwdX;

    // Sample 3 positions ahead, at offsets -2, -1, 0, +1, +2 laterally
    int leftScore = 0;
    int rightScore = 0;
    int centerScore = 0;

    for (int ahead = 1; ahead <= 3; ahead++) {
        float ax = px + fwdX * (float)ahead * TGRID_CELL_SIZE;
        float az = pz + fwdZ * (float)ahead * TGRID_CELL_SIZE;

        // Center
        {
            int idx = TGridIdx(TGridX(ax), TGridZ(az));
            uint16_t dt = t - g_TGrid[idx].tick;
            if (dt < TGRID_DECAY && g_TGrid[idx].unitId != myUid)
                centerScore++;
        }

        // Left (-1, -2)
        for (int lat = 1; lat <= 2; lat++) {
            float lx = ax - rightX * (float)lat * TGRID_CELL_SIZE;
            float lz = az - rightZ * (float)lat * TGRID_CELL_SIZE;
            int idx = TGridIdx(TGridX(lx), TGridZ(lz));
            uint16_t dt = t - g_TGrid[idx].tick;
            if (dt < TGRID_DECAY && g_TGrid[idx].unitId != myUid)
                leftScore++;
        }

        // Right (+1, +2)
        for (int lat = 1; lat <= 2; lat++) {
            float rx = ax + rightX * (float)lat * TGRID_CELL_SIZE;
            float rz = az + rightZ * (float)lat * TGRID_CELL_SIZE;
            int idx = TGridIdx(TGridX(rx), TGridZ(rz));
            uint16_t dt = t - g_TGrid[idx].tick;
            if (dt < TGRID_DECAY && g_TGrid[idx].unitId != myUid)
                rightScore++;
        }
    }

    // No obstacles ahead — no offset needed
    if (centerScore == 0 && leftScore == 0 && rightScore == 0)
        return 0.0f;

    // Only offset if something is actually blocking center/ahead
    if (centerScore == 0) return 0.0f;

    // Steer AWAY from the more blocked side
    // rightScore high = more units on our right = go LEFT
    // leftScore high = more units on our left = go RIGHT
    if (leftScore > rightScore)
        return TGRID_CELL_SIZE;     // Left is more blocked → go right
    else if (rightScore > leftScore)
        return -TGRID_CELL_SIZE;    // Right is more blocked → go left
    else
        // Both sides equal — use unit ID for deterministic tie-break (Rechtsverkehr)
        return (myUnitId & 1) ? TGRID_CELL_SIZE : -TGRID_CELL_SIZE;
}

// ==========================================================================
// Check if a cell is occupied by a moving unit (used by PathCostHook)
// ==========================================================================

bool TGrid_IsCellOccupied(float worldX, float worldZ, uint32_t myUnitId, uint32_t tick) {
    uint16_t t = (uint16_t)(tick & 0xFFFF);
    uint16_t myUid = (uint16_t)(myUnitId & 0xFFFF);
    if (myUid == 0) myUid = 1;

    int idx = TGridIdx(TGridX(worldX), TGridZ(worldZ));
    uint16_t dt = t - g_TGrid[idx].tick;

    // Cell is occupied if recently marked by a DIFFERENT unit
    return (dt < TGRID_DECAY && g_TGrid[idx].unitId != 0 && g_TGrid[idx].unitId != myUid);
}

// ==========================================================================
// Reset grid (called on new match detection)
// ==========================================================================

void TGrid_Reset() {
    memset(g_TGrid, 0, sizeof(g_TGrid));
}
