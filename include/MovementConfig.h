#pragma once

// ==========================================================================
// MovementConfig.h — All constants for the Movement System
// Offsets verified against IDA decompile + SupComDecomp source + moho.h
// ==========================================================================

// Spatial Hash (sized to fit in 512KB section)
// 1024 buckets * 8 entries * 20B * 2 buffers = ~320KB
#define SPATIAL_CELL_SIZE       10.0f
#define SPATIAL_BUCKETS         1024
#define SPATIAL_MAX_PER_BUCKET  8

// RVO2 (Phase 2)
#define RVO_TIME_HORIZON        2.5f
#define RVO_NEIGHBOR_RADIUS     12.0f
#define RVO_MAX_NEIGHBORS       32

// Lane Separation
#define LANE_COUNT              3       // Lanes per direction group
#define LANE_BASE_WIDTH         1.2f    // Base lane width in unit sizes
#define LANE_SPREAD_FACTOR      3.0f    // Spread factor on oncoming traffic
#define LANE_SPREAD_LERP        0.12f   // Lerp per tick (0-1)
#define LANE_CONTRACT_LERP      0.04f   // Contract lerp per tick
#define LANE_ONCOMING_DOT       -0.5f   // Dot threshold for oncoming detection

// ── Engine offsets (IDA-verified) ──────────────────────────────────────

// CAiSteeringImpl_CTask (this in OnTick)
#define OFF_STEERING_UNIT       0x1C    // -> CUnit* (direct, NOT via navigator)

// Unit (from moho.h + IDA)
#define OFF_UNIT_ID             0x70    // uint32_t UnitID
#define OFF_UNIT_BLUEPRINT      0x74    // RUnitBlueprint*
#define OFF_UNIT_SIM            0x150   // Sim*
#define OFF_UNIT_POS            0x160   // Vector3f Pos3 (current world position)
#define OFF_UNIT_MOTION         0x4B0   // CUnitMotion*
#define OFF_UNIT_NAVIGATOR      0x4B4   // CAiNavigatorImpl*

// Sim
#define OFF_SIM_CURTICK         0x900   // uint32_t mCurTick

// Blueprint -> Physics (Blueprint+0x278 = RUnitBlueprintPhysics start)
#define OFF_BP_PHYSICS          0x278   // Physics sub-struct offset in blueprint
#define OFF_PHYS_MAXSPEED       0x28    // float MaxSpeed (relative to Physics start)
#define OFF_PHYS_MAXACCEL       0x30    // float MaxAcceleration
#define OFF_PHYS_MAXSTEER       0x38    // float MaxSteerForce
// Absolute: Blueprint + OFF_BP_PHYSICS + OFF_PHYS_MAXSPEED = Blueprint + 0x2A0

// Blueprint sizes (used for collision radius)
#define OFF_BP_SIZEX            0xAC    // float SizeX
#define OFF_BP_SIZEZ            0xB4    // float SizeZ

// Capacity
#define MAX_TRACKED_UNITS       512

// ==========================================================================
// Shared pointer validation
// ==========================================================================

static inline bool IsValidPtr(uint32_t ptr) {
    return (ptr >= 0x00400000 && ptr < 0x3F000000 && (ptr & 3) == 0);
}

// ==========================================================================
// Shared struct for spatial hash entries (used by MovementSystem, RVO2, Lane)
// ==========================================================================

struct SpatialEntry {
    float x, z;             // World position
    uint32_t unitAddr;      // Unit pointer (for identity check)
    uint32_t unitId;        // Unit ID (for deterministic sorting)
    float size;             // Unit footprint size
};
