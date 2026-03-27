// ==========================================================================
// MovementSystem.cpp — Phase 1: SpatialHash + Steering Hook
//
// Hooks into CAiSteeringImpl::OnTick (0x5D32B0) to build a spatial hash
// of all moving units and compute lane offsets for collision avoidance.
//
// Phase 1: LogF only, no engine state writes. Validates that the hook
// fires correctly and the spatial hash returns proper neighbors.
// ==========================================================================

#include "CObject.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"
#include "MovementConfig.h"

// IsValidPtr defined in MovementConfig.h

// Forward declarations (defined in TransientGrid.cpp)
void TGrid_MarkUnit(float px, float pz, float velX, float velZ,
                    uint32_t unitId, uint32_t tick, int footprint);
void TGrid_Reset();
bool TGrid_IsCellOccupied(float worldX, float worldZ, uint32_t myUnitId, uint32_t tick);

// ==========================================================================
// Spatial Hash — fixed-size arrays, no std:: containers
// ==========================================================================

// SpatialEntry defined in MovementConfig.h

struct SpatialBucket {
    SpatialEntry entries[SPATIAL_MAX_PER_BUCKET];
    int count;
};

// Double buffer: write to active, read from inactive (frame N-1)
static SpatialBucket g_HashA[SPATIAL_BUCKETS];
static SpatialBucket g_HashB[SPATIAL_BUCKETS];
static SpatialBucket* g_WriteHash = g_HashA;    // Current frame: insert here
static SpatialBucket* g_ReadHash  = g_HashB;    // Previous frame: query here

static uint32_t g_LastTick    = 0xFFFFFFFF;
static int      g_FrameUnits  = 0;             // Units processed this frame
static bool     g_FirstFrame  = true;
static bool     g_Initialized = false;

// Debug counters (reset each log cycle)
static int g_DbgCalls     = 0;
static int g_DbgMaxNbrs   = 0;
static int g_DbgTotalNbrs = 0;
static int g_DbgSkipped   = 0;

// ── Hash function (deterministic) ──────────────────────────────────────

static int HashPos(float x, float z) {
    int cx = (int)(x / SPATIAL_CELL_SIZE);
    int cz = (int)(z / SPATIAL_CELL_SIZE);
    return (int)(((unsigned int)(cx * 2654435761u) ^ (unsigned int)(cz * 2246822519u))
                 & (SPATIAL_BUCKETS - 1));
}

// ── Insert into write hash ─────────────────────────────────────────────

static void HashInsert(float x, float z, uint32_t unitAddr, uint32_t unitId, float size) {
    int bucket = HashPos(x, z);
    SpatialBucket* b = &g_WriteHash[bucket];
    if (b->count < SPATIAL_MAX_PER_BUCKET) {
        SpatialEntry* e = &b->entries[b->count++];
        e->x = x;
        e->z = z;
        e->unitAddr = unitAddr;
        e->unitId = unitId;
        e->size = size;
    }
}

// ── Query from read hash (previous frame) ──────────────────────────────

static int HashQuery(float x, float z, float radius,
                     SpatialEntry* outResults, int maxResults,
                     uint32_t excludeUnit) {
    int found = 0;
    float radiusSq = radius * radius;
    int r = (int)(radius / SPATIAL_CELL_SIZE) + 1;
    int cx0 = (int)(x / SPATIAL_CELL_SIZE);
    int cz0 = (int)(z / SPATIAL_CELL_SIZE);

    for (int dx = -r; dx <= r && found < maxResults; dx++) {
        for (int dz = -r; dz <= r && found < maxResults; dz++) {
            int bucket = HashPos((float)(cx0 + dx) * SPATIAL_CELL_SIZE,
                                 (float)(cz0 + dz) * SPATIAL_CELL_SIZE);
            SpatialBucket* b = &g_ReadHash[bucket];
            for (int i = 0; i < b->count && found < maxResults; i++) {
                if (b->entries[i].unitAddr == excludeUnit) continue;
                float ex = b->entries[i].x - x;
                float ez = b->entries[i].z - z;
                if (ex * ex + ez * ez <= radiusSq) {
                    outResults[found++] = b->entries[i];
                }
            }
        }
    }
    return found;
}

// ── Public query interface (used by GetTargetPosHook.cpp) ───────────────

int MovSys_QueryNeighbors(float x, float z, float radius,
                          SpatialEntry* outResults, int maxResults,
                          uint32_t excludeUnit) {
    return HashQuery(x, z, radius, outResults, maxResults, excludeUnit);
}

// ── Frame tick detection + buffer swap ──────────────────────────────────

static void CheckNewFrame(uint32_t tick) {
    if (tick == g_LastTick) return;

    // Detect new match: tick jumped backwards or far forward
    if (tick < g_LastTick && g_LastTick != 0xFFFFFFFF) {
        // New match started — full reset
        memset(g_HashA, 0, sizeof(g_HashA));
        memset(g_HashB, 0, sizeof(g_HashB));
        g_WriteHash = g_HashA;
        g_ReadHash  = g_HashB;
        g_FirstFrame = true;
        g_Initialized = false;
        LogF("[MOV] New match detected (tick=%u, lastTick=%u) — reset\n", tick, g_LastTick);
    }

    SpatialBucket* temp = g_ReadHash;
    g_ReadHash  = g_WriteHash;
    g_WriteHash = temp;
    memset(g_WriteHash, 0, sizeof(SpatialBucket) * SPATIAL_BUCKETS);

    g_LastTick   = tick;
    g_FrameUnits = 0;

    if (g_FirstFrame) {
        g_FirstFrame = false;
    }
}

// ==========================================================================
// ASM wrapper — saves registers, calls C function, restores overwritten insns
// Hook overwrites 10 bytes at 0x5D32BD:
//   MOV EAX,[EBX+0x1C]  / PUSH ESI / MOV ESI,[EAX+0x150]
// ==========================================================================

asm(R"(
.globl _OnSteeringTickWrapper
_OnSteeringTickWrapper:
    push eax
    push ecx
    push edx

    push ebx
    call _OnSteeringTick
    add  esp, 4

    pop  edx
    pop  ecx
    pop  eax

    # Restore overwritten instructions
    mov  eax, [ebx+0x1C]
    push esi
    mov  esi, [eax+0x150]

    # Return to 0x5D32C7
    push 0x5D32C7
    ret
)");

// ==========================================================================
// OnSteeringTick — called from ASM wrapper
// ==========================================================================

// Debug: track which check fails
static int g_SkipSteering = 0;
static int g_SkipUnit = 0;
static int g_SkipSim = 0;
static int g_SkipPos = 0;

extern "C" void OnSteeringTick(void* steeringImpl) {
    if (!g_Initialized) {
        g_Initialized = true;
        LogF("[MOV] Hook active! steeringImpl=0x%X\n", (uint32_t)steeringImpl);
    }

    // Validate steering pointer
    if (!IsValidPtr((uint32_t)steeringImpl)) {
        g_SkipSteering++;
        g_DbgSkipped++;
        return;
    }

    // Get CUnit* from steering
    uint32_t unitAddr = *(uint32_t*)((uint8_t*)steeringImpl + OFF_STEERING_UNIT);
    if (!IsValidPtr(unitAddr)) {
        // Log first few bad addresses for debugging
        static int g_BadUnitLog = 0;
        if (g_BadUnitLog < 5) {
            LogF("[MOV] BAD unit ptr=0x%X from steering=0x%X\n", unitAddr, (uint32_t)steeringImpl);
            g_BadUnitLog++;
        }
        g_SkipUnit++;
        g_DbgSkipped++;
        return;
    }
    uint8_t* unit = (uint8_t*)unitAddr;

    // Validate Sim pointer before reading tick
    uint32_t simAddr = *(uint32_t*)(unit + OFF_UNIT_SIM);
    if (!IsValidPtr(simAddr)) {
        g_SkipSim++;
        g_DbgSkipped++;
        return;
    }
    uint32_t tick = *(uint32_t*)(simAddr + OFF_SIM_CURTICK);

    CheckNewFrame(tick);

    // Read unit position
    float px = *(float*)(unit + OFF_UNIT_POS + 0x00);
    float pz = *(float*)(unit + OFF_UNIT_POS + 0x08);
    uint32_t uid = *(uint32_t*)(unit + OFF_UNIT_ID);

    // Sanity check position (NaN or absurd values)
    if (px != px || pz != pz || px < -10000.0f || px > 10000.0f ||
        pz < -10000.0f || pz > 10000.0f) {
        g_SkipPos++;
        g_DbgSkipped++;
        return;
    }

    // Insert into write hash (current frame)
    HashInsert(px, pz, unitAddr, uid, 1.0f);
    g_FrameUnits++;
    g_DbgCalls++;

    // Read footprint size from blueprint (SFootprint at BP+0xD8)
    // SFootprint.SizeX = uchar at +0, SizeZ = uchar at +1
    int footprint = 1;
    uint32_t blueprint = *(uint32_t*)(unit + OFF_UNIT_BLUEPRINT);
    if (IsValidPtr(blueprint)) {
        uint8_t fpSizeX = *(uint8_t*)(blueprint + 0xD8);     // SFootprint.SizeX
        uint8_t fpSizeZ = *(uint8_t*)(blueprint + 0xD8 + 1); // SFootprint.SizeZ
        footprint = (fpSizeX > fpSizeZ) ? fpSizeX : fpSizeZ;
        if (footprint < 1) footprint = 1;
        if (footprint > 6) footprint = 6;
    }

    // Mark transient grid with footprint
    TGrid_MarkUnit(px, pz, 0.0f, 0.0f, uid, tick, footprint);

    // Force repath if on occupied cell — rate-limited + staggered
    uint32_t navAddr = *(uint32_t*)(unit + OFF_UNIT_NAVIGATOR);
    if (IsValidPtr(navAddr) && TGrid_IsCellOccupied(px, pz, uid, tick)) {
        uint32_t* repathCounter = (uint32_t*)(navAddr + 0x9C);
        if (*repathCounter == 0 && ((tick + uid) % 15) == 0) {
            *repathCounter = 1;
        }
    }

    // Query neighbors from read hash (previous frame)
    SpatialEntry neighbors[RVO_MAX_NEIGHBORS];
    int nbrCount = HashQuery(px, pz, RVO_NEIGHBOR_RADIUS,
                             neighbors, RVO_MAX_NEIGHBORS, unitAddr);

    // Track debug stats
    g_DbgTotalNbrs += nbrCount;
    if (nbrCount > g_DbgMaxNbrs) g_DbgMaxNbrs = nbrCount;

    // Log first few per frame for verification
    static int g_LogCount = 0;
    if (g_LogCount < 3 && nbrCount > 0) {
        LogF("[MOV] unit=%u tick=%u nbrs=%d pos=(%.1f,%.1f)\n",
             uid, tick, nbrCount, px, pz);
        g_LogCount++;
    }

    // Periodic summary log (every ~3 seconds at 10Hz tick rate)
    static uint32_t g_LastLogTick = 0;
    if (tick - g_LastLogTick > 30) {
        float avgNbrs = g_DbgCalls > 0 ? (float)g_DbgTotalNbrs / (float)g_DbgCalls : 0.0f;
        LogF("[MOV] tick=%u units=%d avgNbrs=%.1f maxNbrs=%d skip=%d (steer=%d unit=%d sim=%d pos=%d)\n",
             tick, g_FrameUnits, avgNbrs, g_DbgMaxNbrs, g_DbgSkipped,
             g_SkipSteering, g_SkipUnit, g_SkipSim, g_SkipPos);
        g_DbgCalls = 0;
        g_DbgTotalNbrs = 0;
        g_DbgMaxNbrs = 0;
        g_DbgSkipped = 0;
        g_SkipSteering = 0;
        g_SkipUnit = 0;
        g_SkipSim = 0;
        g_SkipPos = 0;
        g_LogCount = 0;
        g_LastLogTick = tick;
    }
}
