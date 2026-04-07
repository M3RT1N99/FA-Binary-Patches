// ==========================================================================
// PathfinderOccupancy.h — Shared declarations for the per-tick spatial
// occupancy hash. Defined in section/PathfinderOccupancy.cpp, consumed
// by section/PathfinderTuning.cpp (in ComputeHeuristicC).
// ==========================================================================
#pragma once

#include <stdint.h>

// Spatial hash dimensions
//   - 4096 buckets × 4 bytes = 16 KB
//   - bucket value packs (sim_tick << 8) | unit_count
//   - bucket coordinate = world_unit / OCC_BUCKET_SIZE
#define OCC_HASH_BITS    12
#define OCC_HASH_SIZE   (1u << OCC_HASH_BITS)
#define OCC_HASH_MASK   (OCC_HASH_SIZE - 1u)
#define OCC_BUCKET_SIZE 4

extern uint32_t gPathOccupancyHash[OCC_HASH_SIZE];

// Path reservation hash — distinct from occupancy. Updated when a unit's
// pathfind is accepted (CAiPathFinder::OnPathAccepted hook). Each cell on
// the accepted path bumps a bucket count tagged with an expiration tick
// (current tick + OCC_RESERVATION_LIFETIME). Subsequent A* searches see
// these reservations and add penalty, so units pathing toward the same
// area after another unit just claimed a route diverge naturally.
//
// Bucket pack: (expirationTick << 8) | count, count saturates at 255.
//
// This is "Cooperative A*" / Silver 2005 reservation, simplified: no
// per-cell timing — all cells in a path get the same expiration.
extern uint32_t gPathReservationHash[OCC_HASH_SIZE];

#define OCC_RESERVATION_LIFETIME  60u   // ticks (~6s @ sim 10)

// Pure deterministic hash on coarse cell coordinates.
// Defined inline so both writer (occupancy) and reader (heuristic) get
// the SAME bit pattern without depending on translation-unit ordering.
static inline uint32_t PathOcc_HashCell(int cx, int cz)
{
    uint32_t h = (uint32_t)cx * 0x9E3779B1u;
    h ^= (uint32_t)cz * 0x85EBCA6Bu;
    h ^= h >> 16;
    h *= 0x7FEB352Du;
    h ^= h >> 15;
    h *= 0x846CA68Bu;
    h ^= h >> 16;
    return h & OCC_HASH_MASK;
}
