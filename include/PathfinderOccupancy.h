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
