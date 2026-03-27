// ==========================================================================
// RVO2.cpp — Simplified ORCA Velocity Obstacles
//
// Computes a collision-free velocity adjustment using half-plane constraints.
// Each nearby neighbor creates an ORCA half-plane; we find the closest
// velocity to the preferred velocity that satisfies all constraints.
//
// Simplified for FA: 2D only (XZ plane), no dynamic obstacles, uses
// SpatialHash neighbors passed from MovementSystem.cpp.
// ==========================================================================

#include "CObject.h"
#include "moho.h"
#include "global.h"
#include "MovementConfig.h"

// SpatialEntry defined in MovementConfig.h

// ==========================================================================
// ORCA Half-Plane
// ==========================================================================

struct OrcaLine {
    float pointX, pointZ;       // Point on the half-plane boundary
    float dirX, dirZ;           // Direction of the boundary (unit vector)
};

#define ORCA_MAX_LINES  32
#define RVO_EPSILON     0.00001f
#define RVO_MAX_SPEED_MULT  1.2f  // Allow slightly over max speed for avoidance

// Debug counters
static int g_RvoDbgCalls     = 0;
static int g_RvoDbgAdjusted  = 0;
static float g_RvoDbgMaxAdj  = 0.0f;

// ==========================================================================
// 2D cross product (determinant)
// ==========================================================================

static float Det2D(float ax, float az, float bx, float bz) {
    return ax * bz - az * bx;
}

// ==========================================================================
// Project velocity onto ORCA half-plane (linearProgram1)
//
// Projects the velocity onto a single ORCA line while staying on the
// permitted side of all previously processed lines.
// ==========================================================================

static bool ProjectOnLine(OrcaLine* lines, int lineNo,
                          float radius, float maxSpeed,
                          float* velX, float* velZ) {
    OrcaLine* line = &lines[lineNo];

    float dotProduct = line->pointX * line->dirX + line->pointZ * line->dirZ;
    float discriminant = dotProduct * dotProduct + maxSpeed * maxSpeed
                       - (line->pointX * line->pointX + line->pointZ * line->pointZ);

    if (discriminant < 0.0f) return false;

    float sqrtDisc = 0.0f;
    {
        // Fast sqrt via inverse sqrt
        if (discriminant > RVO_EPSILON) {
            float xhalf = 0.5f * discriminant;
            int fi = *(int*)&discriminant;
            fi = 0x5f3759df - (fi >> 1);
            float invSqrt = *(float*)&fi;
            invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
            sqrtDisc = discriminant * invSqrt;
        }
    }

    float tLeft  = -dotProduct - sqrtDisc;
    float tRight = -dotProduct + sqrtDisc;

    // Constrain t range against previous lines
    for (int i = 0; i < lineNo; i++) {
        float denom = Det2D(line->dirX, line->dirZ, lines[i].dirX, lines[i].dirZ);
        float numer = Det2D(lines[i].dirX, lines[i].dirZ,
                           line->pointX - lines[i].pointX,
                           line->pointZ - lines[i].pointZ);

        if (denom * denom <= RVO_EPSILON) {
            // Lines are nearly parallel
            if (numer < 0.0f) return false;
            continue;
        }

        float t = numer / denom;
        if (denom > 0.0f) {
            if (t < tRight) tRight = t;
        } else {
            if (t > tLeft) tLeft = t;
        }

        if (tLeft > tRight) return false;
    }

    // Project current velocity onto line
    float t = line->dirX * (*velX - line->pointX) + line->dirZ * (*velZ - line->pointZ);

    // Clamp t to valid range
    if (t < tLeft) t = tLeft;
    if (t > tRight) t = tRight;

    *velX = line->pointX + t * line->dirX;
    *velZ = line->pointZ + t * line->dirZ;
    return true;
}

// ==========================================================================
// ComputeRVOVelocity — main entry point
//
// Given current position, preferred velocity, and neighbors from SpatialHash,
// computes a collision-free velocity adjustment.
//
// Parameters:
//   myX, myZ        - Current position
//   prefVelX/Z      - Preferred (desired) velocity
//   myRadius        - Unit collision radius
//   maxSpeed        - Maximum speed from blueprint
//   neighbors       - Array of SpatialEntry from HashQuery
//   nbrCount        - Number of neighbors
//   unitAddr        - This unit's address (to skip self)
//   outVelX/Z       - Output: adjusted velocity
// ==========================================================================

extern "C" void ComputeRVOVelocity(
    float myX, float myZ,
    float prefVelX, float prefVelZ,
    float myRadius, float maxSpeed,
    SpatialEntry* neighbors, int nbrCount,
    uint32_t unitAddr,
    float* outVelX, float* outVelZ)
{
    *outVelX = prefVelX;
    *outVelZ = prefVelZ;
    g_RvoDbgCalls++;

    if (nbrCount == 0) return;

    OrcaLine orcaLines[ORCA_MAX_LINES];
    int numLines = 0;
    float invTimeHorizon = 1.0f / RVO_TIME_HORIZON;
    float adjustedMaxSpeed = maxSpeed * RVO_MAX_SPEED_MULT;

    for (int i = 0; i < nbrCount && numLines < ORCA_MAX_LINES; i++) {
        if (neighbors[i].unitAddr == unitAddr) continue;

        // Relative position and velocity
        float relPosX = neighbors[i].x - myX;
        float relPosZ = neighbors[i].z - myZ;

        float distSq = relPosX * relPosX + relPosZ * relPosZ;
        float combinedRadius = myRadius + neighbors[i].size;
        float combinedRadiusSq = combinedRadius * combinedRadius;

        OrcaLine* line = &orcaLines[numLines];

        if (distSq > combinedRadiusSq) {
            // No collision — project onto velocity obstacle boundary

            // Vector from cutoff center to preferred velocity
            float wX = prefVelX - (relPosX * invTimeHorizon);
            float wZ = prefVelZ - (relPosZ * invTimeHorizon);
            float wLenSq = wX * wX + wZ * wZ;

            float dotProduct1 = wX * relPosX + wZ * relPosZ;

            if (dotProduct1 < 0.0f &&
                dotProduct1 * dotProduct1 > combinedRadiusSq * wLenSq) {
                // Project on cutoff circle
                float wLen = 0.0f;
                if (wLenSq > RVO_EPSILON) {
                    float xhalf = 0.5f * wLenSq;
                    int fi = *(int*)&wLenSq;
                    fi = 0x5f3759df - (fi >> 1);
                    float invSqrt = *(float*)&fi;
                    invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
                    wLen = wLenSq * invSqrt;
                }
                if (wLen < RVO_EPSILON) continue;

                float unitWX = wX / wLen;
                float unitWZ = wZ / wLen;

                line->dirX =  unitWZ;
                line->dirZ = -unitWX;

                float u = (combinedRadius * invTimeHorizon - wLen);
                line->pointX = prefVelX + u * unitWX * 0.5f;
                line->pointZ = prefVelZ + u * unitWZ * 0.5f;
            } else {
                // Project on legs
                float dist = 0.0f;
                if (distSq > RVO_EPSILON) {
                    float xhalf = 0.5f * distSq;
                    int fi = *(int*)&distSq;
                    fi = 0x5f3759df - (fi >> 1);
                    float invSqrt = *(float*)&fi;
                    invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
                    dist = distSq * invSqrt;
                }
                if (dist < RVO_EPSILON) continue;

                float leg = distSq - combinedRadiusSq;
                if (leg < 0.0f) leg = 0.0f;
                float legLen = 0.0f;
                if (leg > RVO_EPSILON) {
                    float xhalf = 0.5f * leg;
                    int fi = *(int*)&leg;
                    fi = 0x5f3759df - (fi >> 1);
                    float invSqrt = *(float*)&fi;
                    invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
                    legLen = leg * invSqrt;
                }

                // Determine which side
                float det = Det2D(relPosX, relPosZ, prefVelX, prefVelZ);
                if (det > 0.0f) {
                    // Left leg
                    line->dirX = ( relPosX * legLen + relPosZ * combinedRadius) / distSq;
                    line->dirZ = (-relPosX * combinedRadius + relPosZ * legLen) / distSq;
                } else {
                    // Right leg
                    line->dirX = -(relPosX * legLen - relPosZ * combinedRadius) / distSq;
                    line->dirZ = -( relPosX * combinedRadius + relPosZ * legLen) / distSq;
                }

                float dotProduct2 = relPosX * line->dirX + relPosZ * line->dirZ;
                line->pointX = (prefVelX + (dotProduct2 * invTimeHorizon - prefVelX)) * 0.5f;
                line->pointZ = (prefVelZ + (dotProduct2 * invTimeHorizon - prefVelZ)) * 0.5f;
            }
        } else {
            // Already colliding — push apart
            float dist = 0.0f;
            if (distSq > RVO_EPSILON) {
                float xhalf = 0.5f * distSq;
                int fi = *(int*)&distSq;
                fi = 0x5f3759df - (fi >> 1);
                float invSqrt = *(float*)&fi;
                invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
                dist = distSq * invSqrt;
            }

            // Direction to push (away from neighbor, or arbitrary if on top)
            float pushX, pushZ;
            if (dist > RVO_EPSILON) {
                pushX = relPosX / dist;
                pushZ = relPosZ / dist;
            } else {
                // Units on same spot — use unit ID to break symmetry
                pushX = (unitAddr > neighbors[i].unitAddr) ? 1.0f : -1.0f;
                pushZ = 0.0f;
            }

            line->dirX = -pushZ;
            line->dirZ =  pushX;

            float overlap = combinedRadius - dist;
            // Push half the overlap per tick
            line->pointX = prefVelX + pushX * overlap * 0.5f;
            line->pointZ = prefVelZ + pushZ * overlap * 0.5f;
        }

        numLines++;
    }

    // --- Solve: project preferred velocity onto all ORCA half-planes ---
    float newVelX = prefVelX;
    float newVelZ = prefVelZ;

    for (int i = 0; i < numLines; i++) {
        // Check if current velocity is already on the permitted side
        float det = Det2D(orcaLines[i].dirX, orcaLines[i].dirZ,
                         orcaLines[i].pointX - newVelX,
                         orcaLines[i].pointZ - newVelZ);
        if (det >= 0.0f) continue;  // Already satisfied

        // Project onto this line
        float tempVelX = newVelX;
        float tempVelZ = newVelZ;
        if (!ProjectOnLine(orcaLines, i, 0.0f, adjustedMaxSpeed,
                          &tempVelX, &tempVelZ)) {
            // Projection failed — use best effort from previous lines
            continue;
        }
        newVelX = tempVelX;
        newVelZ = tempVelZ;
    }

    // Clamp to max speed
    float newSpeedSq = newVelX * newVelX + newVelZ * newVelZ;
    if (newSpeedSq > adjustedMaxSpeed * adjustedMaxSpeed) {
        float scale = adjustedMaxSpeed;
        if (newSpeedSq > RVO_EPSILON) {
            float xhalf = 0.5f * newSpeedSq;
            int fi = *(int*)&newSpeedSq;
            fi = 0x5f3759df - (fi >> 1);
            float invSqrt = *(float*)&fi;
            invSqrt = invSqrt * (1.5f - xhalf * invSqrt * invSqrt);
            scale = adjustedMaxSpeed * invSqrt * newSpeedSq;
            // Actually: scale = adjustedMaxSpeed / sqrt(newSpeedSq)
            // invSqrt ~= 1/sqrt(newSpeedSq), so:
            newVelX = newVelX * adjustedMaxSpeed * invSqrt;
            newVelZ = newVelZ * adjustedMaxSpeed * invSqrt;
        }
    } else {
        // No clamping needed but suppress unused variable
        (void)newSpeedSq;
    }

    // Check if we actually adjusted
    float diffX = newVelX - prefVelX;
    float diffZ = newVelZ - prefVelZ;
    float adjMag = diffX * diffX + diffZ * diffZ;
    if (adjMag > 0.01f) {
        g_RvoDbgAdjusted++;
        if (adjMag > g_RvoDbgMaxAdj) g_RvoDbgMaxAdj = adjMag;
    }

    *outVelX = newVelX;
    *outVelZ = newVelZ;
}

// ==========================================================================
// Debug logging (called from MovementSystem periodic log)
// ==========================================================================

extern "C" void RvoDebugLog() {
    LogF("[RVO] calls=%d adjusted=%d maxAdj=%.2f\n",
         g_RvoDbgCalls, g_RvoDbgAdjusted, g_RvoDbgMaxAdj);
    g_RvoDbgCalls = 0;
    g_RvoDbgAdjusted = 0;
    g_RvoDbgMaxAdj = 0.0f;
}
