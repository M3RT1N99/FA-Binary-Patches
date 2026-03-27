// ==========================================================================
// DebugMovement.cpp — Debug toggle hotkeys for movement system
//
// Called from OnWindowMessage on WM_KEYDOWN:
//   Shift+1 → toggle ren_Steering (nav debug visuals)
//   Shift+2 → toggle dbg NavWaypoints, NavPath, NavSteering
// ==========================================================================

#include "CObject.h"
#include "moho.h"
#include "global.h"

// Moho::CON_Execute(const char*) at 0x41CC90 — executes console command
static void (*CON_Execute)(const char*) = (void(*)(const char*))0x41CC90;

// ren_Steering bool (IDA verified: 0x10A6395)
static bool* g_pRenSteering = (bool*)0x10A6395;

static bool g_NavDebugOn = false;

void DebugMovement_OnKeyDown(unsigned int vkCode, bool shiftHeld) {
    if (!shiftHeld) return;

    // Shift+1 (VK_1 = 0x31) → ren_Steering
    if (vkCode == 0x31) {
        *g_pRenSteering = !(*g_pRenSteering);
        LogF("[DBG] ren_Steering = %d\n", *g_pRenSteering);
    }

    // Shift+2 (VK_2 = 0x32) → NavWaypoints + NavPath + NavSteering
    if (vkCode == 0x32) {
        g_NavDebugOn = !g_NavDebugOn;
        CON_Execute("dbg NavWaypoints");
        CON_Execute("dbg NavPath");
        CON_Execute("dbg NavSteering");
        LogF("[DBG] Nav debug = %d\n", g_NavDebugOn);
    }

    // Shift+3 (VK_3 = 0x33) → Toggle navPersonalPosMaxDistance (50 ↔ 9999)
    if (vkCode == 0x33) {
        extern float navPersonalPosMaxDistance;
        if (navPersonalPosMaxDistance > 100.0f) {
            navPersonalPosMaxDistance = 50.0f;
            LogF("[DBG] navPersonalPosMaxDistance = 50 (columns)\n");
        } else {
            navPersonalPosMaxDistance = 9999.0f;
            LogF("[DBG] navPersonalPosMaxDistance = 9999 (personal)\n");
        }
    }
}
