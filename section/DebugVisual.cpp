// ==========================================================================
// DebugVisual.cpp — Visual debug hotkeys
//
//   Shift+1: Toggle ren_Steering (shows steering vectors)
//   Shift+2: Toggle NavWaypoints + NavPath + NavSteering debug
// ==========================================================================

#include "CObject.h"
#include "magic_classes.h"
#include "moho.h"
#include "global.h"

static void (*CON_Execute)(const char*) = (void(*)(const char*))0x41CC90;

static bool* g_pRenSteering = (bool*)0x10A6395;
static bool g_NavDebugOn = false;

typedef short (__stdcall *fn_GetAsyncKeyState)(int vKey);
static fn_GetAsyncKeyState g_GetAsyncKeyState = nullptr;

static bool IsShiftHeld() {
    if (!g_GetAsyncKeyState) {
        void* user32 = GetModuleHandleA("user32.dll");
        if (user32)
            g_GetAsyncKeyState = (fn_GetAsyncKeyState)GetProcAddress(user32, "GetAsyncKeyState");
    }
    if (g_GetAsyncKeyState)
        return g_GetAsyncKeyState(0x10) < 0; // VK_SHIFT
    return false;
}

void DebugKeyDown(unsigned int vkCode) {
    if (vkCode == 0x31) { // Shift+1: steering vectors
        *g_pRenSteering = !(*g_pRenSteering);
        LogF("[DBG] ren_Steering = %d\n", *g_pRenSteering);
    }

    if (vkCode == 0x32) { // Shift+2: nav debug
        g_NavDebugOn = !g_NavDebugOn;
        CON_Execute("dbg NavWaypoints");
        CON_Execute("dbg NavPath");
        CON_Execute("dbg NavSteering");
        LogF("[DBG] Nav debug = %d\n", g_NavDebugOn);
    }
}
