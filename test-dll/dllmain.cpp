#include <windows.h>
#include <cstdio>
#include <cstdarg>
#include <share.h>
#include <cstdint>

/**
 * dllmain.cpp - DLL Entry Point and Core Hook Infrastructure
 * 
 * This file handles the DLL lifecycle, provides the logging system,
 * and maintains the persistent log file handle to prevent I/O bottlenecks.
 */

// Global Log File Handle
static FILE* g_LogFile = nullptr;

/**
 * Standardized logging function.
 * Prepends messages to FATest.log in the game directory.
 */
extern "C" void Log(const char* fmt, ...) {
    // Fallback to DebugView/Sysinternals
    char buffer[1024];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    OutputDebugStringA(buffer);

    if (!g_LogFile) return;
    
    fputs(buffer, g_LogFile);
    
    // Flush immediately to ensure messages are visible even if the game crashes
    fflush(g_LogFile);
}

// External hook installation from collision.cpp (Declared with C linkage)
extern "C" void InstallSteeringHook();

static DWORD WINAPI DebugToggleThread(LPVOID) {
    bool lastState = false;
    while (true) {
        bool shiftHeld = (GetAsyncKeyState(VK_SHIFT) & 0x8000) != 0;
        bool f9Pressed = (GetAsyncKeyState(VK_F9) & 0x8000) != 0;
        bool pressed = shiftHeld && f9Pressed;
        if (pressed && !lastState) {
            uint8_t* pFlag = (uint8_t*)0x010A6395;
            *pFlag = (*pFlag == 0) ? 1 : 0;
            // Log the new state (use your existing Log function)
            Log("AI_DebugCollision toggled: %s\n", 
                *pFlag ? "ON" : "OFF");
        }
        lastState = pressed;
        Sleep(50);
    }
    return 0;
}

/**
 * Initializes the DLL, opens the log, and installs engine hooks.
 */
void Init() {
    // Attempt to create the logs directory just in case
    CreateDirectoryA("logs", NULL);

    // Open log file in the user's preferred relative folder
    g_LogFile = _fsopen("logs\\FATest.log", "w", _SH_DENYNO);
    
    // Fallback if the logs folder doesn't exist or is inaccessible
    if (!g_LogFile) {
        g_LogFile = _fsopen("FATest.log", "w", _SH_DENYNO);
    }

    if (g_LogFile) {
        Log("\n[v22 - Per-Unit Steering Hook]\n");
        Log("-----------------------------------------\n");
        
        // Setup engine hooks
        InstallSteeringHook();
        
        CreateThread(nullptr, 0, DebugToggleThread, nullptr, 0, nullptr);
        
        Log("Initialization Complete.\n\n");
        
        // Alert the user that the DLL is alive
        MessageBoxA(NULL, "FATestDll.dll v22 LOADED\n\nCHECK GAME FOLDER FOR FATest.log", "FA Hook", MB_OK | MB_ICONINFORMATION | MB_TOPMOST);
    } else {
        // If even the fallback failed, log to debug output only
        OutputDebugStringA("FATestDll: Failed to open log file at 'logs\\FATest.log' or 'FATest.log'. Logging to DebugView only.\n");
        MessageBoxA(NULL, "FATestDll.dll v22 LOADED\n\nWARNING: Failed to open log file. Check DebugView for output.", "FA Hook - Log Error", MB_OK | MB_ICONWARNING | MB_TOPMOST);
    }
}

/**
 * DLL Entry Point
 */
BOOL APIENTRY DllMain(HMODULE hModule, DWORD  ul_reason_for_call, LPVOID lpReserved) {
    switch (ul_reason_for_call) {
    case DLL_PROCESS_ATTACH:
        DisableThreadLibraryCalls(hModule);
        Init();
        break;
        
    case DLL_PROCESS_DETACH:
        if (g_LogFile) {
            Log("DLL Detached. Closing log.\n");
            fclose(g_LogFile);
        }
        break;
    }
    return TRUE;
}
