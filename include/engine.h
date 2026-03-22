#pragma once

/**
 * engine.h – ForgedAlliance.exe "Engine as Library"
 *
 * All addresses verified against ForgedAlliance.exe v3599 loaded at 0x00400000.
 * MohoEngine.dll does NOT exist as a separate DLL – all code is embedded in the EXE.
 *
 * Usage:
 *   #include "engine.h"
 *   g_CWldSession->TogglePause(0);
 *   WLD_SetupSessionInfo(info);
 */

#include "global.h"
#include "moho.h"

// ---------------------------------------------------------------------------
// Core vtable call macro
// VCALL(obj, byte_offset, ReturnType [, arg_types...])
// ---------------------------------------------------------------------------
#define VCALL(obj, offset, ret, ...) \
    ((ret(__thiscall*)(decltype(obj), ##__VA_ARGS__)) \
     (((void**)*(void**)(obj))[(offset)/4]))(obj, ##__VA_ARGS__)

// ---------------------------------------------------------------------------
// CWldSession – key methods
// All addresses verified in Ghidra @ ForgedAlliance.exe
// ---------------------------------------------------------------------------

// Toggle pause (SP/MP) – 0x008942B0
inline void CWldSession_TogglePause(CWldSession* s, int reason)
{
    typedef void(__thiscall* Fn)(CWldSession*, int);
    reinterpret_cast<Fn>(0x8942B0)(s, reason);
}

// Request pause (Lua entry point) – 0x00897800
inline void CWldSession_RequestPause(CWldSession* s)
{
    typedef void(__thiscall* Fn)(CWldSession*);
    reinterpret_cast<Fn>(0x897800)(s);
}

// Resume session – 0x00894330
inline void CWldSession_Resume(CWldSession* s)
{
    typedef void(__thiscall* Fn)(CWldSession*);
    reinterpret_cast<Fn>(0x894330)(s);
}

// Lua_SessionIsPaused – 0x008979A0
inline bool CWldSession_IsPaused(CWldSession* s)
{
    typedef bool(__thiscall* Fn)(CWldSession*);
    return reinterpret_cast<Fn>(0x8979A0)(s);
}

// ---------------------------------------------------------------------------
// WLD Session API
// All confirmed in Ghidra @ ForgedAlliance.exe
// ---------------------------------------------------------------------------

// Setup session info (call before BeginSession) – 0x0088BBB0
inline void WLD_SetupSessionInfo(SWldSessionInfo& info)
{
    typedef void(*Fn)(SWldSessionInfo&);
    reinterpret_cast<Fn>(0x88BBB0)(info);
}

// ---------------------------------------------------------------------------
// Sim – key thread functions (EXE addresses confirmed in Ghidra)
// ---------------------------------------------------------------------------

// Sim beat (called each game tick) – 0x00749F40
inline void Sim_SimBeat(void* simDriver)
{
    typedef void(*Fn)(void*);
    reinterpret_cast<Fn>(0x749F40)(simDriver);
}

// Sim sync (sends checksum over network) – 0x0073DAD0
inline void Sim_SimSync(void* param)
{
    typedef void(*Fn)(void*);
    reinterpret_cast<Fn>(0x73DAD0)(param);
}

// Sim set command source – 0x00748650
inline void Sim_SetCommandSource(int source)
{
    typedef void(*Fn)(int);
    reinterpret_cast<Fn>(0x748650)(source);
}

// Sim verify checksum – 0x007487C0
inline void Sim_VerifyChecksum()
{
    typedef void(*Fn)();
    reinterpret_cast<Fn>(0x7487C0)();
}

// Sim log printf – 0x00746280
inline void Sim_LogPrintf(const char* fmt, ...)
{
    // Note: variadic – use with simple strings only
    typedef void(*Fn)(const char*);
    reinterpret_cast<Fn>(0x746280)(fmt);
}

// ---------------------------------------------------------------------------
// CLobby – key methods (ALL confirmed in Ghidra)
// ---------------------------------------------------------------------------

// Launch game – 0x007C38C0
inline void CLobby_LaunchGame(void* lobby)
{
    typedef void(__thiscall* Fn)(void*);
    reinterpret_cast<Fn>(0x7C38C0)(lobby);
}

// Eject a peer – 0x007C7AC0
inline void CLobby_EjectPeer(void* lobby, void* peer)
{
    typedef void(__thiscall* Fn)(void*, void*);
    reinterpret_cast<Fn>(0x7C7AC0)(lobby, peer);
}

// Send system message to UI – 0x007C7FC0
inline void CLobby_SendSystemMessage(void* lobby, const char* msg)
{
    typedef void(__thiscall* Fn)(void*, const char*);
    reinterpret_cast<Fn>(0x7C7FC0)(lobby, msg);
}

// On connection made – 0x007C5CA0
inline void CLobby_OnConnectionMade(void* lobby, void* connection)
{
    typedef void(__thiscall* Fn)(void*, void*);
    reinterpret_cast<Fn>(0x7C5CA0)(lobby, connection);
}

// On peer disconnect – 0x007C5ED0
inline void CLobby_OnPeerDisconnect(void* lobby, void* peer)
{
    typedef void(__thiscall* Fn)(void*, void*);
    reinterpret_cast<Fn>(0x7C5ED0)(lobby, peer);
}

// ---------------------------------------------------------------------------
// Replay API (confirmed in Ghidra)
// ---------------------------------------------------------------------------

// Launch replay session – 0x008765E0
inline void Lua_LaunchReplaySession(void* luaState)
{
    typedef void(*Fn)(void*);
    reinterpret_cast<Fn>(0x8765E0)(luaState);
}

// SaveGame single player – 0x008807F0
inline void CSaveGame_CreateSinglePlayerSession(void* saveGame)
{
    typedef void(*Fn)(void*);
    reinterpret_cast<Fn>(0x8807F0)(saveGame);
}

// ---------------------------------------------------------------------------
// NOTE: The following APIs from engine-internals.md were listed with
// 0x104xxxxx addresses. These are WRONG – MohoEngine.dll is not a
// separate DLL. The correct EXE addresses need to be found via Ghidra.
//
// TODO: Find correct EXE addresses for:
//   - WLD_IsSessionActive
//   - WLD_GetSession
//   - WLD_BeginSession
//   - WLD_RequestEndSession
//   - WLD_Teardown
//   - Sim::Get() singleton
//   - Sim::GetCurrentTick()
//   - Sim::GetArmyCount()
//   - UI_StartGame / UI_StartFrontEnd
//   - CON_Execute
//   - THREAD_IsMainThread
// ---------------------------------------------------------------------------
