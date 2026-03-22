/**
 * ExampleEngineHook.cpp
 *
 * Demonstrates how to use include/engine.h to write new hooks without
 * raw addresses or inline-ASM – just clean C++.
 *
 * This example adds a new Lua function "GetSessionInfo" that returns
 * detailed session state info (pause, beat, focus army, etc.) as a table.
 */

#include "../define.h"
#include "../include/engine.h"

// ---------------------------------------------------------------------------
// Lua function: SessionGetInfo()
// Returns: table {
//   beat         = int,
//   tick         = int,
//   isMultiplayer = bool,
//   isReplay     = bool,
//   paused       = bool,
//   focusArmy    = int,
//   armyCount    = int,
//   simRate      = float,
// }
// ---------------------------------------------------------------------------
static int Lua_SessionGetInfo(lua_State* L)
{
    CWldSession* session = g_CWldSession;
    if (!session) {
        lua_pushnil(L);
        return 1;
    }

    Sim* sim = g_Sim;
    if (!sim) {
        lua_pushnil(L);
        return 1;
    }

    lua_newtable(L);

    // Beat counter
    lua_pushstring(L, "beat");
    lua_pushinteger(L, sim->beatCounter);
    lua_settable(L, -3);

    // Tick number
    lua_pushstring(L, "tick");
    lua_pushinteger(L, sim->tickNumber);
    lua_settable(L, -3);

    // Multiplayer flag
    lua_pushstring(L, "isMultiplayer");
    lua_pushboolean(L, session->isMultiplayer);
    lua_settable(L, -3);

    // Replay flag
    lua_pushstring(L, "isReplay");
    lua_pushboolean(L, session->isReplay);
    lua_settable(L, -3);

    // Paused (SP or MP)
    lua_pushstring(L, "paused");
    lua_pushboolean(L, session->IsPaused());
    lua_settable(L, -3);

    // Focus army
    lua_pushstring(L, "focusArmy");
    lua_pushinteger(L, session->focusArmy);
    lua_settable(L, -3);

    // Army count from Sim
    lua_pushstring(L, "armyCount");
    lua_pushinteger(L, sim->GetArmyCount());
    lua_settable(L, -3);

    // Cheats enabled
    lua_pushstring(L, "cheatsEnabled");
    lua_pushboolean(L, session->cheatsEnabled);
    lua_settable(L, -3);

    return 1;
}

// ---------------------------------------------------------------------------
// Hook registration
// Uses standard RegisterLuaCFunction mechanism (see global.h)
// ---------------------------------------------------------------------------
asm(
    ".section h0; .set h0, 0x4CD3A0;"
    "push offset Lua_SessionGetInfo;"
    "push offset s_SessionGetInfo;"
    "push offset s_global;"
    "call *0x0;"   // RegisterLuaCFunction – replace 0x0 with actual hook point
    "add esp, 0xC;"
    "ret;"
);

// NOTE: For actual use, see other hooks in hooks/ for the correct asm pattern.
// This is a demonstration of the C++ side only.
