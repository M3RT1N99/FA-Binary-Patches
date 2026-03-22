# FA Engine Test DLL

Tests `engine.h` at runtime by injecting into a running `ForgedAlliance.exe`.

## Build

### Prerequisites: MSYS2 + MinGW32

Install MSYS2 (if not already):
```
winget install MSYS2.MSYS2
```
Then in the MSYS2 MinGW32 shell:
```
pacman -S mingw-w64-i686-gcc
```

### Compile

**Option A – Just double-click `build.bat`** (auto-detects MSYS2)

**Option B – MSYS2 MinGW32 shell:**
```bash
cd test-dll
make
```

Output: `FATestDll.dll` + `Injector.exe`

## Run

1. Start `ForgedAlliance.exe` (via FAF client or directly)
2. Run `Injector.exe` (as Administrator if injection fails)
3. A MessageBox appears in-game when tests complete
4. Check `FATest.log` in the game directory

## What gets tested

| Test | What it checks |
|------|----------------|
| Global pointers | `g_CWldSession`, `g_Sim` addresses readable |
| Session fields | `isMultiplayer`, `isReplay`, `pauseActive`, `focusArmy` offsets correct |
| Sim fields | `beatCounter`, `tickNumber`, `commandSource`, `luaState` offsets correct |
| WLD API | `WLD_IsSessionActive()`, `WLD_GetSession()` callable |

## Files

| File | Purpose |
|------|---------|
| `dllmain.cpp` | Test DLL – runs tests, writes `FATest.log` |
| `injector.cpp` | Injector EXE – finds FA process, injects DLL |
| `Makefile` | MinGW32 build rules |
| `build.bat` | Windows batch build script |

## Troubleshooting

- **Injection fails** → Run `Injector.exe` as Administrator
- **FATest.log is empty** → DLL crashed. Check that `FATestDll.dll` is 32-bit: `file FATestDll.dll` should say `PE32`
- **0x00000000 addresses** → Normal when injecting at main menu (no session active)
