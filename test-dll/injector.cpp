/**
 * injector.cpp – Injects/Ejects FATestDll.dll into ForgedAlliance.exe (v20)
 *
 * Usage: 
 *   Injector.exe          (Injects)
 *   Injector.exe --eject  (Ejects)
 *
 * v20 Improvements:
 * - Polling wait loop for process start.
 * - Ejection support via FreeLibrary.
 * - Exe-relative DLL path resolution.
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <tlhelp32.h>
#include <cstdio>
#include <cstring>
#include <string>

// ---------------------------------------------------------------------------
// Resolve DLL path relative to the Injector EXE
// ---------------------------------------------------------------------------
static std::string ResolveDllPath(const char* dllName) {
    char exePath[MAX_PATH] = {};
    if (GetModuleFileNameA(NULL, exePath, MAX_PATH)) {
        std::string path(exePath);
        size_t lastSlash = path.find_last_of("\\/");
        if (lastSlash != std::string::npos) {
            std::string dir = path.substr(0, lastSlash + 1);
            std::string full = dir + dllName;
            
            // Check if file exists
            DWORD attr = GetFileAttributesA(full.c_str());
            if (attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY)) {
                return full;
            }
        }
    }
    // Fallback to CWD
    return std::string(dllName);
}

// ---------------------------------------------------------------------------
// Find a process by name, return its PID
// ---------------------------------------------------------------------------
static DWORD FindProcess(const char* exeName) {
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return 0;

    PROCESSENTRY32 pe = {};
    pe.dwSize = sizeof(pe);

    DWORD pid = 0;
    if (Process32First(snap, &pe)) {
        do {
            if (_stricmp(pe.szExeFile, exeName) == 0) {
                pid = pe.th32ProcessID;
                break;
            }
        } while (Process32Next(snap, &pe));
    }

    CloseHandle(snap);
    return pid;
}

static DWORD LaunchProcess(const char* exePath) {
    STARTUPINFOA si = {};
    PROCESS_INFORMATION pi = {};
    si.cb = sizeof(si);

    // Set working directory to the game folder
    char workDir[MAX_PATH] = {};
    strncpy(workDir, exePath, MAX_PATH);
    char* lastSlash = strrchr(workDir, '\\');
    if (lastSlash) *lastSlash = '\0';

    if (!CreateProcessA(
            exePath,    // Application path
            nullptr,    // Command line
            nullptr,    // Process security
            nullptr,    // Thread security
            FALSE,      // Inherit handles
            0,          // Creation flags
            nullptr,    // Environment
            workDir,    // Working directory = game folder
            &si, &pi))
    {
        printf("[!] CreateProcessA failed: %lu\n", GetLastError());
        printf("    Path: %s\n", exePath);
        return 0;
    }

    printf("[*] Launched PID: %lu\n", pi.dwProcessId);
    CloseHandle(pi.hThread);
    CloseHandle(pi.hProcess);
    return pi.dwProcessId;
}

// ---------------------------------------------------------------------------
// Inject a DLL into a process
// ---------------------------------------------------------------------------
static bool InjectDll(DWORD pid, const char* dllPath) {
    printf("[*] Injecting: %s\n", dllPath);

    HANDLE hProc = OpenProcess(
        PROCESS_CREATE_THREAD | PROCESS_VM_OPERATION |
        PROCESS_VM_WRITE | PROCESS_VM_READ | PROCESS_QUERY_INFORMATION,
        FALSE, pid);
    if (!hProc) {
        printf("[!] OpenProcess failed: %lu\n", GetLastError());
        return false;
    }

    SIZE_T pathLen = strlen(dllPath) + 1;
    LPVOID remoteMem = VirtualAllocEx(hProc, nullptr, pathLen, MEM_COMMIT | MEM_RESERVE, PAGE_READWRITE);
    if (!remoteMem) {
        printf("[!] VirtualAllocEx failed: %lu\n", GetLastError());
        CloseHandle(hProc);
        return false;
    }

    if (!WriteProcessMemory(hProc, remoteMem, dllPath, pathLen, nullptr)) {
        printf("[!] WriteProcessMemory failed: %lu\n", GetLastError());
        VirtualFreeEx(hProc, remoteMem, 0, MEM_RELEASE);
        CloseHandle(hProc);
        return false;
    }

    LPVOID loadLib = (LPVOID)GetProcAddress(GetModuleHandleA("kernel32.dll"), "LoadLibraryA");
    HANDLE hThread = CreateRemoteThread(hProc, nullptr, 0, (LPTHREAD_START_ROUTINE)loadLib, remoteMem, 0, nullptr);

    if (!hThread) {
        printf("[!] CreateRemoteThread failed: %lu\n", GetLastError());
        VirtualFreeEx(hProc, remoteMem, 0, MEM_RELEASE);
        CloseHandle(hProc);
        return false;
    }

    WaitForSingleObject(hThread, INFINITE);
    DWORD exitCode = 0;
    GetExitCodeThread(hThread, &exitCode);
    
    CloseHandle(hThread);
    VirtualFreeEx(hProc, remoteMem, 0, MEM_RELEASE);
    CloseHandle(hProc);

    if (exitCode != 0) {
        printf("[+] LoadLibraryA returned 0x%08lX (Success)\n", exitCode);
        printf("[!] Note: If the DLL was already loaded, DllMain will NOT be called again.\n");
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Eject a DLL from a process
// ---------------------------------------------------------------------------
static bool EjectDll(DWORD pid, const char* dllName) {
    printf("[*] Attempting to eject: %s\n", dllName);

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, pid);
    if (snap == INVALID_HANDLE_VALUE) return false;

    MODULEENTRY32 me = {};
    me.dwSize = sizeof(me);
    HMODULE hTargetMod = NULL;

    if (Module32First(snap, &me)) {
        do {
            if (_stricmp(me.szModule, dllName) == 0) {
                hTargetMod = me.hModule;
                break;
            }
        } while (Module32Next(snap, &me));
    }
    CloseHandle(snap);

    if (!hTargetMod) {
        printf("[!] DLL not found in target process.\n");
        return false;
    }

    HANDLE hProc = OpenProcess(
        PROCESS_CREATE_THREAD | PROCESS_VM_OPERATION |
        PROCESS_VM_WRITE | PROCESS_VM_READ | PROCESS_QUERY_INFORMATION,
        FALSE, pid);
    if (!hProc) return false;

    LPVOID freeLib = (LPVOID)GetProcAddress(GetModuleHandleA("kernel32.dll"), "FreeLibrary");
    HANDLE hThread = CreateRemoteThread(hProc, nullptr, 0, (LPTHREAD_START_ROUTINE)freeLib, (LPVOID)hTargetMod, 0, nullptr);

    if (hThread) {
        WaitForSingleObject(hThread, INFINITE);
        CloseHandle(hThread);
        CloseHandle(hProc);
        printf("[+] Ejection script triggered.\n");
        return true;
    }

    CloseHandle(hProc);
    return false;
}

int main(int argc, char* argv[]) {
    const char* targetName = "ForgedAlliance.exe";
    const char* dllName    = "FATestDll.dll";
    bool ejectMode = false;

    for (int i=1; i<argc; ++i) {
        if (strcmp(argv[i], "--eject") == 0) ejectMode = true;
    }

    printf("=== FA Engine v20 Injector ===\n");

    const char* gamePath = "C:\\ProgramData\\FAForever\\bin\\ForgedAlliance.exe";

    DWORD pid = FindProcess(targetName);
    if (pid) {
        printf("[*] Game already running (PID: %lu), skipping launch.\n", pid);
    } else {
        printf("[*] Launching %s...\n", gamePath);
        pid = LaunchProcess(gamePath);
        if (!pid) {
            printf("[!] Failed to launch game.\n");
            getchar();
            return 1;
        }
        // Wait for the process to finish initializing before injecting
        printf("[*] Waiting for game to initialize");
        for (int i = 0; i < 15; ++i) {
            Sleep(200);
            printf(".");
            fflush(stdout);
            // Confirm PID is still alive
            HANDLE hCheck = OpenProcess(PROCESS_QUERY_INFORMATION, FALSE, pid);
            if (hCheck) { CloseHandle(hCheck); }
            else { printf("\n[!] Process exited early.\n"); return 1; }
        }
        printf("\n");
    }

    if (ejectMode) {
        EjectDll(pid, dllName);
    } else {
        std::string fullPath = ResolveDllPath(dllName);
        InjectDll(pid, fullPath.c_str());
    }

    printf("[*] Finished. Press any key.\n");
    getchar();
    return 0;
}
