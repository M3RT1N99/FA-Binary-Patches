# Makefile for FA Engine Test DLL
# Requires MinGW32 (i686-w64-mingw32-g++)
# Build: make   OR   run build.bat

CXX      = i686-w64-mingw32-g++
CXXFLAGS = -m32 -O0 -g -std=c++17 -Wall -I../include -I.
LDFLAGS  = -m32

.PHONY: all clean

all: FATestDll.dll Injector.exe

# -----------------------------------------------------------------------
# The test DLL (32-bit, injected into ForgedAlliance.exe)
# -----------------------------------------------------------------------
FATestDll.dll: dllmain.cpp collision.cpp steering.cpp ../include/engine.h ../include/global.h ../include/moho.h
	$(CXX) $(CXXFLAGS) \
	    -DBUILDING_FA_TEST_DLL \
	    -shared \
	    -o $@ \
	    dllmain.cpp collision.cpp steering.cpp \
	    -lkernel32 -luser32 -lpsapi
	@echo "[OK] Built FATestDll.dll"

# -----------------------------------------------------------------------
# The injector EXE (32-bit standalone)
# -----------------------------------------------------------------------
Injector.exe: injector.cpp
	$(CXX) $(CXXFLAGS) \
	    -o $@ \
	    injector.cpp \
	    -lkernel32
	@echo "[OK] Built Injector.exe"

clean:
	rm -f FATestDll.dll Injector.exe *.o

# -----------------------------------------------------------------------
# Check the output is actually 32-bit
# -----------------------------------------------------------------------
check:
	file FATestDll.dll
	file Injector.exe
