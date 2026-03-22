# Structural Analysis of Forged Alliance (`CUnit`)

Based on the `SupComDecomp` repository reverse engineering effort, we have identified the true structural layouts of `CUnit` (`Moho::Unit`), `Entity`, and `SPhysBody`.

---

## 1. The Mysterious `+0x44` Offset
Our previous assumption that `CUnit+0x44` and `CUnit+0x48` were the horizontal components of a force pseudo-accumulator is **disproven**.

In `src/sim/Entity.h`, the `Entity` class (which `Unit` inherits from) contains `SSTIEntityVariableData mVarDat;`.
Accounting for the base classes `CScriptObject` and `CTask`, `mVarDat` starts exactly at `+0x44`:

```cpp
struct SSTIEntityVariableData
{
    // Offset +0x44 AND +0x48!
    boost::shared_ptr<Moho::RScmResource> mScmResource; 
    
    int mMesh; // +0x4C
    Wm3::Vector3f mScale; // +0x50
    // ...
```

**Conclusion:** `CUnit+0x44` is the raw pointer to `RScmResource` and `CUnit+0x48` is the pointer to its `boost::detail::sp_counted_base` (reference count block). **Writing floats here was corrupting standard C++ smart pointers!**

---

## 2. Physics and Force Accumulation

Forces in Forged Alliance are NOT stored directly flat within the `CUnit` byte layout. Instead, physics properties are grouped via a pointer.

```cpp
class Entity : public CScriptObject, public CTask {
    // ... other fields ...
    Moho::SPhysBody *mPhysBody; // Pointer to actual physics properties
    // ...
};
```

And in `src/sim/PhysBody.h`:
```cpp
struct SPhysBody
{
    float mGravity;              // +0x00
    float mMass;                 // +0x04
    Wm3::Vector3f mInvInertiaTensor; // +0x08
    Wm3::Vector3f mCollisionOffset;  // +0x14
    Wm3::Vector3f mPos;              // +0x20
    Wm3::Quaternionf mOrientation;   // +0x2C
    Wm3::Vector3f mVelocity;         // +0x3C
    Wm3::Vector3f mWorldImpulse;     // +0x48 <-- Potential Transient Force Accumulator!
};
```
If we need a transient force accumulator that gets zeroed/processed every tick, `SPhysBody::mWorldImpulse` (at `mPhysBody + 0x48`) is the mathematically correct target.

---

## 3. The `+0x180` Velocity Reading
During Phase 9, manual diagnostics showed moving non-zero velocity vectors around `CUnit+0x180`/`0x174`.

Because `CUnit` has so many embedded structures (like `SSTIEntityVariableData`, `VTransform` matrices, `SSTIUnitVariableData`), the velocity and position caches are deeply nested flat copies used for interpolation and immediate scripting access (e.g. `mCurTransform` and `mLastTransform`).

Instead of navigating the complex inheritance offsets for temporary vectors, modifying the direct velocity component (`CUnit+0x174` as X or `CUnit+0x180` as speed/forward vector depending on the update phase) relies on the engine's built-in interpolation. 

## 4. `CAiSteeringImpl` and Steering
The `SupComDecomp` repository does not contain the implementation details `CAiSteeringImpl` (or `AIPathNavigatorImpl`). It only contains the interface pointer:
```cpp
Moho::IAiSteering *mSteering;
```
Thus, the steering internals (+0x88, +0x90, +0x98) remain opaque to the decomp repo, confirming that manual disassembly (as we did with `+0x90` being Y-altitude and `+0x8C` being the Vector start) is the only source of truth.
