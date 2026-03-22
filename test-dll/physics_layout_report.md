# WildMagic3 Physics and Vector Layout

Based on the `GeometricTools` library sources provided:

## 1. `Wm3Vector3.h` Layout
The `Vector3` class is extremely simple and perfectly mapped to a contiguous 12-byte float array.

```cpp
template <class Real> // Real is float (4 bytes)
class Vector3 {
private:
    Real m_afTuple[3]; // +0x00: float X, Y, Z (12 bytes)
};
```

## 2. `Wm3RigidBody.h` Layout
Tracing the `RigidBody<float>` class variables, we can determine the exact byte offsets from the start of a `RigidBody` instance.

| Offset (Hex) | Size | Type | Name | Description |
|---|---|---|---|---|
| `+0x00` | 4 | Real | `m_fMass` | Mass |
| `+0x04` | 4 | Real | `m_fInvMass` | Inverse Mass |
| `+0x08` | 36 | Matrix3 | `m_kInertia` | Inertia Matrix |
| `+0x2C` | 36 | Matrix3 | `m_kInvInertia` | Inverse Inertia Matrix |
| `+0x50` | 12 | Vector3 | `m_kPos` | **Position** |
| `+0x5C` | 16 | Quat | `m_kQOrient` | Orientation |
| `+0x6C` | 12 | Vector3 | `m_kLinMom` | Linear Momentum |
| `+0x78` | 12 | Vector3 | `m_kAngMom` | Angular Momentum |
| `+0x84` | 36 | Matrix3 | `m_kROrient` | Orientation Matrix |
| `+0xA8` | 12 | Vector3 | `m_kLinVel` | **Linear Velocity** |
| `+0xB4` | 12 | Vector3 | `m_kAngVel` | **Angular Velocity** |

*(Note: Matrix3 is 9 Reals (36 bytes). Quaternion is 4 Reals (16 bytes).*

## 3. Cross-Referencing CUnit Offsets
Your diagnostic logs showed:
- `CUnit+0x168`: Position (X, Y, Z)
- `CUnit+0x174`: `v1 = (0.0, 1.0, 0.0)`
- `CUnit+0x180`: `v2 = (0.0, 22.0, 0.0)` (looks like speed)

If `CUnit+0x168` correctly maps to the start of the `RigidBody` position (`m_kPos` at `+0x50`), then the `RigidBody` struct would start at `CUnit+0x118`.
Let's see what happens if we apply that `0x118` base to the rest of the layout:

- `0x118 + 0x50` = **`+0x168`**: `m_kPos` (Position. **Matches!**)
- `0x118 + 0x5C` = **`+0x174`**: `m_kQOrient` (Quaternion Orientation. 16 bytes. **Matches v1!** The `(0.0, 1.0, 0.0, 0.0)` represents an identity or axis-rotation quaternion).
- `0x118 + 0x6C` = **`+0x184`**: `m_kLinMom` (Linear Momentum). Wait, if quaternion is 16 bytes, then the next vector starts at `0x184`. 
- What is `+0x180`? If `v2` was seen at `+0x180`, it falls perfectly on the `w` component of the quaternion! 

Wait, if `CUnit+0x174` is `m_kQOrient`, a quaternion has `(W, X, Y, Z)` or `(X, Y, Z, W)`. At `+0x174`, `+0x178`, `+0x17C`, `+0x180`. 
This explains why `v2` at `+0x180` looked like `(0.0, 22.0, 0.0)`—you were reading a float sequence spanning from the end of the quaternion into the start of the linear momentum!

If we want the true **Linear Velocity** of this embedded `RigidBody`, it is at:
**`0x118 + 0xA8 = CUnit + 0x1C0`**

And the **Linear Momentum** (which acts as a velocity accumulator that scales by inverse mass later) is at:
**`0x118 + 0x6C = CUnit + 0x184`**
