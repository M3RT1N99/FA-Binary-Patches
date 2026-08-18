// Stubs for hooks/VisionFlush.hook -- fog-of-war stencil overflow fix.
//
// Both stubs are entered via jmp (not call) from func_ren_FogOfWar
// (0x0081C660), so esp and all registers are exactly the engine's loop
// state. Stack slots at loop level (verified against the engine's own
// accesses): [esp+0x18] done counter, [esp+0x1C] total circle count,
// [esp+0x20] current batch size, [esp+0x30] gal::Head* (width/height),
// [esp+0xC80] Moho::VisionRenderer*, ebp = gpg::gal::DeviceD3D9*.
//
// VisionRenderer layout used here: +0x0C/+0x10 vertex-declaration
// shared_ptr, +0x14/+0x18 template-cone VB shared_ptr, +0x1C/+0x20 index
// buffer shared_ptr, +0x30 embedded Moho::CRenFrame.
//
// Engine calling conventions (same as the RenderRingsFlush patch):
//   0x007F5DA0 Moho::CRenFrame::InitTransformedVerts(float,float): this@EBX
//   0x004059E0 std::string::string(char const*,uint): this@ECX (thiscall)
//   0x007F6030 Moho::CRenFrame::Render(int,int): this@EDI
// Device methods take a BY-VALUE boost::shared_ptr (8 bytes on the stack,
// manually addref'd, consumed by the callee):
//   vtbl+0xA0 SetVertexDeclaration(sp)
//   vtbl+0xA4 SetVertexBuffer(stream, sp, frequency, offset)
//   vtbl+0xA8 SetBufferIndices(sp)

// Clamp the initial stream-0 instancing frequency (edi = total circle
// count) to the first batch size, then reproduce the four replaced
// instructions and resume. edx already holds the template VB pointer.
void asm__VisionVB0Clamp()
{
    asm(
        "cmp     edi, 127;"
        "jle     L_clampDone;"
        "mov     edi, 127;"
        "L_clampDone:;"
        "push    0;"                       // offset
        "push    edi;"                     // frequency = first batch size
        "sub     esp, 8;"                  // shared_ptr temp
        "mov     ecx, esp;"
        "jmp     0x0081C967;"              // resume: mov [ecx], edx; ...
    );
}

// Replaced loop tail of the fog batch loop. Advances the counter first: on
// loop exit no flush is needed (the final Vision pass treats both the 0x80
// mark and a raw 1..127 count as "seen"), so a single batch -- the common
// case -- costs nothing extra.
void asm__VisionFlushTail()
{
    asm(
        "mov     eax, [esp+0x18];"         // done
        "add     eax, [esp+0x20];"         // += batch
        "mov     [esp+0x18], eax;"
        "cmp     eax, [esp+0x1C];"         // vs total
        "jl      L_moreBatches;"
        "jmp     0x0081CB23;"              // done: engine's final Vision pass

        "L_moreBatches:;"
        // Fullscreen VisionMask pass: convert stencil counts (bits 0-6)
        // to the bit-7 "seen" mark before the next batch can wrap them.
        "mov     esi, [esp+0xC80];"        // VisionRenderer*
        "lea     edi, [esi+0x30];"         // &VisionRenderer->mFrame
        "mov     esi, [esp+0x30];"         // gal::Head*
        "cvtsi2ss xmm0, dword ptr [esi+0x14];"
        "movd    eax, xmm0;"
        "push    eax;"                     // height (float)
        "cvtsi2ss xmm0, dword ptr [esi+0x10];"
        "movd    eax, xmm0;"
        "push    eax;"                     // width (float)
        "mov     ebx, edi;"                // this
        "call    0x007F5DA0;"              // CRenFrame::InitTransformedVerts
        "push    10;"                      // len
        "push    offset L_strVisionMask;"
        "mov     ecx, edi;"                // this
        "call    0x004059E0;"              // std::string::string
        "push    dword ptr [esi+0x14];"    // height
        "push    dword ptr [esi+0x10];"    // width
        "call    0x007F6030;"              // CRenFrame::Render

        // CRenFrame::Render draws a fullscreen quad UP-style, clobbering
        // the vertex declaration and stream 0. Restore the declaration and
        // (cheap insurance) the index buffer, mirroring the engine's own
        // setup sequence at 0x81C92B..0x81C9C1.
        "mov     esi, [esp+0xC80];"        // VisionRenderer*
        "mov     edx, [esi+0x0C];"
        "sub     esp, 8;"
        "mov     ecx, esp;"
        "mov     [ecx], edx;"
        "mov     eax, [esi+0x10];"
        "test    eax, eax;"
        "mov     [ecx+4], eax;"
        "jz      L_declNull;"
        "add     eax, 4;"
        "mov     ecx, 1;"
        "lock xadd [eax], ecx;"
        "L_declNull:;"
        "mov     edx, [ebp];"
        "mov     eax, [edx+0xA0];"
        "mov     ecx, ebp;"
        "call    eax;"                     // SetVertexDeclaration

        "mov     edx, [esi+0x1C];"
        "sub     esp, 8;"
        "mov     ecx, esp;"
        "mov     [ecx], edx;"
        "mov     eax, [esi+0x20];"
        "test    eax, eax;"
        "mov     [ecx+4], eax;"
        "jz      L_idxNull;"
        "add     eax, 4;"
        "mov     ecx, 1;"
        "lock xadd [eax], ecx;"
        "L_idxNull:;"
        "mov     edx, [ebp];"
        "mov     eax, [edx+0xA8];"
        "mov     ecx, ebp;"
        "call    eax;"                     // SetBufferIndices

        // Stream-0 instancing frequency for the next batch:
        // min(total - done, 127), matching the loop head's batch clamp.
        "mov     edi, [esp+0x1C];"
        "sub     edi, [esp+0x18];"
        "cmp     edi, 127;"
        "jle     L_freqOk;"
        "mov     edi, 127;"
        "L_freqOk:;"
        "mov     edx, [esi+0x14];"
        "push    0;"                       // offset
        "push    edi;"                     // frequency
        "sub     esp, 8;"
        "mov     ecx, esp;"
        "mov     [ecx], edx;"
        "mov     eax, [esi+0x18];"
        "test    eax, eax;"
        "mov     [ecx+4], eax;"
        "jz      L_vbNull;"
        "add     eax, 4;"
        "mov     ecx, 1;"
        "lock xadd [eax], ecx;"
        "L_vbNull:;"
        "mov     edx, [ebp];"
        "mov     eax, [edx+0xA4];"
        "push    0;"                       // stream 0
        "mov     ecx, ebp;"
        "call    eax;"                     // SetVertexBuffer
        "jmp     0x0081C9D5;"              // next batch (reloads regs from stack)

        "L_strVisionMask: .asciz \"VisionMask\";"
    );
}
