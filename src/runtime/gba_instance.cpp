#include "gba_instance.h"

#include "runtime_arm.h"
#include "runtime_bus_bridge.h"

namespace gbarecomp {

void GbaInstance::activate() {
    set_active_bus(&bus);
    set_active_ppu(&ppu);
    runtime_init(&bus);
}

void reset_guest_cpu() {
    runtime_trace_reset();
    for (int i = 0; i < 16; ++i) g_cpu.R[i] = 0;
    for (int i = 0; i < ARM_BANK_COUNT; ++i) {
        g_cpu.banked_sp[i] = 0;
        g_cpu.banked_lr[i] = 0;
        g_cpu.banked_spsr[i] = 0;
    }
    for (int i = 0; i < 5; ++i) { g_cpu.r8_12_user[i] = 0; g_cpu.r8_12_fiq[i] = 0; }
    g_cpu.R[13] = 0x03007FE0;
    g_cpu.cpsr = CPSR_I_BIT | CPSR_F_BIT | 0x13u /* SVC */;
    // Seed the banked stack pointers to the canonical GBA post-reset values
    // (GBATEK "GBA Reset"; what hardware / mGBA / the bios_smoke interpreter
    // oracle leave after BIOS reset). Without this the User/System and IRQ banks
    // were 0, so the BIOS reset path's first `msr cpsr,#0x1f` (System mode, at
    // BIOS 0x90) banked in SP=0 instead of 0x03007F00 — the first recomp-vs-interp
    // divergence (cycle 16), cascading into stack writes to address ~0 and a
    // multi-KB IWRAM divergence. (MC-HP-002 fresh-boot root.)
    g_cpu.banked_sp[ARM_BANK_SUPERVISOR] = 0x03007FE0;
    g_cpu.banked_sp[ARM_BANK_IRQ]        = 0x03007FA0;
    g_cpu.banked_sp[ARM_BANK_USER]       = 0x03007F00;
}

}  // namespace gbarecomp
