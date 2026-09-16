#include "gba_instance.h"

#include "overlay_loader.h"
#include "runtime_arm.h"
#include "runtime_bus_bridge.h"

namespace gbarecomp {

void GbaInstance::attach_link(gba::GbaLink* link, int port) {
    bus.io().set_link(link, port);
}

void GbaInstance::activate() {
    // The IO block reaches back into the machine for two things, and both fail
    // silently when unwired.
    //
    // Without the PPU it answers every VCOUNT read with 0, so a scanline-polling
    // wait loop never ends. Without the bus it has nothing to move data through,
    // so DMA transfers never happen: no data moves and no bus cycles are stolen,
    // which drifts the clock away from a correctly wired machine and eventually
    // desynchronises device timing badly enough to crash the guest.
    bus.io().set_ppu(&ppu);
    bus.io().set_bus(&bus);
    set_active_bus(&bus);
    set_active_ppu(&ppu);
    runtime_init(&bus);
}

std::uint32_t GbaInstance::pump_idle(std::uint32_t max_cycles) {
    std::uint32_t chunk = ppu.cycles_until_next_event();
    const std::uint32_t until_timer = bus.io().cycles_until_next_timer_event();
    const std::uint32_t until_sample = bus.audio().cycles_until_next_sample();
    if (until_timer < chunk) chunk = until_timer;
    if (until_sample < chunk) chunk = until_sample;
    if (chunk == 0 || chunk == 0xFFFFFFFFu) chunk = 1;
    if (chunk > max_cycles) chunk = max_cycles;
    runtime_tick(chunk);
    cycles_elapsed += chunk;
    last_step_cycles += chunk;
    return chunk;
}

bool GbaInstance::step_once() {
    last_step_cycles = 0;
    // Fold any worker-finished native overlays into the dispatch table before
    // the next dispatch can use them. Cheap when idle.
    overlay_drain_ready();
    if (bus.io().halted()) {
        ++halt_steps;
        std::uint32_t idle_budget = gba::GbaPpu::kCyclesPerFrame;
        while (bus.io().halted() && idle_budget != 0) {
            const std::uint32_t chunk = pump_idle(idle_budget);
            idle_budget -= chunk;
        }
        ++steps;
        vblank_count = ppu.frame_count();
        return true;
    }

    // Co-simulation "interp" backend: interpret one guest instruction rather
    // than dispatching generated code. Shares runtime_tick / runtime_swi with
    // the recomp backend, so only instruction execution differs.
    const std::uint32_t step_pc = g_cpu.R[15] & ~1u;
    const int step_thumb = (g_cpu.cpsr & CPSR_T_BIT) != 0;
    if (g_force_interp ||
        (g_runtime_force_interp_hook &&
         g_runtime_force_interp_hook(step_pc, step_thumb))) {
        runtime_force_interp_step();
    } else {
        runtime_dispatch(g_cpu.R[15]);
    }
    ++steps;
    vblank_count = ppu.frame_count();
    return true;
}

bool GbaInstance::step_frame() {
    const unsigned long long start_vbl = g_runtime_vblank_starts;
    constexpr unsigned long long kMaxDispatchesPerFrame = 2'000'000ull;
    for (unsigned long long i = 0; i < kMaxDispatchesPerFrame; ++i) {
        if (!step_once()) return false;
        if (g_runtime_vblank_starts != start_vbl) return true;
    }
    return false;
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
