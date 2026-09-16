// gba_instance.h — one complete guest machine, as an object.
//
// The runner historically held the bus and the PPU as two separate locals and
// wired them to the generated-code runtime with a short sequence of free
// calls. Grouping them makes it possible to have more than one machine alive
// at once, which is what emulating a link cable requires: four GBAs exchange
// input every frame and each renders its own view.
//
// The register file is the deliberate omission. It lives in the thread-local
// g_cpu (see armv4t/runtime_arm.h), so a machine is bound to a thread by
// activate() rather than carrying its CPU state here.

#pragma once

#include <cstdint>

#include "gba_bus.h"
#include "gba_link.h"
#include "gba_ppu.h"

namespace gbarecomp {

struct GbaInstance {
    gba::GbaBus bus;
    gba::GbaPpu ppu;

    // Host-loop bookkeeping. Advisory reporting, never guest state.
    std::uint64_t steps            = 0;  // step_once() calls
    std::uint64_t halt_steps       = 0;  // of those, ones that found HALT
    std::uint64_t cycles_elapsed   = 0;  // cycles the halt pump accounted for
    std::uint32_t last_step_cycles = 0;  // cycles pumped during the last step
    std::uint64_t vblank_count     = 0;  // ppu.frame_count() as of the last step

    // Point the generated-code runtime at this machine for the calling thread:
    // subsequent bus and PPU access from recompiled guest code resolves here.
    // Also clears the host call-return stack, so this is machine bring-up, not
    // a cheap rebind — it is not yet safe to call to swap between live
    // machines.
    void activate();

    // Join this machine to a link cable at the given position in the chain.
    // Port 0 is the parent. Must be called before activate(); a machine with no
    // cable behaves as a console on its own.
    void attach_link(gba::GbaLink* link, int port);

    // Advance devices by one scheduling quantum while the guest is halted,
    // stopping short of the next PPU, timer or audio event so nothing is
    // stepped over. Returns the cycles consumed.
    std::uint32_t pump_idle(std::uint32_t max_cycles);

    // Run the guest to the next host-visible boundary: pump the devices if it
    // is halted, otherwise execute from the current PC. Always true today; the
    // bool is the seam for an abnormal stop.
    bool step_once();

    // step_once() until the PPU reaches the next VBlank start. False means the
    // guest never got there within the per-frame dispatch bound, which is a
    // runaway rather than a slow frame.
    bool step_frame();
};

// Put the calling thread's CPU state into the GBA reset condition: SVC mode,
// IRQ and FIQ masked, ARM state, PC 0, and the canonical post-reset banked
// stack pointers. Operates on the thread-local register file rather than on a
// GbaInstance, so it is a free function: it resets whichever machine this
// thread is running.
void reset_guest_cpu();

}  // namespace gbarecomp
