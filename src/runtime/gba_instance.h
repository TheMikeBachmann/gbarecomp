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

#include "gba_bus.h"
#include "gba_ppu.h"

namespace gbarecomp {

struct GbaInstance {
    gba::GbaBus bus;
    gba::GbaPpu ppu;

    // Point the generated-code runtime at this machine for the calling thread:
    // subsequent bus and PPU access from recompiled guest code resolves here.
    // Also clears the host call-return stack, so this is machine bring-up, not
    // a cheap rebind — it is not yet safe to call to swap between live
    // machines.
    void activate();
};

// Put the calling thread's CPU state into the GBA reset condition: SVC mode,
// IRQ and FIQ masked, ARM state, PC 0, and the canonical post-reset banked
// stack pointers. Operates on the thread-local register file rather than on a
// GbaInstance, so it is a free function: it resets whichever machine this
// thread is running.
void reset_guest_cpu();

}  // namespace gbarecomp
