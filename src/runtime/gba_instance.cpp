#include "gba_instance.h"

#include "runtime_arm.h"
#include "runtime_bus_bridge.h"

namespace gbarecomp {

void GbaInstance::activate() {
    set_active_bus(&bus);
    set_active_ppu(&ppu);
    runtime_init(&bus);
}

}  // namespace gbarecomp
