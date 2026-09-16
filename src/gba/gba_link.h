// gba_link.h — the cable between several emulated consoles.
//
// GBA multi-player mode (GBATEK § "SIO Multi-Player Mode") wires up to four
// consoles in a chain. One is the parent, decided by the cable rather than by
// software; the rest are children. Every console keeps one outgoing halfword in
// SIOMLT_SEND. When the parent sets the start bit, all four are exchanged at
// once: each console ends up with every console's word in SIOMULTI0..3, indexed
// by position on the cable, and each takes a serial interrupt.
//
// Only the parent starts a transfer. A child needs no cooperation at all — its
// register is already latched and the parent's clock shifts it out. That is the
// awkward part to emulate: the consoles here are separate threads running
// freely, so "the value a child had at the moment the parent started" has to be
// given a meaning. GbaLink supplies it by making the exchange a rendezvous —
// see transfer().

#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>

namespace gba {

constexpr int kLinkMaxPlayers = 4;

// Value read for a slot with no console in it, and the value a disconnected
// console reads for everyone. GBATEK: unused SIOMULTI entries read FFFFh.
constexpr uint16_t kLinkNoData = 0xFFFFu;

class GbaLink {
public:
    // Number of consoles sharing this cable. Ports are handed out in order, so
    // port 0 is the parent — on hardware that is decided by which end of the
    // chain a console is plugged into, not by software.
    void configure(int players);

    int players() const { return players_; }
    bool connected() const { return players_ > 1; }

    // Latch this console's outgoing halfword. Cheap and lock-free from the
    // console's own thread; only transfer() reads across threads.
    void set_send(int port, uint16_t value);

    // Run one exchange on behalf of the parent, filling `out` with every
    // console's word. Blocks until all consoles have reached this transfer, so
    // that the words gathered belong to the same moment on every machine rather
    // than to whenever each thread happened to be. Returns false if the cable
    // was torn down while waiting, in which case the caller should behave as a
    // console with no partner.
    bool transfer(int port, std::array<uint16_t, kLinkMaxPlayers>* out);

    // Release everyone waiting; used when shutting the machines down so no
    // thread is left blocked in transfer().
    void shutdown();

    // Exchanges attempted and exchanges where everybody turned up. The first
    // answers "is the game using the cable at all", which is the question while
    // bringing this up; the gap between them is how often a console was late.
    uint64_t exchanges() const {
        return exchanges_.load(std::memory_order_relaxed);
    }
    uint64_t timeouts() const {
        return timeouts_.load(std::memory_order_relaxed);
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    int players_ = 1;
    bool running_ = true;

    std::array<uint16_t, kLinkMaxPlayers> send_{
        {kLinkNoData, kLinkNoData, kLinkNoData, kLinkNoData}};

    // A transfer is identified by a counter so a console can tell "the exchange
    // I am joining" from "one that already finished".
    std::atomic<uint64_t> exchanges_{0};
    std::atomic<uint64_t> timeouts_{0};

    uint64_t round_ = 0;
    int arrived_ = 0;
    std::array<uint16_t, kLinkMaxPlayers> latched_{
        {kLinkNoData, kLinkNoData, kLinkNoData, kLinkNoData}};
};

}  // namespace gba
