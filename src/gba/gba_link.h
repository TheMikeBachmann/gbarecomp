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
// given a meaning.
//
// The shape that follows from that: children never block. A child posts its
// word with set_send() and carries on. The parent calls run_exchange(), which
// waits — briefly, and only on the consoles that say they are in multi-player
// mode — for everyone to have posted a word for this round, then latches all
// four and publishes one result. Each console picks that result up on its own
// thread through poll_result(), so delivery never reaches into another
// machine's IO block.
//
// Waiting for a word is not what hardware does: there the parent's clock shifts
// out whatever a child happens to be holding, and the game's protocol notices
// the stale word and retries. Waiting turns that race into a rendezvous, which
// is both cheaper (no retry storms between threads that are merely a few
// hundred microseconds apart) and reproducible. The timeout is the escape
// hatch: a console that never answers degrades to the hardware behaviour of a
// bad connection rather than stalling the parent for good.

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

// One exchange's worth of data, as every console sees it.
struct LinkResult {
    std::array<uint16_t, kLinkMaxPlayers> words{
        {kLinkNoData, kLinkNoData, kLinkNoData, kLinkNoData}};
    // False when somebody who claimed to be in multi-player mode did not post
    // a word in time; the console reports it through SIOCNT's error bit.
    bool ok = false;
    // Which exchange this is. A console remembers the last one it applied so
    // the lock-free check below can skip the common "nothing new" case.
    uint64_t round = 0;
};

class GbaLink {
public:
    // Number of consoles sharing this cable. Ports are handed out in order, so
    // port 0 is the parent — on hardware that is decided by which end of the
    // chain a console is plugged into, not by software.
    void configure(int players);

    int players() const { return players_; }
    bool connected() const { return players_ > 1; }

    // Whether this console currently has SIO configured for multi-player. Only
    // consoles that say yes are waited for; the rest read as empty slots, which
    // is what a console with nothing plugged in looks like from the others.
    void set_participating(int port, bool on);

    // Latch this console's outgoing halfword for the next exchange.
    void set_send(int port, uint16_t value);

    // Parent only. Wait for every participating child to have posted a word
    // for this round, then latch all four and publish the result to every
    // console. Returns false if the cable is down. Blocks for at most the join
    // timeout; see the note above on why it blocks at all.
    bool run_exchange();

    // Non-blocking. True when an exchange this console has not picked up yet is
    // waiting, in which case `out` is filled and the exchange is marked seen.
    bool poll_result(int port, LinkResult* out);

    // Number of the most recently published exchange, readable without the
    // lock. Every console checks this once per device tick, so the common case
    // — no exchange since the last one it applied — must not take the mutex.
    uint64_t published_round() const {
        return published_round_.load(std::memory_order_acquire);
    }

    // Release everyone waiting; used when shutting the machines down so no
    // thread is left blocked in run_exchange().
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
    // Posted a word since the last exchange closed.
    std::array<bool, kLinkMaxPlayers> fresh_{};
    std::array<bool, kLinkMaxPlayers> participating_{};

    // Exchanges are numbered so a console can tell a result it has already
    // applied from one it has not.
    uint64_t round_ = 0;
    std::atomic<uint64_t> published_round_{0};
    std::array<uint64_t, kLinkMaxPlayers> seen_{};
    LinkResult result_;

    std::atomic<uint64_t> exchanges_{0};
    std::atomic<uint64_t> timeouts_{0};
};

}  // namespace gba
