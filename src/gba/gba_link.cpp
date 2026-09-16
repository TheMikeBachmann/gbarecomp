#include "gba_link.h"

#include <chrono>

namespace gba {
namespace {

// How long a console waits for the others before giving up on an exchange.
// Hardware has no such timer; it reports a bad connection through SIOCNT's
// error bit when a console does not answer. The timeout exists so one machine
// stalling — paused in a menu, descheduled, stopped in a debugger — degrades to
// that same "bad connection" rather than freezing every other machine with it.
// Generous relative to a frame, so ordinary scheduling jitter never trips it.
constexpr auto kJoinTimeout = std::chrono::milliseconds(250);

}  // namespace

void GbaLink::configure(int players) {
    std::lock_guard<std::mutex> lk(m_);
    if (players < 1) players = 1;
    if (players > kLinkMaxPlayers) players = kLinkMaxPlayers;
    players_ = players;
    running_ = true;
    arrived_ = 0;
    round_ = 0;
    send_.fill(kLinkNoData);
    latched_.fill(kLinkNoData);
}

void GbaLink::set_send(int port, uint16_t value) {
    if (port < 0 || port >= kLinkMaxPlayers) return;
    std::lock_guard<std::mutex> lk(m_);
    send_[static_cast<std::size_t>(port)] = value;
}

bool GbaLink::transfer(int port, std::array<uint16_t, kLinkMaxPlayers>* out) {
    if (port < 0 || port >= kLinkMaxPlayers || !out) return false;
    std::unique_lock<std::mutex> lk(m_);
    if (!running_ || players_ <= 1) return false;

    const uint64_t my_round = round_;

    // Take this console's outgoing word as it stands now. Every console does
    // this on the way into the same round, so the set that comes out belongs to
    // one moment across all of them.
    latched_[static_cast<std::size_t>(port)] =
        send_[static_cast<std::size_t>(port)];

    if (++arrived_ >= players_) {
        // Last one in closes the round and releases the rest.
        ++round_;
        arrived_ = 0;
        cv_.notify_all();
    } else {
        const bool completed = cv_.wait_for(lk, kJoinTimeout, [&] {
            return round_ != my_round || !running_;
        });
        if (!running_) return false;
        if (!completed) {
            // Somebody never arrived. Abandon the round so the next one starts
            // clean, and report the words we do have — absent consoles keep
            // their FFFFh, which is what a console with nothing on the other
            // end of the cable reads.
            arrived_ = 0;
            ++round_;
            for (int i = 0; i < kLinkMaxPlayers; ++i)
                (*out)[static_cast<std::size_t>(i)] =
                    latched_[static_cast<std::size_t>(i)];
            latched_.fill(kLinkNoData);
            cv_.notify_all();
            return false;
        }
    }

    *out = latched_;
    return true;
}

void GbaLink::shutdown() {
    std::lock_guard<std::mutex> lk(m_);
    running_ = false;
    cv_.notify_all();
}

}  // namespace gba
