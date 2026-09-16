#include "gba_link.h"

#include <chrono>

namespace gba {
namespace {

// How long the parent waits for the children before giving up on an exchange.
// Hardware has no such timer; it reports a bad connection through SIOCNT's
// error bit when a console does not answer. The timeout exists so one machine
// stalling — paused in a menu, descheduled, stopped in a debugger — degrades to
// that same "bad connection" rather than freezing every other machine with it.
// Short enough that a stalled console costs the parent a few frames rather than
// seconds, long enough that ordinary scheduling jitter never trips it.
constexpr auto kJoinTimeout = std::chrono::milliseconds(50);

}  // namespace

void GbaLink::configure(int players) {
    std::lock_guard<std::mutex> lk(m_);
    if (players < 1) players = 1;
    if (players > kLinkMaxPlayers) players = kLinkMaxPlayers;
    players_ = players;
    running_ = true;
    round_ = 0;
    published_round_.store(0, std::memory_order_release);
    send_.fill(kLinkNoData);
    fresh_.fill(false);
    participating_.fill(false);
    seen_.fill(0);
    result_ = LinkResult{};
}

void GbaLink::set_participating(int port, bool on) {
    if (port < 0 || port >= kLinkMaxPlayers) return;
    std::lock_guard<std::mutex> lk(m_);
    if (participating_[static_cast<std::size_t>(port)] == on) return;
    participating_[static_cast<std::size_t>(port)] = on;
    // A console leaving multi-player mode is one the parent must stop waiting
    // for, so wake it to re-test.
    cv_.notify_all();
}

void GbaLink::set_send(int port, uint16_t value) {
    if (port < 0 || port >= kLinkMaxPlayers) return;
    std::lock_guard<std::mutex> lk(m_);
    send_[static_cast<std::size_t>(port)] = value;
    fresh_[static_cast<std::size_t>(port)] = true;
    cv_.notify_all();
}

bool GbaLink::run_exchange() {
    std::unique_lock<std::mutex> lk(m_);
    if (!running_ || players_ <= 1) return false;

    exchanges_.fetch_add(1, std::memory_order_relaxed);

    // Everyone who is on the cable and in multi-player mode has to have posted
    // a word, so the set that comes out belongs to one moment across all of
    // them rather than to whenever each thread happened to be.
    const bool all_ready = cv_.wait_for(lk, kJoinTimeout, [&] {
        if (!running_) return true;
        for (int i = 1; i < players_; ++i) {
            const auto p = static_cast<std::size_t>(i);
            if (participating_[p] && !fresh_[p]) return false;
        }
        return true;
    });
    if (!running_) return false;
    if (!all_ready) timeouts_.fetch_add(1, std::memory_order_relaxed);

    // A console that is not in multi-player mode is an empty slot on the
    // cable, and an empty slot reads FFFFh.
    for (int i = 0; i < kLinkMaxPlayers; ++i) {
        const auto p = static_cast<std::size_t>(i);
        const bool present = i < players_ && (i == 0 || participating_[p]);
        result_.words[p] = present ? send_[p] : kLinkNoData;
    }
    result_.ok = all_ready;

    ++round_;
    result_.round = round_;
    published_round_.store(round_, std::memory_order_release);
    fresh_.fill(false);
    cv_.notify_all();
    return true;
}

bool GbaLink::poll_result(int port, LinkResult* out) {
    if (port < 0 || port >= kLinkMaxPlayers || !out) return false;
    std::lock_guard<std::mutex> lk(m_);
    if (!running_ || players_ <= 1) return false;
    const auto p = static_cast<std::size_t>(port);
    if (seen_[p] == round_) return false;
    seen_[p] = round_;
    *out = result_;
    return true;
}

void GbaLink::shutdown() {
    std::lock_guard<std::mutex> lk(m_);
    running_ = false;
    cv_.notify_all();
}

}  // namespace gba
