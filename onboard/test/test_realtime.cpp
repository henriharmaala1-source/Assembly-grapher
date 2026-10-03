// F9 real-time scheduling helper. Verifies make_realtime() either takes effect
// (the calling thread really is SCHED_FIFO at the requested priority) or fails
// gracefully — CI usually lacks CAP_SYS_NICE, so a false return is an accepted
// skip, not a failure. Either way it must never crash and must return a bool.

#include <cstdio>
#include <thread>

#include "realtime.hpp"

static int fails = 0;
#define CHECK(cond) do { if (!(cond)) { \
    std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); ++fails; } } while (0)

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>

static void verify_policy(int wantPrio, bool applied) {
    int policy = 0;
    sched_param sp{};
    CHECK(pthread_getschedparam(pthread_self(), &policy, &sp) == 0);
    if (applied) {                              // succeeded → must actually be FIFO
        CHECK(policy == SCHED_FIFO);
        CHECK(sp.sched_priority == wantPrio);
    } else {                                    // skipped → left as default class
        CHECK(policy == SCHED_OTHER);
        std::printf("[test] make_realtime skipped (no CAP_SYS_NICE) — accepted\n");
    }
}
#else
static void verify_policy(int, bool) {}
#endif

int main() {
    // Priority elevation: applied or gracefully skipped, verified against the
    // actual thread policy. Runs on a worker so the harness/main thread's policy
    // is untouched.
    std::thread([]{
        const bool ok = rt::make_realtime("test", 15, -1);   // no CPU pin
        verify_policy(15, ok);
    }).join();

    // prio<=0 with no pin is a no-op that must not fault (returns without error).
    std::thread([]{ rt::make_realtime("noop", 0, -1); }).join();

    std::printf("test_realtime: %s (%d failures)\n", fails ? "FAIL" : "OK", fails);
    return fails ? 1 : 0;
}
