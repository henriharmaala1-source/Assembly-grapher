#include "realtime.hpp"

#include <cstdio>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <cstring>

namespace rt {

bool make_realtime(const char* name, int prio, int cpu) {
    bool ok = true;

    if (prio > 0) {
        sched_param sp{};
        sp.sched_priority = prio;
        const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp);
        if (rc != 0) {
            std::fprintf(stderr,
                "[rt] %s: SCHED_FIFO(%d) not permitted (%s) — running normal "
                "priority. Grant CAP_SYS_NICE or run as root on the Pi.\n",
                name, prio, std::strerror(rc));
            ok = false;
        } else {
            std::printf("[rt] %s: SCHED_FIFO priority %d\n", name, prio);
        }
    }

    if (cpu >= 0) {
        cpu_set_t set;
        CPU_ZERO(&set);
        CPU_SET(cpu, &set);
        const int rc = pthread_setaffinity_np(pthread_self(), sizeof(set), &set);
        if (rc != 0) {
            std::fprintf(stderr, "[rt] %s: pin to CPU %d failed (%s)\n",
                         name, cpu, std::strerror(rc));
            ok = false;
        } else {
            std::printf("[rt] %s: pinned to CPU %d\n", name, cpu);
        }
    }

    return ok;
}

}  // namespace rt

#else  // non-Linux: scheduling knobs unavailable

namespace rt {
bool make_realtime(const char*, int, int) { return false; }
}  // namespace rt

#endif
