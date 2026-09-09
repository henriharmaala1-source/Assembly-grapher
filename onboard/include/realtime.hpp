#pragma once

// Real-time scheduling for the control-critical threads (F9).
//
// The two/three-tier split (fly loop / FcLink / Deliberator) exists so slow
// inference can never stall control — but with default OS scheduling that's a
// SOFT guarantee: under CPU pressure the kernel's fair scheduler can still let a
// depth model on the Deliberator thread delay the fly loop or FcLink. Putting
// the two control threads on SCHED_FIFO (a real-time class that preempts the
// normal SCHED_OTHER Deliberator whenever they're runnable) makes it HARD.
//
// The Deliberator is left on SCHED_OTHER on purpose — it should always yield to
// control. CPU pinning is optional and OFF by default: FIFO priority alone gives
// the preemption guarantee on any core. Only enable pinning (`cpu >= 0`) with
// care — pinning a FIFO thread that fails to yield to a single core can starve
// everything else on it. Our control threads always block (camera read / a
// 20 ms sleep), so they yield; keep it that way.
namespace rt {

// Elevate the CALLING thread to SCHED_FIFO at `prio` (1..99; 0 = leave as-is),
// optionally pinning it to CPU `cpu` (>= 0). Non-fatal: returns false with a
// one-time note if it can't (no CAP_SYS_NICE — e.g. an unprivileged dev box or
// CI), so callers just proceed at normal priority. No-op / false on non-Linux.
bool make_realtime(const char* name, int prio, int cpu);

}  // namespace rt
