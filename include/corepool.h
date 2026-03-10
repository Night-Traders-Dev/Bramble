#ifndef COREPOOL_H
#define COREPOOL_H

#include <pthread.h>
#include <stdint.h>

/* ========================================================================
 * Core Pool — Host-Threaded Execution & Multi-Instance Coordination
 *
 * Each emulated RP2040 core runs in its own host pthread for true
 * parallel execution on multi-core host CPUs.
 *
 * Thread model:
 *   - One pthread per active emulated core
 *   - Big lock (mutex) protects shared peripheral state
 *   - WFI/WFE cores sleep on a condition variable (zero CPU usage)
 *   - Interrupt delivery signals the condvar to wake sleeping cores
 *
 * Multi-instance coordination:
 *   - Shared registry file tracks running bramble instances
 *   - Each instance registers its PID and core count
 *   - Dynamic allocation: new instances check available host CPUs
 *     and adjust core count to avoid oversubscription
 *   - `-cores auto` queries the pool for optimal allocation
 *
 * Usage:
 *   ./bramble firmware.uf2 -cores 2          # Explicit 2 cores
 *   ./bramble firmware.uf2 -cores 1          # Single-core mode
 *   ./bramble firmware.uf2 -cores auto       # Auto-detect from pool
 * ======================================================================== */

#define COREPOOL_REGISTRY_PATH  "/tmp/bramble-corepool.reg"
#define COREPOOL_MAX_INSTANCES  16

/* Per-instance registration entry */
typedef struct {
    pid_t pid;              /* Process ID */
    int num_cores;          /* Cores allocated to this instance */
    int active;             /* 1 = running, 0 = stale entry */
} corepool_entry_t;

/* Core pool state (per-process) */
typedef struct {
    /* Threading */
    pthread_t threads[2];           /* One per emulated core */
    int thread_active[2];           /* Thread is running */
    pthread_mutex_t emu_lock;       /* Big lock for peripheral access */
    pthread_cond_t wfi_cond;        /* WFI/WFE sleep condition */
    volatile int running;           /* Global run flag */

    /* Coordination */
    int registered;                 /* Instance registered in pool */
    int auto_cores;                 /* -cores auto requested */
    int host_cpus;                  /* Detected host CPU count */
} corepool_state_t;

extern corepool_state_t corepool;

/* Initialize core pool (detect host CPUs, create mutex/condvar) */
void corepool_init(void);

/* Register this instance in the shared pool registry */
void corepool_register(int num_cores);

/* Unregister this instance from the pool */
void corepool_unregister(void);

/* Query optimal core count for a new instance (based on host CPUs and existing instances) */
int corepool_query_cores(void);

/* Start threaded execution: spawns a pthread per active core */
void corepool_start_threads(void);

/* Stop all core threads and join */
void corepool_stop_threads(void);

/* Signal sleeping cores to wake (call after NVIC pend, SysTick, etc.) */
void corepool_wake_cores(void);

/* Acquire/release the big emulator lock (for peripheral access from threads) */
void corepool_lock(void);
void corepool_unlock(void);

/* Cleanup mutex/condvar */
void corepool_cleanup(void);

/* Detect number of host CPU cores */
int corepool_detect_host_cpus(void);

#endif /* COREPOOL_H */
