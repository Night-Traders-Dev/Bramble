/*
 * Core Pool — Host-Threaded Execution & Multi-Instance Coordination
 *
 * Provides pthread-per-core execution model so each emulated RP2040 core
 * runs on its own host CPU thread. Uses a big lock for shared state and
 * condition variables for WFI/WFE sleep optimization.
 *
 * Multi-instance coordination uses a shared file registry so multiple
 * bramble processes can detect each other and avoid oversubscribing
 * the host CPU.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <time.h>
#include "corepool.h"
#include "emulator.h"
#include "nvic.h"
#include "pio.h"
#include "usb.h"

corepool_state_t corepool = {0};

/* ========================================================================
 * Host CPU Detection
 * ======================================================================== */

int corepool_detect_host_cpus(void) {
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    if (n < 1) n = 1;
    return (int)n;
}

/* ========================================================================
 * Initialization
 * ======================================================================== */

void corepool_init(void) {
    pthread_mutex_init(&corepool.emu_lock, NULL);

    pthread_condattr_t attr;
    pthread_condattr_init(&attr);
    pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
    pthread_cond_init(&corepool.wfi_cond, &attr);
    pthread_condattr_destroy(&attr);

    corepool.running = 0;
    corepool.registered = 0;
    corepool.host_cpus = corepool_detect_host_cpus();
    memset(corepool.thread_active, 0, sizeof(corepool.thread_active));
}

/* ========================================================================
 * Multi-Instance Registry
 *
 * A simple file-based registry at /tmp/bramble-corepool.reg.
 * Each line: PID CORES ACTIVE
 * File-locked (flock) during read/write for process safety.
 * Stale entries (dead PIDs) are cleaned on each access.
 * ======================================================================== */

static int is_pid_alive(pid_t pid) {
    return kill(pid, 0) == 0 || errno == EPERM;
}

static int registry_read(corepool_entry_t *entries, int max_entries) {
    FILE *f = fopen(COREPOOL_REGISTRY_PATH, "r");
    if (!f) return 0;

    int fd = fileno(f);
    flock(fd, LOCK_SH);

    int count = 0;
    while (count < max_entries) {
        int pid, cores, active;
        if (fscanf(f, "%d %d %d", &pid, &cores, &active) != 3) break;
        if (active && is_pid_alive((pid_t)pid)) {
            entries[count].pid = (pid_t)pid;
            entries[count].num_cores = cores;
            entries[count].active = 1;
            count++;
        }
    }

    flock(fd, LOCK_UN);
    fclose(f);
    return count;
}

static void registry_write(corepool_entry_t *entries, int count) {
    FILE *f = fopen(COREPOOL_REGISTRY_PATH, "w");
    if (!f) return;

    int fd = fileno(f);
    flock(fd, LOCK_EX);

    for (int i = 0; i < count; i++) {
        if (entries[i].active) {
            fprintf(f, "%d %d %d\n", (int)entries[i].pid,
                    entries[i].num_cores, entries[i].active);
        }
    }

    flock(fd, LOCK_UN);
    fclose(f);
}

void corepool_register(int cores) {
    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read(entries, COREPOOL_MAX_INSTANCES);

    /* Remove any existing entry for this PID */
    pid_t my_pid = getpid();
    for (int i = 0; i < count; i++) {
        if (entries[i].pid == my_pid) {
            entries[i] = entries[count - 1];
            count--;
            i--;
        }
    }

    /* Add our entry */
    if (count < COREPOOL_MAX_INSTANCES) {
        entries[count].pid = my_pid;
        entries[count].num_cores = cores;
        entries[count].active = 1;
        count++;
    }

    registry_write(entries, count);
    corepool.registered = 1;

    fprintf(stderr, "[CorePool] Registered PID %d with %d core(s) (host: %d CPUs)\n",
            (int)my_pid, cores, corepool.host_cpus);
}

void corepool_unregister(void) {
    if (!corepool.registered) return;

    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read(entries, COREPOOL_MAX_INSTANCES);

    pid_t my_pid = getpid();
    for (int i = 0; i < count; i++) {
        if (entries[i].pid == my_pid) {
            entries[i] = entries[count - 1];
            count--;
            i--;
        }
    }

    registry_write(entries, count);
    corepool.registered = 0;
}

int corepool_query_cores(void) {
    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read(entries, COREPOOL_MAX_INSTANCES);

    /* Sum cores already allocated across all active instances */
    int total_allocated = 0;
    for (int i = 0; i < count; i++) {
        total_allocated += entries[i].num_cores;
    }

    int available = corepool.host_cpus - total_allocated;

    /* Always allocate at least 1 core, up to MAX_CORES */
    if (available >= MAX_CORES) return MAX_CORES;
    if (available >= 1) return available;
    return 1;
}

/* ========================================================================
 * Core Thread Entry Point
 *
 * Each thread runs one emulated core in a tight loop:
 *   1. Acquire big lock
 *   2. Check if core is in WFI → sleep on condvar
 *   3. Step one instruction
 *   4. Release big lock
 *   5. Yield to let other core thread run
 * ======================================================================== */

static void *core_thread_fn(void *arg) {
    int core_id = (int)(intptr_t)arg;

    while (corepool.running) {
        pthread_mutex_lock(&corepool.emu_lock);

        /* Check if emulator should stop */
        if (!corepool.running || cores[core_id].is_halted) {
            pthread_mutex_unlock(&corepool.emu_lock);
            if (cores[core_id].is_halted) {
                /* Halted core: sleep briefly then re-check (may be re-launched) */
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec += 1000000;  /* 1ms */
                if (ts.tv_nsec >= 1000000000) {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000;
                }
                pthread_mutex_lock(&corepool.emu_lock);
                pthread_cond_timedwait(&corepool.wfi_cond, &corepool.emu_lock, &ts);
                pthread_mutex_unlock(&corepool.emu_lock);
                continue;
            }
            break;
        }

        /* WFI/WFE sleep: wait on condition variable until interrupt pending */
        if (cores[core_id].is_wfi) {
            /* Check for pending interrupt before sleeping */
            set_active_core(core_id);
            uint32_t pending = nvic_get_pending_irq();
            int wake = (pending != 0xFFFFFFFF) ||
                       systick_states[core_id].pending ||
                       nvic_states[core_id].pendsv_pending;

            if (!wake) {
                /* Sleep with 1ms timeout (for periodic checks) */
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                ts.tv_nsec += 1000000;  /* 1ms */
                if (ts.tv_nsec >= 1000000000) {
                    ts.tv_sec++;
                    ts.tv_nsec -= 1000000000;
                }
                pthread_cond_timedwait(&corepool.wfi_cond, &corepool.emu_lock, &ts);
                pthread_mutex_unlock(&corepool.emu_lock);
                continue;
            }
            cores[core_id].is_wfi = 0;
        }

        /* Step one instruction */
        cpu_step_core(core_id);

        /* Also step shared peripherals (only core 0 drives these) */
        if (core_id == CORE0) {
            pio_step();
            usb_step();
        }

        pthread_mutex_unlock(&corepool.emu_lock);

        /* Brief yield to allow other core thread to acquire lock */
        sched_yield();
    }

    return NULL;
}

/* ========================================================================
 * Thread Lifecycle
 * ======================================================================== */

void corepool_start_threads(void) {
    corepool.running = 1;

    for (int i = 0; i < num_active_cores; i++) {
        if (pthread_create(&corepool.threads[i], NULL,
                           core_thread_fn, (void *)(intptr_t)i) == 0) {
            corepool.thread_active[i] = 1;
            fprintf(stderr, "[CorePool] Started thread for Core %d\n", i);
        } else {
            fprintf(stderr, "[CorePool] Failed to start thread for Core %d\n", i);
            corepool.thread_active[i] = 0;
        }
    }
}

void corepool_stop_threads(void) {
    corepool.running = 0;

    /* Wake any sleeping threads */
    pthread_cond_broadcast(&corepool.wfi_cond);

    for (int i = 0; i < NUM_CORES; i++) {
        if (corepool.thread_active[i]) {
            pthread_join(corepool.threads[i], NULL);
            corepool.thread_active[i] = 0;
            fprintf(stderr, "[CorePool] Joined thread for Core %d\n", i);
        }
    }
}

/* ========================================================================
 * Wake / Lock Utilities
 * ======================================================================== */

void corepool_wake_cores(void) {
    pthread_cond_broadcast(&corepool.wfi_cond);
}

void corepool_lock(void) {
    pthread_mutex_lock(&corepool.emu_lock);
}

void corepool_unlock(void) {
    pthread_mutex_unlock(&corepool.emu_lock);
}

void corepool_cleanup(void) {
    corepool_unregister();
    pthread_mutex_destroy(&corepool.emu_lock);
    pthread_cond_destroy(&corepool.wfi_cond);
}
