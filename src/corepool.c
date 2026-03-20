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

/*
 * Each host thread keeps the big lock for a short burst of guest work before
 * yielding. This cuts mutex/scheduler overhead without changing interrupt
 * checks inside cpu_step_core().
 */
#define COREPOOL_STEP_QUANTUM_DEFAULT 64
#define COREPOOL_STEP_QUANTUM_MIN     1
#define COREPOOL_STEP_QUANTUM_MAX     4096

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
    corepool.step_quantum = COREPOOL_STEP_QUANTUM_DEFAULT;
    memset(corepool.thread_active, 0, sizeof(corepool.thread_active));
}

void corepool_set_step_quantum(int quantum) {
    if (quantum < COREPOOL_STEP_QUANTUM_MIN) {
        quantum = COREPOOL_STEP_QUANTUM_MIN;
    } else if (quantum > COREPOOL_STEP_QUANTUM_MAX) {
        quantum = COREPOOL_STEP_QUANTUM_MAX;
    }
    corepool.step_quantum = quantum;
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

static const char *corepool_registry_path(void) {
    const char *path = getenv(COREPOOL_REGISTRY_ENV);
    if (path && path[0]) {
        return path;
    }
    return COREPOOL_REGISTRY_PATH;
}

static FILE *registry_open_locked(int lock_type, int *fd_out) {
    FILE *f = fopen(corepool_registry_path(), "r+");
    if (!f) {
        f = fopen(corepool_registry_path(), "w+");
    }
    if (!f) {
        return NULL;
    }

    int fd = fileno(f);
    if (flock(fd, lock_type) < 0) {
        fclose(f);
        return NULL;
    }

    *fd_out = fd;
    return f;
}

static void registry_close_locked(FILE *f, int fd) {
    flock(fd, LOCK_UN);
    fclose(f);
}

static int registry_read_file(FILE *f, corepool_entry_t *entries, int max_entries) {
    rewind(f);
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
    clearerr(f);
    return count;
}

static void registry_write_file(FILE *f, int fd,
                                corepool_entry_t *entries, int count) {
    if (fflush(f) != 0) {
        return;
    }
    if (ftruncate(fd, 0) != 0) {
        return;
    }
    rewind(f);

    for (int i = 0; i < count; i++) {
        if (entries[i].active) {
            fprintf(f, "%d %d %d\n", (int)entries[i].pid,
                    entries[i].num_cores, entries[i].active);
        }
    }
    fflush(f);
    fsync(fd);
}

void corepool_register(int cores) {
    int fd;
    FILE *f = registry_open_locked(LOCK_EX, &fd);
    if (!f) {
        fprintf(stderr, "[CorePool] Failed to open registry: %s\n", strerror(errno));
        return;
    }

    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read_file(f, entries, COREPOOL_MAX_INSTANCES);

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

    registry_write_file(f, fd, entries, count);
    registry_close_locked(f, fd);
    corepool.registered = 1;

    fprintf(stderr, "[CorePool] Registered PID %d with %d core(s) (host: %d CPUs)\n",
            (int)my_pid, cores, corepool.host_cpus);
}

void corepool_unregister(void) {
    if (!corepool.registered) return;

    int fd;
    FILE *f = registry_open_locked(LOCK_EX, &fd);
    if (!f) {
        fprintf(stderr, "[CorePool] Failed to open registry: %s\n", strerror(errno));
        corepool.registered = 0;
        return;
    }

    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read_file(f, entries, COREPOOL_MAX_INSTANCES);

    pid_t my_pid = getpid();
    for (int i = 0; i < count; i++) {
        if (entries[i].pid == my_pid) {
            entries[i] = entries[count - 1];
            count--;
            i--;
        }
    }

    registry_write_file(f, fd, entries, count);
    registry_close_locked(f, fd);
    corepool.registered = 0;
}

int corepool_query_cores(void) {
    int fd;
    FILE *f = registry_open_locked(LOCK_EX, &fd);
    if (!f) {
        return 1;
    }

    corepool_entry_t entries[COREPOOL_MAX_INSTANCES];
    int count = registry_read_file(f, entries, COREPOOL_MAX_INSTANCES);
    registry_write_file(f, fd, entries, count);
    registry_close_locked(f, fd);

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

        cpu_bind_context_t ctx;
        if (!cpu_bind_core_context(core_id, &ctx)) {
            pthread_mutex_unlock(&corepool.emu_lock);
            continue;
        }

        for (int steps = 0; steps < corepool.step_quantum; steps++) {
            if (!corepool.running || cores[core_id].is_wfi) {
                break;
            }

            cpu_step();

            /* Shared peripheral progression stays tied to guest instruction flow. */
            if (core_id == CORE0) {
                pio_step();
                usb_step();
            }

            if (cpu.r[15] == 0xFFFFFFFF) {
                break;
            }
        }
        cpu_unbind_core_context(core_id, &ctx);

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
    if (corepool.running) {
        corepool_stop_threads();
    }
    corepool_unregister();
    pthread_mutex_destroy(&corepool.emu_lock);
    pthread_cond_destroy(&corepool.wfi_cond);
}
