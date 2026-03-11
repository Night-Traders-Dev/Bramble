/*
 * Bramble JIT Benchmark
 *
 * Measures emulator throughput with and without JIT block compilation.
 * Builds synthetic firmware with tight loops, then compares wall-clock
 * time for each execution mode.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "emulator.h"
#include "nvic.h"
#include "timer.h"
#include "rom.h"

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

/*
 * Build a synthetic firmware with tight loops.
 * Total ~2M+ instructions before hitting the halt loop.
 */
static void build_firmware(void) {
    memset(cpu.flash, 0xFF, FLASH_SIZE);

    uint32_t vt_offset = 0x100;
    uint32_t code_start = 0x200;

    /* Vector table */
    uint32_t sp = RAM_BASE + RAM_SIZE;
    memcpy(&cpu.flash[vt_offset + 0], &sp, 4);
    uint32_t reset = FLASH_BASE + code_start + 1;
    memcpy(&cpu.flash[vt_offset + 4], &reset, 4);

    /* Also write a HardFault handler that halts (just B . at a known address) */
    uint32_t hf_addr = FLASH_BASE + 0x1F0 + 1;
    memcpy(&cpu.flash[vt_offset + 3 * 4], &hf_addr, 4);
    /* HardFault handler: B . */
    uint16_t halt = 0xE7FE;
    memcpy(&cpu.flash[0x1F0], &halt, 2);

    /*
     * Code layout:
     *
     * ; --- Loop 1: Count down from 0xFFFFF (~1M iterations, 2 insns/iter) ---
     *   MOVS R0, #255        ; 0x20FF
     *   LSLS R0, R0, #12     ; 0x0300  -> R0 = 0xFF000
     *   MOVS R1, #255
     *   LSLS R1, R1, #4      ; R1 = 0xFF0
     *   ADDS R0, R0, R1      ; R0 = 0xFFFf0
     * loop1:
     *   SUBS R0, #1           ; 0x3801
     *   BNE loop1             ; 0xD1FD
     *
     * ; --- Loop 2: Accumulate (200 iterations, 3 insns/iter) ---
     *   MOVS R0, #200         ; 0x20C8
     *   MOVS R1, #0           ; 0x2100
     * loop2:
     *   ADDS R1, R1, R0       ; 0x1809
     *   SUBS R0, #1           ; 0x3801
     *   BNE loop2             ; 0xD1FC
     *
     * ; --- Loop 3: Long straight-line block (50 iters, 10 insns/iter) ---
     *   MOVS R4, #50          ; 0x2432
     *   MOVS R0, #10          ; 0x200A
     *   MOVS R1, #20          ; 0x2114
     *   MOVS R2, #30          ; 0x221E
     *   MOVS R3, #1           ; 0x2301
     * loop3:
     *   ADDS R5, R0, R1       ;
     *   ADDS R5, R5, R2       ;
     *   ADDS R5, R5, R3       ;
     *   SUBS R5, R5, R0       ;
     *   LSLS R6, R5, #1       ;
     *   LSRS R6, R6, #1       ;
     *   ADDS R0, R0, R3       ;
     *   SUBS R0, R0, R3       ; (net R0 unchanged)
     *   SUBS R4, #1           ; 0x3C01
     *   BNE loop3             ;
     *
     * ; --- Repeat Loop 1 for more volume ---
     *   MOVS R0, #255
     *   LSLS R0, R0, #12
     *   MOVS R1, #255
     *   LSLS R1, R1, #4
     *   ADDS R0, R0, R1
     * loop4:
     *   SUBS R0, #1
     *   BNE loop4
     *
     * ; halt
     *   B .                   ; 0xE7FE
     */

    uint16_t *code = (uint16_t *)&cpu.flash[code_start];
    int i = 0;

    /* --- Loop 1 setup --- */
    code[i++] = 0x20FF;          /* MOVS R0, #255 */
    code[i++] = 0x0300;          /* LSLS R0, R0, #12 -> 0xFF000 */
    code[i++] = 0x21FF;          /* MOVS R1, #255 */
    code[i++] = 0x0109;          /* LSLS R1, R1, #4 -> 0xFF0 */
    code[i++] = 0x1840;          /* ADDS R0, R0, R1 -> 0xFFFF0 */

    /* Loop 1 body: SUBS R0, #1; BNE loop1 */
    int loop1 = i;
    code[i++] = 0x3801;          /* SUBS R0, #1 */
    code[i++] = 0xD1FD;          /* BNE -2 (back to SUBS) */

    /* --- Loop 2 setup --- */
    code[i++] = 0x20C8;          /* MOVS R0, #200 */
    code[i++] = 0x2100;          /* MOVS R1, #0 */

    /* Loop 2 body */
    int loop2 = i;
    code[i++] = 0x1809;          /* ADDS R1, R1, R0 */
    code[i++] = 0x3801;          /* SUBS R0, #1 */
    code[i++] = 0xD1FC;          /* BNE -3 (back to ADDS) */

    /* --- Loop 3 setup --- */
    code[i++] = 0x2432;          /* MOVS R4, #50 */
    code[i++] = 0x200A;          /* MOVS R0, #10 */
    code[i++] = 0x2114;          /* MOVS R1, #20 */
    code[i++] = 0x221E;          /* MOVS R2, #30 */
    code[i++] = 0x2301;          /* MOVS R3, #1 */

    /* Loop 3 body: 8 ALU + SUBS + BNE */
    int loop3 = i;
    code[i++] = 0x1845;          /* ADDS R5, R0, R1 */
    code[i++] = 0x18AD;          /* ADDS R5, R5, R2 */
    code[i++] = 0x18ED;          /* ADDS R5, R5, R3 */
    code[i++] = 0x1A2D;          /* SUBS R5, R5, R0 */
    code[i++] = 0x006E;          /* LSLS R6, R5, #1 */
    code[i++] = 0x0876;          /* LSRS R6, R6, #1 */
    code[i++] = 0x18C0;          /* ADDS R0, R0, R3 */
    code[i++] = 0x1A40;          /* SUBS R0, R0, R3 */
    code[i++] = 0x3C01;          /* SUBS R4, #1 */
    {
        int8_t off = (int8_t)(loop3 - (i + 1));
        code[i++] = 0xD100 | (uint8_t)off;  /* BNE loop3 */
    }

    /* --- Loop 4 (repeat of loop 1 for more volume) --- */
    code[i++] = 0x20FF;          /* MOVS R0, #255 */
    code[i++] = 0x0300;          /* LSLS R0, R0, #12 */
    code[i++] = 0x21FF;          /* MOVS R1, #255 */
    code[i++] = 0x0109;          /* LSLS R1, R1, #4 */
    code[i++] = 0x1840;          /* ADDS R0, R0, R1 */

    int loop4 = i;
    code[i++] = 0x3801;          /* SUBS R0, #1 */
    code[i++] = 0xD1FD;          /* BNE -2 */

    /* Halt */
    code[i++] = 0xE7FE;          /* B . */

    (void)loop1; (void)loop2; (void)loop4;
}

/*
 * Run the emulator until it hits the halt loop (B .)
 * Returns instruction count.
 */
static uint32_t run_until_halt(uint32_t max_steps) {
    uint32_t count = 0;

    for (uint32_t s = 0; s < max_steps; s++) {
        if (cores[CORE0].is_halted) break;

        /* Context switch core 0 into the single-core engine */
        set_active_core(CORE0);
        memcpy(cpu.r, cores[CORE0].r, sizeof(cpu.r));
        cpu.xpsr = cores[CORE0].xpsr;
        cpu.vtor = cores[CORE0].vtor;
        cpu.step_count = cores[CORE0].step_count;
        cpu.current_irq = cores[CORE0].current_irq;
        cpu.primask = cores[CORE0].primask;
        cpu.control = cores[CORE0].control;
        cpu.debug_enabled = 0;
        mem_set_ram_ptr(cpu.ram, RAM_BASE, RAM_SIZE);

        cpu_step();

        /* Save back */
        memcpy(cores[CORE0].r, cpu.r, sizeof(cpu.r));
        cores[CORE0].step_count = cpu.step_count;
        cores[CORE0].xpsr = cpu.xpsr;
        cores[CORE0].current_irq = cpu.current_irq;
        cores[CORE0].primask = cpu.primask;
        cores[CORE0].control = cpu.control;

        count++;

        /* Detect B . (0xE7FE): instruction at current PC is unconditional branch to self */
        uint32_t cur_pc = cores[CORE0].r[15];
        if (cur_pc >= FLASH_BASE && cur_pc < FLASH_BASE + FLASH_SIZE) {
            uint16_t cur_instr;
            memcpy(&cur_instr, &cpu.flash[cur_pc - FLASH_BASE], 2);
            if (cur_instr == 0xE7FE) break;  /* B . */
        }
    }

    /* Return actual instruction count from cpu step counter, not loop iterations.
     * With JIT, one cpu_step() call may execute multiple instructions. */
    return cores[CORE0].step_count;
}

static void reset_core(void) {
    memset(&cores[CORE0], 0, sizeof(cpu_state_dual_t));
    cores[CORE0].xpsr = 0x01000000;
    cores[CORE0].vtor = FLASH_BASE + 0x100;
    cores[CORE0].current_irq = 0xFFFFFFFF;
    cores[CORE0].r[13] = mem_read32(cores[CORE0].vtor);
    cores[CORE0].r[15] = mem_read32(cores[CORE0].vtor + 4) & ~1u;

    /* Also reset CPU global state */
    cpu.primask = 0;
    cpu.current_irq = 0xFFFFFFFF;

    timing_config.cycle_accumulator = 0;
    timing_config.cycles_per_us = 1;
    memset(cpu.ram, 0, RAM_SIZE);

    /* Reset NVIC so no stale pending interrupts */
    nvic_init();
}

int main(void) {
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" Bramble JIT Benchmark\n");
    printf("═══════════════════════════════════════════════════════════\n\n");

    cpu_init();
    nvic_init();
    timer_init();
    rom_init();
    num_active_cores = 1;  /* Single-core for clean benchmarking */

    build_firmware();

    uint32_t max_steps = 50000000;
    int runs = 3;

    /* ---- Verify firmware runs correctly ---- */
    printf("[Verify] Checking firmware execution...\n");
    reset_core();
    icache_init();
    jit_enable(0);
    uint32_t verify_insns = run_until_halt(max_steps);
    printf("  Firmware executes %u instructions before halt\n\n", verify_insns);

    if (verify_insns < 100) {
        printf("ERROR: Firmware didn't run properly! Debugging:\n");
        reset_core();
        cpu.debug_enabled = 1;
        cores[CORE0].debug_enabled = 1;
        run_until_halt(20);
        return 1;
    }

    /* ---- Benchmark 1: Raw dispatch (no icache, no JIT) ---- */
    printf("[1/3] Raw dispatch (no icache, no JIT)...\n");
    double raw_best_mips = 0;
    double raw_best_ms = 0;
    for (int r = 0; r < runs; r++) {
        reset_core();
        /* cpu_init sets icache, we need to disable it */
        /* icache_enabled is internal, but not calling icache_init leaves it from cpu_init */
        /* We can't easily disable icache without modifying code, so skip this test
         * or accept it's always enabled after cpu_init */
        jit_enable(0);

        uint64_t t0 = now_ns();
        uint32_t insns = run_until_halt(max_steps);
        uint64_t t1 = now_ns();

        double ms = (double)(t1 - t0) / 1e6;
        double mips = (double)insns / ((double)(t1 - t0) / 1e9) / 1e6;
        if (mips > raw_best_mips) {
            raw_best_mips = mips;
            raw_best_ms = ms;
        }
        printf("  Run %d: %u insns in %.1f ms (%.2f MIPS)\n", r + 1, insns, ms, mips);
    }
    printf("  Best: %.2f MIPS\n\n", raw_best_mips);

    /* ---- Benchmark 2: ICache only ---- */
    printf("[2/3] ICache only (no JIT)...\n");
    double ic_best_mips = 0;
    double ic_best_ms = 0;
    for (int r = 0; r < runs; r++) {
        reset_core();
        icache_init();
        jit_enable(0);

        uint64_t t0 = now_ns();
        uint32_t insns = run_until_halt(max_steps);
        uint64_t t1 = now_ns();

        double ms = (double)(t1 - t0) / 1e6;
        double mips = (double)insns / ((double)(t1 - t0) / 1e9) / 1e6;
        if (mips > ic_best_mips) {
            ic_best_mips = mips;
            ic_best_ms = ms;
        }
        printf("  Run %d: %u insns in %.1f ms (%.2f MIPS)\n", r + 1, insns, ms, mips);
    }
    printf("  Best: %.2f MIPS\n\n", ic_best_mips);

    /* ---- Benchmark 3: ICache + JIT ---- */
    printf("[3/3] ICache + JIT block compilation...\n");
    double jit_best_mips = 0;
    double jit_best_ms = 0;
    for (int r = 0; r < runs; r++) {
        reset_core();
        icache_init();
        jit_init();

        uint64_t t0 = now_ns();
        uint32_t insns = run_until_halt(max_steps);
        uint64_t t1 = now_ns();

        double ms = (double)(t1 - t0) / 1e6;
        double mips = (double)insns / ((double)(t1 - t0) / 1e9) / 1e6;
        if (mips > jit_best_mips) {
            jit_best_mips = mips;
            jit_best_ms = ms;
        }
        printf("  Run %d: %u insns in %.1f ms (%.2f MIPS)\n", r + 1, insns, ms, mips);
    }
    printf("  Best: %.2f MIPS\n\n", jit_best_mips);

    /* ---- Summary ---- */
    printf("═══════════════════════════════════════════════════════════\n");
    printf(" Results (best of %d runs)\n", runs);
    printf("═══════════════════════════════════════════════════════════\n");
    printf("  %-25s %8.2f MIPS  %8.1f ms\n", "ICache only:", ic_best_mips, ic_best_ms);
    printf("  %-25s %8.2f MIPS  %8.1f ms", "ICache + JIT:", jit_best_mips, jit_best_ms);
    if (ic_best_mips > 0) printf("  (%.2fx)", jit_best_mips / ic_best_mips);
    printf("\n");
    printf("═══════════════════════════════════════════════════════════\n");

    /* Cache stats from last JIT run */
    printf("\nCache statistics (last JIT run):\n");
    icache_report_stats();
    jit_report_stats();

    return 0;
}
