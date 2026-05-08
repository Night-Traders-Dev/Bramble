/*
 * RISC-V Instruction Cache for Hazard3
 *
 * Direct-mapped decoded instruction cache for RV32 execution.
 * Caches the fetched instruction word indexed by PC, avoiding
 * repeated memory bus reads for flash/ROM instruction fetches.
 *
 * Only caches flash and ROM addresses (immutable code).
 * SRAM instruction fetches bypass the cache.
 */

#ifndef RV_ICACHE_H
#define RV_ICACHE_H

#include <stdint.h>

#define RV_ICACHE_SIZE  (64 * 1024)  /* 64K entries */
#define RV_ICACHE_MASK  (RV_ICACHE_SIZE - 1)

typedef struct {
    uint32_t tag;    /* PC address (0 = invalid) */
    uint32_t instr;  /* Cached instruction (16 or 32 bit, stored in low bits) */
    uint8_t  size;   /* 2 = compressed, 4 = 32-bit */
} rv_icache_entry_t;

typedef struct {
    rv_icache_entry_t entries[RV_ICACHE_SIZE];
    uint64_t hits;
    uint64_t misses;
} rv_icache_t;

static inline void rv_icache_init(rv_icache_t *cache) {
    for (uint32_t i = 0; i < RV_ICACHE_SIZE; i++) {
        cache->entries[i].tag = 0xFFFFFFFF;
        cache->entries[i].instr = 0;
        cache->entries[i].size = 0;
    }
    cache->hits = 0;
    cache->misses = 0;
}

/* Lookup instruction at PC. Returns 1 if hit, 0 if miss. */
static inline int rv_icache_lookup(rv_icache_t *cache, uint32_t pc,
                                    uint32_t *instr, uint8_t *size) {
    uint32_t idx = (pc >> 1) & RV_ICACHE_MASK;
    rv_icache_entry_t *e = &cache->entries[idx];
    if (e->tag == pc) {
        *instr = e->instr;
        *size = e->size;
        cache->hits++;
        return 1;
    }
    cache->misses++;
    return 0;
}

/* Insert instruction into cache (only for flash/ROM addresses) */
static inline void rv_icache_insert(rv_icache_t *cache, uint32_t pc,
                                     uint32_t instr, uint8_t size) {
    uint32_t idx = (pc >> 1) & RV_ICACHE_MASK;
    rv_icache_entry_t *e = &cache->entries[idx];
    e->tag = pc;
    e->instr = instr;
    e->size = size;
}

#endif /* RV_ICACHE_H */
