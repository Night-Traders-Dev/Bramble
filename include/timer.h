#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

/* Timer Base Address */
#define TIMER_BASE          0x40054000

/* Timer Registers */
#define TIMER_TIMEHW        (TIMER_BASE + 0x00)  /* Write high word */
#define TIMER_TIMELW        (TIMER_BASE + 0x04)  /* Write low word */
#define TIMER_TIMEHR        (TIMER_BASE + 0x08)  /* Read high word */
#define TIMER_TIMELR        (TIMER_BASE + 0x0C)  /* Read low word */
#define TIMER_ALARM0        (TIMER_BASE + 0x10)  /* Alarm 0 */
#define TIMER_ALARM1        (TIMER_BASE + 0x14)  /* Alarm 1 */
#define TIMER_ALARM2        (TIMER_BASE + 0x18)  /* Alarm 2 */
#define TIMER_ALARM3        (TIMER_BASE + 0x1C)  /* Alarm 3 */
#define TIMER_ARMED         (TIMER_BASE + 0x20)  /* Armed alarms */
#define TIMER_TIMERAWH      (TIMER_BASE + 0x24)  /* Raw high word */
#define TIMER_TIMERAWL      (TIMER_BASE + 0x28)  /* Raw low word */
#define TIMER_DBGPAUSE      (TIMER_BASE + 0x2C)  /* Debug pause */
#define TIMER_PAUSE         (TIMER_BASE + 0x30)  /* Pause timer */
#define TIMER_INTR          (TIMER_BASE + 0x34)  /* Raw interrupt status */
#define TIMER_INTE          (TIMER_BASE + 0x38)  /* Interrupt enable */
#define TIMER_INTF          (TIMER_BASE + 0x3C)  /* Interrupt force */
#define TIMER_INTS          (TIMER_BASE + 0x40)  /* Interrupt status */

/* Timer state */
typedef struct {
    uint64_t time_us;        /* Current time in microseconds */
    uint32_t alarm[4];       /* 4 alarm compare values */
    uint32_t armed;          /* Which alarms are armed */
    uint32_t intr;           /* Interrupt status */
    uint32_t inte;           /* Interrupt enable */
    uint32_t paused;         /* Timer paused flag */
} timer_state_t;

/* Functions */
void timer_init(void);
void timer_reset(void);
void timer_tick(uint32_t cycles);  /* Update timer based on CPU cycles */
uint32_t timer_read32(uint32_t addr);
void timer_write32(uint32_t addr, uint32_t val);

extern timer_state_t timer_state;

#endif /* TIMER_H */
