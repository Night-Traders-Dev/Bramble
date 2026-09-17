#ifndef BRAMBLE_EXT_H
#define BRAMBLE_EXT_H

/*
 * Optional in-process extension hooks (weak stubs in stock Bramble).
 * megaflash-vm overlay provides strong definitions for Apple-bus / MegaFlash.
 */

/* Return 1 if argv[*argi] was consumed (may advance *argi). */
int bramble_ext_parse_arg(int *argi, int argc, char **argv);

/* After symbols/UF2 load — start bridges, resolve BSS, etc. */
void bramble_ext_post_init(const char *symbols_path);

/* Nonzero while an interactive extension session should disable safety exits. */
int bramble_ext_active(void);

void bramble_ext_poll(void);
void bramble_ext_cleanup(void);

/* Per-guest-instruction hook. Return 1 if PC was handled. */
int bramble_ext_guest_hook(void);

/*
 * Script extensions: parse returns 1 and fills type (>= 100 for ext types).
 * run returns 1 if type was handled.
 */
int bramble_ext_script_parse(const char *cmd, const char *arg,
                             int *type, int *channel, int *gpio_val);
int bramble_ext_script_run(int type, int channel, int gpio_val);

/* Wall-clock script timing while extension active (MAME sessions). */
int bramble_ext_script_use_wallclock(void);

void bramble_ext_print_help(void);

#endif /* BRAMBLE_EXT_H */
