#include "bramble_ext.h"

__attribute__((weak)) int bramble_ext_parse_arg(int *argi, int argc, char **argv)
{
    (void)argi;
    (void)argc;
    (void)argv;
    return 0;
}

__attribute__((weak)) void bramble_ext_post_init(const char *symbols_path)
{
    (void)symbols_path;
}

__attribute__((weak)) int bramble_ext_active(void)
{
    return 0;
}

__attribute__((weak)) void bramble_ext_poll(void)
{
}

__attribute__((weak)) void bramble_ext_cleanup(void)
{
}

__attribute__((weak)) int bramble_ext_guest_hook(void)
{
    return 0;
}

__attribute__((weak)) int bramble_ext_script_parse(const char *cmd, const char *arg,
                                                   int *type, int *channel, int *gpio_val)
{
    (void)cmd;
    (void)arg;
    (void)type;
    (void)channel;
    (void)gpio_val;
    return 0;
}

__attribute__((weak)) int bramble_ext_script_run(int type, int channel, int gpio_val)
{
    (void)type;
    (void)channel;
    (void)gpio_val;
    return 0;
}

__attribute__((weak)) int bramble_ext_script_use_wallclock(void)
{
    return 0;
}

__attribute__((weak)) void bramble_ext_print_help(void)
{
}

/* Defined here so instructions.c WFI/WFE tracing links without full cpu.c fork. */
int multicore_trace_enabled = 0;
