#ifndef FUSE_MOUNT_H
#define FUSE_MOUNT_H

/* ========================================================================
 * FUSE Filesystem Mount for Flash Storage
 *
 * Mounts the FAT16 filesystem region of emulated flash as a real
 * directory on the host. Changes from either side are immediately
 * visible (FUSE reads/writes operate directly on cpu.flash[]).
 *
 * Thread safety: fuse_flash_mutex must be held when accessing flash
 * memory from outside the emulator's normal execution path (e.g.,
 * ROM flash_range_erase/program should lock this mutex when FUSE is
 * active to prevent torn reads/writes).
 *
 * Requires libfuse3. Optional feature enabled via CMake:
 *   cmake .. -DENABLE_FUSE=ON
 *
 * Usage:
 *   ./bramble firmware.uf2 -flash fs.bin -mount /tmp/pico
 *
 * Then in another terminal:
 *   ls /tmp/pico/
 *   echo 'print("hello")' > /tmp/pico/code.py
 * ======================================================================== */

#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

/* Mutex protecting flash array between FUSE thread and emulator.
 * Lock this before ROM flash writes when FUSE mount is active. */
extern pthread_mutex_t fuse_flash_mutex;

/* Set the flash offset for persistence sync (call before fuse_mount_start).
 * This is the byte offset from cpu.flash[0] to the filesystem region. */
void fuse_set_flash_offset(uint32_t offset);

/* Start FUSE mount in a background thread.
 * flash_data: pointer to flash region containing FAT filesystem
 * flash_size: size of the flash region in bytes
 * mount_point: directory to mount at (created if needed)
 * Returns 0 on success, -1 on error. */
int fuse_mount_start(uint8_t *flash_data, size_t flash_size, const char *mount_point);

/* Stop FUSE mount and unmount */
void fuse_mount_stop(void);

/* Check if FUSE mount is active */
int fuse_mount_active(void);

#endif /* FUSE_MOUNT_H */
