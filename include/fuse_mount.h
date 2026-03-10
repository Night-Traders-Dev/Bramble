#ifndef FUSE_MOUNT_H
#define FUSE_MOUNT_H

/* ========================================================================
 * FUSE Filesystem Mount for Flash Storage
 *
 * Mounts the FAT16 filesystem region of emulated flash as a real
 * directory on the host. Changes from either side are immediately
 * visible (FUSE reads/writes operate directly on cpu.flash[]).
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

/* Start FUSE mount in a background thread.
 * flash_data: pointer to flash region containing FAT filesystem
 * flash_size: size of the flash region in bytes
 * mount_point: directory to mount at (must exist)
 * Returns 0 on success, -1 on error. */
int fuse_mount_start(uint8_t *flash_data, size_t flash_size, const char *mount_point);

/* Stop FUSE mount and unmount */
void fuse_mount_stop(void);

/* Check if FUSE mount is active */
int fuse_mount_active(void);

#endif /* FUSE_MOUNT_H */
