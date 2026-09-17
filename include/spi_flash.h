#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#include <BaseTdefs.h>
typedef SSIZE_T ssize_t;
#else
#include <sys/types.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Configure SPI flash chip backing (path NULL/empty → default file). */
void spi_flash_configure(unsigned chip, const char *path);

/* Set chip capacity in MB (rounded up to a multiple of 32, max 256). */
void spi_flash_set_size(unsigned chip, uint32_t size_mb);

/* Close all chips and release backing resources. */
void spi_flash_close(void);

/* Read len bytes at offset. Returns bytes read, or -1 on error. */
ssize_t spi_flash_read(unsigned chip, uint64_t offset, void *buf, size_t len);

/* Write len bytes at offset. Returns bytes written, or -1 on error. */
ssize_t spi_flash_write(unsigned chip, uint64_t offset, const void *buf, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* SPI_FLASH_H */
