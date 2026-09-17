# Host SPI flash backing

Map one or two SPI NOR images to host files for firmware that talks to external flash:

```bash
./bramble firmware.uf2 -spi-flash1 flash/spi-flash1.bin -spi-flash1-size 64
./bramble firmware.uf2 -spi-flash2 flash/spi-flash2.bin -spi-flash2-size 64
```

Defaults create `flash/spi-flash{1,2}.bin` at 64 MB (sizes are multiples of 32 MB, max 256).
API: `spi_flash_configure` / `spi_flash_set_size` / `spi_flash_read` / `spi_flash_write` / `spi_flash_close`.
