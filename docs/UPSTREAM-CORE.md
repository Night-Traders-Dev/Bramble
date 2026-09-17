# Core emulator improvements (eositis → Night-Traders)

This PR packages RP2350/M33 and host-runtime fixes that are independent of
Apple/MegaFlash/a2bus:

- Thumb-2 / instruction decode improvements (`thumb32`, `instructions`)
- Dual-core / WFI host threading (`corepool`)
- DMA BSWAP / NVIC / PIO bounds and inject helpers
- UART / netbridge console polish
- Weak empty `bramble_ext_*` stubs for optional out-of-tree overlays
- SPI peripheral idle/MISO fidelity (`spi.c`)

**Not included here:** CYW43/TAP (separate radio PR), `-spi-flash*` host files
(separate SPI PR), `-usb-console` (separate USB PR), MegaFlash guest stubs.

**Note:** `src/thumb32.c` also diverged on upstream v0.46 (unknown-op no-ops).
Please reconcile with that change when merging.
