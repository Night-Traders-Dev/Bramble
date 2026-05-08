---
title: "Bramble Emulator: Technical Reference Manual"
author: "Development Documentation"
date: \today
---

# Introduction {.unnumbered}

Bramble is a high-fidelity development and debugging tool for the Raspberry Pi
RP2040 and RP2350 microcontroller ecosystem. Unlike high-level simulators,
Bramble focuses on register-level accuracy, allowing unmodified firmware
binaries---including those from the Pico SDK, MicroPython, CircuitPython, and
littleOS---to execute in a controlled, virtualized environment.

The emulator supports **tri-architecture** execution:

- **RP2040 Cortex-M0+** (ARMv6-M Thumb-1, 65+ instructions)
- **RP2350 Cortex-M33** (ARMv8-M Mainline, full Thumb-2 ISA)
- **RP2350 Hazard3 RISC-V** (RV32IMAC, 93 instructions)

Architecture is auto-detected from UF2 family ID, ELF machine type, or RP2350
picobin IMAGE\_DEF blocks. All three architectures share a unified peripheral
infrastructure covering 30+ emulated hardware modules.

This manual serves as the primary technical reference for the emulator's
architecture, instruction set coverage, peripheral implementation, boot
sequences, debugging facilities, and developer tooling.

**Version 0.43.0** --- 300 tests, 22,663 source lines, zero compiler warnings.
