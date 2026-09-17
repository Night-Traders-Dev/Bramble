# macOS CYW43 host bridge

`-wifi -tap` on macOS uses **utun** (L3) instead of Linux TAP (L2). Bramble adapts Ethernet frames to/from IPv4.

- Guest DHCP: `192.168.4.2/24`, gateway `192.168.4.1`, DNS `8.8.8.8` (emulator-provided)
- **UDP** (DNS/NTP/TFTP): userspace NAT via host sockets (no pf required)
- **TCP/ICMP**: optional pf NAT:

```bash
sudo ./scripts/macos-cyw43-pf-nat.sh enable
sudo ./scripts/macos-cyw43-pf-nat.sh disable
```

Example:

```bash
./bramble firmware.uf2 -wifi -tap utun
```
