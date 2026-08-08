# USB CDC host console

Bidirectional console over the emulated USB CDC device:

```bash
./bramble firmware.uf2 -usb-console 9999
nc localhost 9999

./bramble firmware.uf2 -usb-console pty
screen /tmp/bramble-usb-console 115200
```

`-usb-serial [path]` is an alias for `-usb-console pty[:path]`.
