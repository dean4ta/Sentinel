# Pico Usage

Pico runs MicroPython. Program deploys as `main.py` — runs automatically on boot.

## From Orin

```bash
cd firmware/pico_motor_controller
```

Deploy (program auto-starts on next reset):

```bash
cmake --build build
# put pico into USEL mode by pressing button while turning on
picotool load build/pico_motor_controller.uf2 -v
picotool reboot
```

## Stable device path (udev)

`/dev/ttyACM0` drifts across replugs. Fix with a udev symlink `/dev/sentinel_pico`, matched to this board's per-chip serial.

Gather identifying info:

```bash
udevadm info -n /dev/ttyACM0           # ID_VENDOR_ID, ID_MODEL_ID, ID_SERIAL_SHORT
udevadm info -a -n /dev/ttyACM0        # ATTRS{idVendor}, ATTRS{idProduct}, ATTRS{serial}
```

Sentinel 1 Pico: `idVendor=2e8a`, `idProduct=0005`, `serial=e660d4a0a7695b37`.

Create rule:

```bash
sudo nano /etc/udev/rules.d/99-sentinel-pico.rules
```

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", ATTRS{serial}=="e660d4a0a7695b37", SYMLINK+="sentinel_pico", MODE="0666"
```

Apply:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
# unplug + replug Pico
ls -l /dev/sentinel_pico   # -> /dev/ttyACM0
```

Point config at `/dev/sentinel_pico` (`config/drive_params.yaml`), or use in ad-hoc commands: `mpremote connect /dev/sentinel_pico ...`.

No-rule alternative: systemd already symlinks `/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e660d4a0a7695b37-if00` — stable too, but the name drifts if MicroPython changes the product string.

## Interactive test

Protocol (see `pico_led_serial.py` header): one line per command.

- `15` = LED on for 15 x 10 ms, off 200 ms, repeat
- `fl,fr,bl,br` = float wheel line, -1..1 each (pulse 1100–1900 µs, 0 = neutral).
  `0,0,0,0` = all neutral. The Pico's 200 ms failsafe goes neutral if the
  line stops repeating — send at > 5 Hz for sustained motion.

ROS way (from Orin, wheels on blocks): publish the debug topic:

```bash
ros2 topic pub -r 20 /wheel_cmd std_msgs/msg/String "data: 'FR 30'"   # FR at 30 rpm
```

Wheel rpm values, not -1..1 — `cmd_vel_serial` maps them. One wheel per
command, rest zeroed, overrides `/cmd_vel` while fresh.

Raw serial way (firmware-level check, no ROS): write the line directly
to the port. Stop `cmd_vel_serial` first — one process per port.

```bash
python3 -c "import serial; serial.Serial('/dev/sentinel_pico').write(b'0.2,0,0,0\n')"
```

For a sustained wheel command, repeat the write in a loop (~20 Hz).

## Gotchas

- One process per serial port. Two programs (mpremote + sender) on same port = `Device not configured` errors.
- Pico has no power LED. No light ≠ dead.
- Dead Pico? Hold BOOTSEL, plug in — mounts as `RPI-RP2` drive.
- Flaky connection = suspect cable first. Some micro-USB cables are charge-only.
