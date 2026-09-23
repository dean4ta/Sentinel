# Pico Usage

Pico runs MicroPython. Program deploys as `main.py` — runs automatically on boot.

## From Orin

```bash
cd ~/dean/Sentinel/firmware/pico_motor_controller
python -m venv .venv  # First time - create virtual env if one doesn't already exist
source .venv/bin/activate

# find device — Pico is ttyACM*
ls /dev/ttyACM*
```

Deploy (program auto-starts on next reset):

```bash
mpremote connect /dev/ttyACM0 cp pico_led_serial.py :main.py
mpremote connect /dev/ttyACM0 reset
```

Run without flashing (one-off test, Ctrl+C or kill to stop):

```bash
mpremote connect /dev/ttyACM0 run pico_led_serial.py
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

Terminal 2, after `main.py` is running on the Pico:

```bash
# no virtualenv needed this step
python3 led_sender.py
```

Commands: `15` = LED on 150ms. `x128`/`y128`/`r128` ≈ neutral PWM, `0`–`255` full range.

## Gotchas

- One process per serial port. Two programs (mpremote + sender) on same port = `Device not configured` errors.
- Pico has no power LED. No light ≠ dead.
- Dead Pico? Hold BOOTSEL, plug in — mounts as `RPI-RP2` drive.
- Flaky connection = suspect cable first. Some micro-USB cables are charge-only.
