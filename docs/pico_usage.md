# Pico Usage

Pico runs MicroPython. Program deploys as `main.py` — runs automatically on boot.

## From Orin

```bash
# First time - create virtual env if one doesn't already exist
python -m venv .venv

cd ~/dean/Sentinel/firmware/pico_motor_controller
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
