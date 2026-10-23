# USB udev Rules (Sentinel 1)

Fixed `/dev/sentinel_*` names so launch files never depend on USB
enumeration order. Rule file lives in the repo:

```
ros2_ws/src/sentinel_bringup/config/udev/99-sentinel-usb.rules
```

## Install (on the Orin)

Re-run after every change to the rule file:

```bash
sudo cp ~/dean/Sentinel/ros2_ws/src/sentinel_bringup/config/udev/99-sentinel-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

## Verify

```bash
ls -l /dev/sentinel_*
```

Expected symlinks:

| Symlink | Device | Match |
|---|---|---|
| `/dev/sentinel_pico` | Pico motor controller | VID:PID 2e8a:000a + serial |
| `/dev/sentinel_lidar` | RPLIDAR A1 (CP2102) | CP2102 not the arm's serial |
| `/dev/sentinel_webcam` | Logitech C270 | VID:PID 046d:0825, capture node |
| `/dev/sentinel_arm` | RoArm-M2-S (CP2102N) | programmed serial 1251dd7e |

If a symlink is missing, find the device:

```bash
lsusb                       # is the device even there?
ls -l /dev/serial/by-id/    # by-id names include chip + serial
udevadm info -a -n /dev/ttyUSB0 | grep -E "idVendor|idProduct|serial|KERNELS"
```

`udevadm info -a` prints every level of the device tree — match rules
against attributes in any one level. Rules test `ATTRS{...}` (any level);
`ATTR{...}` tests only the leaf node (e.g. `ATTR{index}=="0"` on video).

Note: lidar rule matches by exclusion (`serial !=` arm's) because both
lidar adapters are CP2102s with the default serial "0001". If a second
lidar ever comes back, its adapter needs a distinct programmed serial
first (cp210x-program) or the rule needs port paths again.
