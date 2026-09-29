# Pico high-speed lockup — debug plan

Status: in progress (2026-09-29)

## Symptom

Driving all four wheels fast (~1.2 m/s threshold, varies per wheel) kills the
Pico: heartbeat LED freezes on or off, encoder stream stops, motors hold last
commanded speed. Unplugging the encoder wires makes the Pico survive the same
speeds. A 2.2 µF capacitor added from each encoder line to ground destroyed
the encoder signal (tried, reverted).

## Working diagnosis

Not a code-induced noise source, but the code converts electrical noise into a
total lockup:

- Encoder A/B lines carry brush/ESC switching noise that rises with speed and
  motor current. Slow Hall edges + weak internal pull-ups (~50 kΩ) chatter
  around the pin threshold → spurious edges.
- The Pico decodes quadrature with **Python soft-IRQ callbacks on every edge**
  (8 pins, both edges, `pico_led_serial.py`). Chatter turns hundreds of real
  edges/s into a callback storm the interpreter can't drain.
- The main loop (serial read, failsafe, LED, report) starves. Hardware PWM
  slices keep running on their own → motors hold last pulse. LED frozen. No
  serial output. This matches the symptom exactly.
- The 200 ms failsafe lives inside the starved loop, so it cannot fire.
- There is no watchdog and no Ctrl-C byte guard: any crash or stray `0x03` on
  stdin kills the script while PWM keeps running.

2.2 µF caps formed a ~1 Hz low-pass with the internal pull-up — they filtered
out the real 370–700 Hz signal, hence the dead encoder ticks. Any filter here
must have a corner of tens of kHz (e.g. 1 kΩ + 4.7 nF ≈ 34 kHz).

## Eliminated so far

- [x] Encoder over-voltage: encoder VCC measured **3.24 V**, powered from the
      Pico's 3V3. No 5 V on RP2040 pins.

## Phase 0 — safety first

Goal: a locked-up Pico must reset itself and the robot must stop.

- [x] Firmware: `micropython.kbd_intr(-1)` so a stray 0x03 byte cannot drop
      the script to the REPL while motors spin. (2026-09-29)
- [x] Firmware: hardware watchdog (`machine.WDT`, 500 ms) fed in the main
      loop. Wedged CPU → watchdog reset → PWM pins float. (2026-09-29)
- [x] Hardware: pull-downs on the level-shifter PWM lines **do not work** —
      the YF08E (TXS0108E-class, auto-sensing) forbids external pull-ups /
      pull-downs on signal lines; they break its edge-detection. Tried 10 kΩ
      and 12 kΩ, both killed the PWM highs. Removed.
- [ ] Hardware: verify the GoBilda 1x20A controller's own failsafe instead —
      **wheels off the ground**, run at speed, kill the Pico (power-cycle or
      `mpremote reset`), motors must stop within ~1 s. If they don't, revisit
      (OE gate / motor-power relay).

Owner: Dean (firmware), hardware check at robot.

## Phase 1 — split firmware vs electrical

Goal: determine whether the lockup is IRQ starvation (firmware) or an
electrical fault (noise/latch-up). No instruments needed.

- [ ] Test A: keep encoders wired, but disable all encoder IRQs after
      registration (`p.irq(handler=None)`). Run wheels at high speed.
      - Survives → IRQ starvation confirmed, latch-up mostly ruled out.
      - Still hangs → electrical path (encoder supply, signal levels, ground
        topology).
- [ ] Test B: add an IRQ-callback counter per wheel, printed in the 50 Hz
      report. At steady speed, callbacks/s ≈ |counts/s|. Ratio climbing with
      speed = chatter, and it should spike just before a hang. Graph this.

Owner: Dean writes instrumented firmware, user runs on robot.

## Phase 2 — fixes

- [ ] Firmware: move quadrature decode from IRQ callbacks to **PIO** (4 state
      machines, zero interrupts). A/B pins are already consecutive pairs
      (6/7, 8/9, 10/11, 12/13), which the standard PIO quadrature program
      expects. Noise can then add bogus counts but can never freeze the CPU.
- [ ] Electrical: 1 kΩ series + 4.7 nF to ground at each Pico encoder pin
      (corner ≈ 34 kHz); 100 nF ceramic across each motor's terminals; 100 nF
      at encoder VCC/GND at the Pico end.
- [ ] Electrical: route encoder cables away from motor leads and the ESC;
      twist or shield them. Single ground point for Pico/ESC/Orin; consider a
      USB isolator on the Orin link if the ground loop persists.

## Phase 3 — verify

- [ ] Scope or 24 MHz+ logic analyzer on encoder A/B at the Pico pins, one
      capture just below and one just above the failure speed: edge quality,
      glitch bursts, plus the Pico 3V3 rail AC-coupled.
- [ ] Confirm the Test B chatter ratio returns to ~1 after PIO + filtering.
- [ ] High-speed floor drive: no freeze, watchdog silent, counts clean.

## Notes

- Root-cause reference: TODO `work_logs/TODO` — "Investigate why the pico cuts
  out when the wheels spin fast. (~1 m/s for FL and ~1.5 m/s for FR)".
- Prior observation (2026-09-24): single-wheel step tests above ~100 rpm were
  noisy but the Pico survived. The four-wheel floor case is ~4× the edge rate
  plus mechanical load → more brush noise. Consistent with the diagnosis.
