// Pi Pico (RP2040) Serial-to-PWM Motor Controller
//
// Ported from the Sentinel 1 ESP32 firmware (see firmware/reference/ESP32_code.txt).
// Changes from ESP32 version:
//   - No LED strip code (strip removed from robot, not coming back).
//   - STATE,* lines still parse (must not count as motor commands) but are no-ops.
//   - PWM via RP2040 analogWriteFreq/analogWriteResolution instead of ledc.
//   - USB serial shows up as /dev/ttyACM0 on the Orin, NOT /dev/ttyUSB1.
//     pwm_serial_transmitter.py must be updated to match.
//
// Receives movement commands from the Jetson Orin over USB serial and converts
// them into 50 Hz servo-style PWM signals for the goBILDA mecanum mixer.
//
// Expected motor command format:
//   drive,rotate,strafe\n
//   Example: 0.25,-0.10,0.50
//
// Expected state command format (accepted, ignored):
//   STATE,BOOT
//   STATE,READY
//   STATE,SAFE
//   STATE,FAULT
//
// The Orin is responsible for joystick processing and deadzone filtering.
// The Pico is responsible only for PWM output, validation, direction
// correction, rotation limiting, and failsafe behavior.

// -------------------- PWM output pins (RP2040 GPIO) --------------------

const int DRIVE_PIN  = 18;
const int ROTATE_PIN = 17;
const int STRAFE_PIN = 16;

// -------------------- PWM signal settings --------------------

const int PWM_FREQ = 50;
const int PWM_RES  = 16;

const int MIN_US     = 1000;
const int NEUTRAL_US = 1500;
const int MAX_US     = 2000;

// -------------------- Movement adjustments --------------------

const bool INVERT_STRAFE = true;
const float ROTATE_LIMIT = 0.5;

// -------------------- Serial and failsafe settings --------------------

const unsigned long FAILSAFE_MS = 200;
unsigned long lastMotorCommandTime = 0;

String currentLine = "";
String newestCompleteLine = "";

bool failsafeActive = false;

// -------------------- PWM helper functions --------------------

int pulseToDuty(int pulse_us) {
  const int period_us = 1000000 / PWM_FREQ;
  const int maxDuty = (1 << PWM_RES) - 1;

  pulse_us = constrain(pulse_us, MIN_US, MAX_US);

  return (int)((pulse_us / (float)period_us) * maxDuty);
}

int commandToPulse(float value) {
  value = constrain(value, -1.0, 1.0);

  int pulse = NEUTRAL_US + (int)(value * 500.0);

  return constrain(pulse, MIN_US, MAX_US);
}

void writePWM(int pin, int pulse_us) {
  analogWrite(pin, pulseToDuty(pulse_us));
}

void neutralAll() {
  writePWM(DRIVE_PIN, NEUTRAL_US);
  writePWM(ROTATE_PIN, NEUTRAL_US);
  writePWM(STRAFE_PIN, NEUTRAL_US);
}

// -------------------- Command validation --------------------

bool isValidNumber(String text) {
  text.trim();

  if (text.length() == 0) {
    return false;
  }

  bool decimalSeen = false;
  bool digitSeen = false;

  for (int i = 0; i < text.length(); i++) {
    char c = text.charAt(i);

    if (c >= '0' && c <= '9') {
      digitSeen = true;
    }
    else if (c == '.' && !decimalSeen) {
      decimalSeen = true;
    }
    else if (c == '-' && i == 0) {
      // Negative sign is allowed only at the start
    }
    else {
      return false;
    }
  }

  return digitSeen;
}

// -------------------- Command parsing --------------------

bool parseAndApplyCommand(String line) {
  line.trim();

  if (line.length() == 0) {
    return false;
  }

  // State commands do not count as motor commands.
  // This prevents state messages from keeping the drive system alive
  // if motor commands stop.
  if (line.startsWith("STATE")) {
    return false;
  }

  int firstComma = line.indexOf(',');
  int secondComma = line.indexOf(',', firstComma + 1);

  if (firstComma == -1 || secondComma == -1) {
    return false;
  }

  String driveText  = line.substring(0, firstComma);
  String rotateText = line.substring(firstComma + 1, secondComma);
  String strafeText = line.substring(secondComma + 1);

  if (!isValidNumber(driveText) || !isValidNumber(rotateText) || !isValidNumber(strafeText)) {
    return false;
  }

  float drive  = driveText.toFloat();
  float rotate = rotateText.toFloat();
  float strafe = strafeText.toFloat();

  // Reject obviously bad values instead of trusting them.
  // Slightly out-of-range values are still clamped later, but values this far
  // out usually mean a corrupted serial line or a software bug upstream.
  if (drive < -1.2 || drive > 1.2 ||
      rotate < -1.2 || rotate > 1.2 ||
      strafe < -1.2 || strafe > 1.2) {
    return false;
  }

  // Correct strafe direction.
  if (INVERT_STRAFE) {
    strafe = -strafe;
  }

  // Limit rotation to half speed.
  rotate = constrain(rotate, -ROTATE_LIMIT, ROTATE_LIMIT);

  writePWM(DRIVE_PIN, commandToPulse(drive));
  writePWM(ROTATE_PIN, commandToPulse(rotate));
  writePWM(STRAFE_PIN, commandToPulse(strafe));

  lastMotorCommandTime = millis();
  failsafeActive = false;

  return true;
}

// -------------------- Serial reading --------------------

void readSerialNewestOnly() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n') {
      newestCompleteLine = currentLine;
      currentLine = "";
    }
    else if (c != '\r') {
      currentLine += c;

      // Prevent corrupted or oversized serial lines from growing forever.
      if (currentLine.length() > 80) {
        currentLine = "";
      }
    }
  }

  // Only apply the newest complete command.
  // This prevents old joystick commands from building up and causing lag.
  if (newestCompleteLine.length() > 0) {
    parseAndApplyCommand(newestCompleteLine);
    newestCompleteLine = "";
  }
}

// -------------------- Setup --------------------

void setup() {
  Serial.begin(460800);

  // RP2040 PWM: per-pin frequency, shared resolution.
  analogWriteFreq(PWM_FREQ);
  analogWriteResolution(PWM_RES);
  pinMode(DRIVE_PIN, OUTPUT);
  pinMode(ROTATE_PIN, OUTPUT);
  pinMode(STRAFE_PIN, OUTPUT);

  neutralAll();

  lastMotorCommandTime = millis();
  failsafeActive = false;

  Serial.println("Pico motor controller ready");
}

// -------------------- Main loop --------------------

void loop() {
  readSerialNewestOnly();

  if (millis() - lastMotorCommandTime > FAILSAFE_MS) {
    if (!failsafeActive) {
      neutralAll();
      failsafeActive = true;
    }
  }
}

// -------------------- Verification --------------------
//
// Bench test before installing:
//   1. Oscilloscope/logic analyzer on GP18/17/16: idle = 1500us pulses @ 50Hz.
//   2. Send "0.0,0.0,0.5\n" over USB: GP16 should go to 1750us (strafe +),
//      GP18/GP17 stay 1500us.
//   3. Stop sending: all return to 1500us within 200ms (failsafe).
//   4. On Orin: device must appear as /dev/ttyACM0.
