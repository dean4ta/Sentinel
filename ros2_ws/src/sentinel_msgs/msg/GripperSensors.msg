# Gripper analog sensors, raw 12-bit ADC counts from the motor-controller
# Pico (0-4095, ~0-3.3 V; v = count * 3.3 / 4096). Published at 10 Hz.
# hinge = purple wire (GP27), anvil = blue wire (GP26).
uint16 hinge
uint16 anvil
