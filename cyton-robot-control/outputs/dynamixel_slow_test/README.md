# DYNAMIXEL slow movement test

This sketch tests the four detected servos (IDs 2, 3, 20, and 21) one at a
time. Each servo is limited to roughly 10% torque and a low Protocol 1.0 moving
speed, moves 5 degrees from its measured position, returns, and is left with
torque disabled. It changes RAM settings only, not EEPROM.

Keep clear of the mechanism and be ready to switch off DYNAMIXEL power.

1. Upload with SW2 in UPLOAD.
2. Move SW2 to DYNAMIXEL and press Reset.
3. Allow about 30 seconds. When the Uno L LED is solid, all attempts are over
   and every tested servo has been commanded torque-off.
4. Move SW2 to UPLOAD to retrieve the repeated report at 115200 baud.
