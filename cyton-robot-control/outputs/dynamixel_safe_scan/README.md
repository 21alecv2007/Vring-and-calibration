# Safe DYNAMIXEL discovery scan

This sketch is for an Arduino Uno with the ROBOTIS DYNAMIXEL Shield. It only
sends DYNAMIXEL `PING` instructions. It does not enable torque, change EEPROM,
or request motion.

For original MX-28 units, the scan includes the factory-default Protocol 1.0,
57,600 baud combination. It also checks 1 Mbps and Protocol 2.0 in case the
servos were reconfigured or upgraded.

## Before powering the servos

1. Read the exact model number and voltage from at least one servo label.
   `T`/`AT` models use the 3-pin TTL connector; `R`/`AR` models use the 4-pin
   RS-485 connector.
2. Confirm the external supply matches that voltage. USB power is not a safe
   assumption for a servo chain.
3. Keep the mechanism clear and supported. A servo may already have torque
   enabled from earlier firmware until power is removed.

## Run the scan

1. In Arduino IDE, install **DynamixelShield** from Library Manager and accept
   its **Dynamixel2Arduino** dependency.
2. Open `dynamixel_safe_scan.ino`.
3. Set the shield's **SW2 / UART** switch to **UPLOAD**.
4. Select **Arduino Uno** and **COM3**, then upload.
5. Close Serial Monitor if it is open.
6. Set SW2 to **DYNAMIXEL**, ensure the servos have the correct external power,
   and press the Uno's RESET button once.
7. Wait. The built-in LED toggles as baud rates are scanned. When it stays
   solid, scanning is finished. It can take roughly a minute or more.
8. Set SW2 back to **UPLOAD** and open Serial Monitor at **115200 baud**.
9. Copy the complete report beginning with `=== DYNAMIXEL SAFE SCAN ===`.

If the report says `NO SERVOS FOUND`, do not try a movement sketch yet. Check
the power switch, supply voltage/current, cable orientation, connector type
(TTL vs RS-485), and whether the chain contains duplicate IDs.
