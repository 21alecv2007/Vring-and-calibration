# VRING 2
Designed for use in AR/VR systems, the VRING is meant to provide a convenient, all-day (if you wish), wireless, finger-worn input device for anything you can dream of.  Currently based on the Xiao NRF52840 platform, it has the following capabilities:
* 6-axis IMU (gyro, accelerometer)
* PDM Microphone
* Temperature sensor

With minor modifications, it adds:

* Capacitive inputs (analog, to some extent) (1 per resistor)
* Vibration output (with the addition of a transistor and vibration motor)

The current software publishes the folowing characteristics at 100Hz, driven off the microphone reading:

* Sensor Data
  * Accelerometer X,Y,Z (2 bytes each, int16 scaled to the +/-16g range)
  * Orientation quaternion X,Y,Z (2 bytes each, int16 scaled by 32767).  W is omitted and
    reconstructed by the receiver as sqrt(1 - x^2 - y^2 - z^2); the firmware negates X,Y,Z
    when needed so W is always positive.
  * Capacitive Readings x 2 (2 bytes each)
  * Battery Reading (2 bytes)
  * Temperature Reading (2 bytes)
  * Sample counter (2 bytes, uint16, wraps every ~164s).  Counts every IMU sample set pulled
    from the FIFO, including sets drained but not transmitted.  The FIFO rate is a fixed 400Hz,
    so sample time is `counter * 2.5ms` exactly.  BLE delivers 2-3 notifications per connection
    interval, so packets arrive in bursts and arrival time is not sample time - use this field
    to place samples in time.  A jump greater than 1 is data the receiver never got.  Appended
    last, so the first 20 bytes keep their original offsets.
* Microphone Data - Opus-encoded 16Khz with 160ms frame size (10ms).  
* Vibration Control - 1 byte, indicating the vibration strength 0 - 255

When disconnected and only advertising, it consumes ~ 100uA, and so for a 100 mAh battery (suggested), you get about 1000hrs standby time (roughly 1.5months).  

When connected, it consumes 6-8 mA, yielding an expected continuous input of about 12 hours.

Battery recharges in about 1 hr of the USB-C port.  

The included HTML test page shows how to read all sensors (use Chrome, which has WebBLE).  

![PXL_20250527_182906354 MP](https://github.com/user-attachments/assets/aa3caed2-5041-4974-8955-8c3e75270c0c)

### Pin connections

To avoid extra bulk, everything is soldered directly to the Xiao.  

* Pin 0 -> MOSFET Gate (2N7000)
* Pin 1 -> GND and MOSFET source (wasted for stability and convenience)
* MOSFET Drain -> Vibration Motor +
* Vibration Motor - VBat 
* 1-MOhm resistors between pins 4 and 9 and 5 and 7 for capacitive sensing
* 100mah battery hooked up underneath (directly soldered) to vbat +/-
* Velcro strap hot glued to seal everything in place and make for an adjustable finger connection

### Building

You will need a few libraries:

* Follow the Xiao guide for getting a basic sketch up and runnning: https://wiki.seeedstudio.com/XIAO_BLE/
  * Note, use the Bluefruit core, non-mbed.  Mbed is very high power, and doesn't offer much.
* Install arduino-libopus: https://github.com/pschatzmann/arduino-libopus
* Follow the guide here to install the IMU library: https://wiki.seeedstudio.com/XIAO-BLE-Sense-IMU-Usage/
* The PDM library from XIAO doesn't work with the non-mbed core.  Instead, install the PDM library from adafruit: https://github.com/adafruit/Adafruit_nRF52_Arduino (Xiao leverages a lot of adafruit, hence the Bluefruit stuff)


