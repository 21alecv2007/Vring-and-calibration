# M5StickC Plus UART bridge (V2, line-paced)

Bridge between the computer-facing USB serial port at 115,200 baud and the
Uno gateway at 57,600 baud.

- G26: UART TX to Uno/shield software-UART RX (D7)
- G36: UART RX from the 1 kΩ / 2 kΩ divider fed by Uno/shield TX (D8)
- GND: common ground

This target is for the first-generation M5StickC Plus with AXP192.

## Why it paces commands

The Uno receives from the bridge on SoftwareSerial, which cannot receive
while it transmits. A host command that arrives while the Uno is still
printing a reply (a full `STATUS` report takes ~300 ms) is lost or garbled.
V2 therefore:

- queues complete host lines (16 deep) and forwards the next one only after
  the Uno's terminating reply line (`OK…`, `ERR…`, `PONG`, `READY`) or a
  1.5 s timeout;
- sends `STOP` immediately, drops any pending `MOVE`/`GROUP`/`ENABLE` lines,
  and resends it every 120 ms until the Uno answers `OK STOP`;
- sends a bare newline and a `STOP` at boot, i.e. every time a host opens the
  port (the FTDI reset line restarts the ESP32);
- after a STOP episode holds the next command for up to 200 ms so late
  `OK STOP` replies to retried copies are not mistaken for its reply;
- answers `M5` locally with `M5 BRIDGE_V2 queued=… in_flight=… stop_pending=…`;
- passes every byte from the Uno through unchanged, so all existing tools
  and a plain serial terminal keep working.

Bridge-generated messages start with `M5 ERR` (`queue_full_dropped`,
`line_too_long`, `no_reply_timeout`, `stop_unacknowledged`).

Flash with `pio run -t upload` from this directory (no switch to flip; the
M5's own USB port is the one the host uses).
