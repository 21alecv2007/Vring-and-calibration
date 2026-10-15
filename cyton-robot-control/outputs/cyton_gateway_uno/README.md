# Cyton Uno gateway (V2)

The Uno keeps its hardware UART dedicated to the DYNAMIXEL Shield at 1 Mbps,
Protocol 1.0. A separate 57,600-baud `SoftwareSerial` link on D7/D8 connects to
the M5StickC Plus.

The sketch disables torque on IDs 8–15 at every boot. Movement requires an
explicit `ENABLE` followed by `MOVE` or `GROUP`. Position commands are rejected
outside each servo's configured CW/CCW limits, and speed is capped at 100 raw.

V2 changes:

- `STOP` is checked for between servos inside `SCAN`, `STATUS`, `ENABLE ALL`
  and `GROUP`; it aborts the command, turns torque off, and replies
  `OK STOP TORQUE_OFF_ALL ABORTED`. Best-effort: SoftwareSerial reception is
  disturbed by DYNAMIXEL traffic, so a STOP that arrives mid-transaction is
  usually garbled; the M5 bridge V2 retries it every 120 ms, so the measured
  worst case is 250 ms (one full STATUS plus a retry).
- Lines containing non-printable bytes (typically a command garbled by a
  reply that was still printing) are dropped with `ERR garbled_line_ignored`
  rather than parsed.
- `HELP` ends with `OK HELP`; `PING` answers `PONG CYTON_GATEWAY_V2`.

Flashing: set the shield UART switch to `UPLOAD`, run `pio run -t upload`
(add `--upload-port` if several boards are attached), then return the switch
to `DYNAMIXEL` and press RESET once.
