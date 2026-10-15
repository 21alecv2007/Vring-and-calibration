#include <Arduino.h>
#include <SoftwareSerial.h>
#include <DynamixelShield.h>

using namespace ControlTableItem;

namespace {
constexpr uint32_t HOST_BAUD = 57600;
constexpr uint32_t DXL_BAUD = 1000000;
constexpr float DXL_PROTOCOL = 1.0;
constexpr uint16_t MAX_SPEED = 100;
constexpr uint16_t DEFAULT_HOLD_SPEED = 25;
constexpr uint8_t DXL_RETRIES = 3;
constexpr uint8_t IDS[] = {8, 9, 10, 11, 12, 13, 14, 15};
constexpr uint8_t SERVO_COUNT = sizeof(IDS) / sizeof(IDS[0]);
constexpr size_t LINE_SIZE = 128;

SoftwareSerial host(7, 8);  // RX, TX
DynamixelShield dxl;
char line[LINE_SIZE];
size_t lineLength = 0;
bool lineGarbled = false;   // a byte outside printable ASCII was seen on this line
bool abortCurrent = false;  // a STOP arrived while a multi-servo command was running

bool torqueOffReliable(uint8_t id);
void stopAll();

// SoftwareSerial cannot receive while we transmit, but between DYNAMIXEL
// transactions it can. Long commands call this between servos so that a
// STOP from the bridge is honoured within a few milliseconds instead of
// after the whole report. Any other pending text is left in the buffer for
// loop() to parse afterwards.
bool hostRequestedStop() {
  static char peek[8];
  static size_t peekLength = 0;
  while (host.available()) {
    const char c = static_cast<char>(host.read());
    if (c == '\r') continue;
    if (c == '\n') {
      peek[peekLength] = '\0';
      const bool stop = !strcasecmp(peek, "STOP");
      peekLength = 0;
      if (stop) {
        abortCurrent = true;
        return true;
      }
      continue;  // some other complete line arrived mid-command: drop it
    }
    if (peekLength + 1 < sizeof(peek)) peek[peekLength++] = c;
    else peekLength = 0;  // longer than any STOP, cannot be one
  }
  return abortCurrent;
}

void finishAbort() {
  stopAll();
  host.println(F("OK STOP TORQUE_OFF_ALL ABORTED"));
  abortCurrent = false;
}

bool allowedId(long id) {
  for (uint8_t candidate : IDS) {
    if (id == candidate) return true;
  }
  return false;
}

bool pingReliable(uint8_t id) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    if (dxl.ping(id)) return true;
    delay(2);
  }
  return false;
}

bool getModelReliable(uint8_t id, uint16_t &model) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    model = dxl.getModelNumber(id);
    if (dxl.getLastLibErrCode() == DXL_LIB_OK && model != 0xFFFF) return true;
    delay(2);
  }
  return false;
}

bool readReliable(uint8_t item, uint8_t id, int32_t &value) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    value = dxl.readControlTableItem(item, id);
    if (dxl.getLastLibErrCode() == DXL_LIB_OK) return true;
    delay(2);
  }
  return false;
}

bool writeReliable(uint8_t item, uint8_t id, int32_t value) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    if (dxl.writeControlTableItem(item, id, value)) return true;
    delay(2);
  }
  return false;
}

bool setGoalReliable(uint8_t id, int32_t position) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    if (dxl.setGoalPosition(id, position, UNIT_RAW)) return true;
    delay(2);
  }
  return false;
}

bool torqueOffReliable(uint8_t id) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    if (dxl.torqueOff(id)) return true;
    delay(2);
  }
  return false;
}

bool torqueOnReliable(uint8_t id) {
  for (uint8_t attempt = 0; attempt < DXL_RETRIES; ++attempt) {
    if (dxl.torqueOn(id)) return true;
    delay(2);
  }
  return false;
}

bool parseLong(const char *token, long &value) {
  if (!token || !*token) return false;
  char *end = nullptr;
  value = strtol(token, &end, 10);
  return end && *end == '\0';
}

void error(const __FlashStringHelper *message) {
  host.print(F("ERR "));
  host.println(message);
}

void reportStatus(uint8_t id) {
  if (!pingReliable(id)) {
    host.print(F("ID "));
    host.print(id);
    host.println(F(" OFFLINE"));
    return;
  }

  host.print(F("ID "));
  host.print(id);
  host.print(F(" MODEL "));
  uint16_t model;
  if (getModelReliable(id, model)) host.print(model); else host.print(F("READ_ERROR"));
  host.print(F(" POS "));
  int32_t value;
  readReliable(PRESENT_POSITION, id, value); host.print(value);
  host.print(F(" GOAL "));
  readReliable(GOAL_POSITION, id, value); host.print(value);
  host.print(F(" TORQUE "));
  readReliable(TORQUE_ENABLE, id, value); host.print(value);
  host.print(F(" MOVING "));
  readReliable(MOVING, id, value); host.print(value);
  host.print(F(" VOLT_X10 "));
  readReliable(PRESENT_VOLTAGE, id, value); host.print(value);
  host.print(F(" TEMP_C "));
  readReliable(PRESENT_TEMPERATURE, id, value); host.println(value);
}

bool withinServoLimits(uint8_t id, long position) {
  int32_t cw, ccw;
  if (!readReliable(CW_ANGLE_LIMIT, id, cw) ||
      !readReliable(CCW_ANGLE_LIMIT, id, ccw) || ccw <= cw) return false;
  return position >= cw && position <= ccw;
}

bool torqueEnabled(uint8_t id) {
  int32_t value;
  return readReliable(TORQUE_ENABLE, id, value) && value == 1;
}

bool enableHoldingPresentPosition(uint8_t id) {
  if (!pingReliable(id)) return false;
  int32_t present;
  if (!readReliable(PRESENT_POSITION, id, present)) return false;
  if (!torqueOffReliable(id)) return false;
  if (!writeReliable(MOVING_SPEED, id, DEFAULT_HOLD_SPEED)) return false;
  if (!setGoalReliable(id, present)) return false;
  return torqueOnReliable(id);
}

void stopAll() {
  for (uint8_t id : IDS) torqueOffReliable(id);
}

void printHelp() {
  host.println(F("COMMANDS"));
  host.println(F("  PING"));
  host.println(F("  SCAN"));
  host.println(F("  STATUS [id]"));
  host.println(F("  LIMITS id"));
  host.println(F("  ENABLE id|ALL   (holds present position first)"));
  host.println(F("  DISABLE id|ALL"));
  host.println(F("  MOVE id raw_position speed_1_to_100"));
  host.println(F("  GROUP speed p8 p9 p10 p11 p12 p13 p14 p15"));
  host.println(F("  STOP"));
  host.println(F("OK HELP"));
}

void handleLine(char *command) {
  char *save = nullptr;
  char *verb = strtok_r(command, " \t", &save);
  if (!verb) return;

  if (!strcasecmp(verb, "PING")) {
    host.println(F("PONG CYTON_GATEWAY_V2"));
    return;
  }

  if (!strcasecmp(verb, "HELP")) {
    printHelp();
    return;
  }

  if (!strcasecmp(verb, "SCAN")) {
    for (uint8_t id : IDS) {
      if (hostRequestedStop()) { finishAbort(); return; }
      host.print(F("ID "));
      host.print(id);
      if (pingReliable(id)) {
        uint16_t model;
        host.print(F(" MODEL "));
        if (getModelReliable(id, model)) host.println(model); else host.println(F("READ_ERROR"));
      } else {
        host.println(F(" OFFLINE"));
      }
    }
    host.println(F("OK SCAN"));
    return;
  }

  if (!strcasecmp(verb, "STATUS")) {
    char *idToken = strtok_r(nullptr, " \t", &save);
    if (!idToken) {
      for (uint8_t id : IDS) {
        if (hostRequestedStop()) { finishAbort(); return; }
        reportStatus(id);
      }
      host.println(F("OK STATUS"));
      return;
    }
    long id;
    if (!parseLong(idToken, id) || !allowedId(id)) {
      error(F("invalid_id"));
      return;
    }
    reportStatus(static_cast<uint8_t>(id));
    host.println(F("OK STATUS"));
    return;
  }

  if (!strcasecmp(verb, "LIMITS")) {
    long id;
    if (!parseLong(strtok_r(nullptr, " \t", &save), id) || !allowedId(id)) {
      error(F("invalid_id"));
      return;
    }
    if (!pingReliable(id)) {
      error(F("offline"));
      return;
    }
    host.print(F("ID ")); host.print(id);
    int32_t cw, ccw;
    if (!readReliable(CW_ANGLE_LIMIT, id, cw) || !readReliable(CCW_ANGLE_LIMIT, id, ccw)) {
      error(F("read_failed"));
      return;
    }
    host.print(F(" CW ")); host.print(cw);
    host.print(F(" CCW ")); host.println(ccw);
    host.println(F("OK LIMITS"));
    return;
  }

  if (!strcasecmp(verb, "STOP")) {
    stopAll();
    host.println(F("OK STOP TORQUE_OFF_ALL"));
    return;
  }

  if (!strcasecmp(verb, "ENABLE") || !strcasecmp(verb, "DISABLE")) {
    const bool enable = !strcasecmp(verb, "ENABLE");
    char *target = strtok_r(nullptr, " \t", &save);
    if (!target) {
      error(F("missing_target"));
      return;
    }
    if (!strcasecmp(target, "ALL")) {
      bool ok = true;
      for (uint8_t id : IDS) {
        if (enable && hostRequestedStop()) { finishAbort(); return; }
        ok &= enable ? enableHoldingPresentPosition(id) : torqueOffReliable(id);
      }
      host.println(ok ? F("OK") : F("ERR one_or_more_failed"));
      return;
    }
    long id;
    if (!parseLong(target, id) || !allowedId(id)) {
      error(F("invalid_id"));
      return;
    }
    const bool ok = enable ? enableHoldingPresentPosition(id) : torqueOffReliable(id);
    host.println(ok ? F("OK") : F("ERR operation_failed"));
    return;
  }

  if (!strcasecmp(verb, "MOVE")) {
    long id, position, speed;
    if (!parseLong(strtok_r(nullptr, " \t", &save), id) || !allowedId(id) ||
        !parseLong(strtok_r(nullptr, " \t", &save), position) ||
        !parseLong(strtok_r(nullptr, " \t", &save), speed)) {
      error(F("usage_MOVE_id_position_speed"));
      return;
    }
    if (speed < 1 || speed > MAX_SPEED) {
      error(F("speed_must_be_1_to_100"));
      return;
    }
    if (!pingReliable(id)) {
      error(F("offline"));
      return;
    }
    if (!torqueEnabled(id)) {
      error(F("torque_off_use_ENABLE_first"));
      return;
    }
    if (!withinServoLimits(id, position)) {
      error(F("position_outside_servo_limits"));
      return;
    }
    const bool speedOk = writeReliable(MOVING_SPEED, id, speed);
    const bool positionOk = setGoalReliable(id, position);
    host.println(speedOk && positionOk ? F("OK MOVE") : F("ERR move_failed"));
    return;
  }

  if (!strcasecmp(verb, "GROUP")) {
    long speed;
    long positions[SERVO_COUNT];
    if (!parseLong(strtok_r(nullptr, " \t", &save), speed) || speed < 1 || speed > MAX_SPEED) {
      error(F("speed_must_be_1_to_100"));
      return;
    }
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      if (!parseLong(strtok_r(nullptr, " \t", &save), positions[i])) {
        error(F("GROUP_requires_8_positions"));
        return;
      }
      if (hostRequestedStop()) { finishAbort(); return; }
      if (!pingReliable(IDS[i]) || !torqueEnabled(IDS[i]) || !withinServoLimits(IDS[i], positions[i])) {
        error(F("group_precheck_failed_no_motion_sent"));
        return;
      }
    }
    bool ok = true;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
      if (hostRequestedStop()) { finishAbort(); return; }
      ok &= writeReliable(MOVING_SPEED, IDS[i], speed);
      ok &= setGoalReliable(IDS[i], positions[i]);
    }
    host.println(ok ? F("OK GROUP") : F("ERR group_write_failed"));
    return;
  }

  error(F("unknown_command_use_HELP"));
}
}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  host.begin(HOST_BAUD);
  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  // Safety invariant: the gateway never starts with an energized arm.
  stopAll();
  host.println(F("READY CYTON_GATEWAY_V2 TORQUE_OFF"));
  digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
  while (host.available()) {
    const char c = static_cast<char>(host.read());
    if (c == '\r') continue;
    if (c == '\n') {
      line[lineLength] = '\0';
      if (lineGarbled) {
        // Bytes mangled while we were transmitting: not a real command.
        error(F("garbled_line_ignored"));
      } else if (lineLength) {
        handleLine(line);
      }
      lineLength = 0;
      lineGarbled = false;
    } else if (c < 0x20 || c > 0x7e) {
      lineGarbled = true;
    } else if (lineLength + 1 < LINE_SIZE) {
      line[lineLength++] = c;
    } else {
      lineLength = 0;
      lineGarbled = false;
      error(F("line_too_long"));
    }
  }
}
