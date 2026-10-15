#include <DynamixelShield.h>

// Read-only inventory for the detected Cyton arm. This sketch sends PING and
// READ instructions only. It performs no writes and cannot enable torque.

using namespace ControlTableItem;

constexpr uint8_t DXL_DIRECTION_PIN = 2;
constexpr uint32_t DXL_BAUD = 1000000;
constexpr float DXL_PROTOCOL = 1.0;
const uint8_t IDS[] = {8, 9, 10, 11, 12, 13, 14, 15};
constexpr uint8_t SERVO_COUNT = sizeof(IDS) / sizeof(IDS[0]);

struct ServoInfo {
  bool found;
  uint16_t model;
  int32_t firmware;
  int32_t cwLimit;
  int32_t ccwLimit;
  int32_t maxTorque;
  int32_t torqueEnabled;
  int32_t goalPosition;
  int32_t movingSpeed;
  int32_t torqueLimit;
  int32_t presentPosition;
  int32_t presentVoltage;
  int32_t presentTemperature;
  int32_t moving;
};

ServoInfo info[SERVO_COUNT];
DynamixelShield dxl;

void readServo(uint8_t index) {
  const uint8_t id = IDS[index];
  ServoInfo &s = info[index];
  s.found = dxl.ping(id);
  if (!s.found) return;

  s.model = dxl.getModelNumber(id);
  s.firmware = dxl.readControlTableItem(FIRMWARE_VERSION, id);
  s.cwLimit = dxl.readControlTableItem(CW_ANGLE_LIMIT, id);
  s.ccwLimit = dxl.readControlTableItem(CCW_ANGLE_LIMIT, id);
  s.maxTorque = dxl.readControlTableItem(MAX_TORQUE, id);
  s.torqueEnabled = dxl.readControlTableItem(TORQUE_ENABLE, id);
  s.goalPosition = dxl.readControlTableItem(GOAL_POSITION, id);
  s.movingSpeed = dxl.readControlTableItem(MOVING_SPEED, id);
  s.torqueLimit = dxl.readControlTableItem(TORQUE_LIMIT, id);
  s.presentPosition = dxl.readControlTableItem(PRESENT_POSITION, id);
  s.presentVoltage = dxl.readControlTableItem(PRESENT_VOLTAGE, id);
  s.presentTemperature = dxl.readControlTableItem(PRESENT_TEMPERATURE, id);
  s.moving = dxl.readControlTableItem(MOVING, id);
}

void printReport() {
  Serial.println(F("=== CYTON READ-ONLY INVENTORY ==="));
  Serial.println(F("No registers were written and torque was not enabled."));
  Serial.println(F("id,found,model,fw,cw_limit,ccw_limit,max_torque,torque_on,goal_pos,moving_speed,torque_limit,present_pos,voltage_x10,temp_c,moving"));
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    const ServoInfo &s = info[i];
    Serial.print(IDS[i]); Serial.print(',');
    Serial.print(s.found); Serial.print(',');
    Serial.print(s.model); Serial.print(',');
    Serial.print(s.firmware); Serial.print(',');
    Serial.print(s.cwLimit); Serial.print(',');
    Serial.print(s.ccwLimit); Serial.print(',');
    Serial.print(s.maxTorque); Serial.print(',');
    Serial.print(s.torqueEnabled); Serial.print(',');
    Serial.print(s.goalPosition); Serial.print(',');
    Serial.print(s.movingSpeed); Serial.print(',');
    Serial.print(s.torqueLimit); Serial.print(',');
    Serial.print(s.presentPosition); Serial.print(',');
    Serial.print(s.presentVoltage); Serial.print(',');
    Serial.print(s.presentTemperature); Serial.print(',');
    Serial.println(s.moving);
  }
  Serial.println(F("=== END INVENTORY ==="));
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    readServo(i);
  }

  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(DXL_DIRECTION_PIN, OUTPUT);
  digitalWrite(DXL_DIRECTION_PIN, LOW);
  Serial.end();
  delay(100);
  Serial.begin(115200);
}

void loop() {
  printReport();
  delay(2000);
}
