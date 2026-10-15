#include <DynamixelShield.h>

// Conservative one-time movement test for the four detected Protocol 1.0
// actuators. No EEPROM settings are changed. Each actuator is tested alone,
// moves five degrees, returns, and has torque disabled again.

using namespace ControlTableItem;

constexpr uint8_t DXL_DIRECTION_PIN = 2;
constexpr uint32_t DXL_BAUD = 1000000;
constexpr float DXL_PROTOCOL = 1.0;
constexpr uint16_t TEST_TORQUE_LIMIT = 100;  // About 10% of 0..1023.
constexpr uint16_t TEST_MOVING_SPEED = 15;   // Slow; never use 0 (means max).
constexpr float TEST_DELTA_DEG = 5.0;

const uint8_t IDS[] = {2, 3, 20, 21};
const float MAX_DEGREES[] = {360.0, 360.0, 300.0, 300.0};
constexpr uint8_t SERVO_COUNT = sizeof(IDS) / sizeof(IDS[0]);

struct TestResult {
  bool found;
  bool configured;
  bool movedOut;
  bool movedBack;
  float startDeg;
  float targetDeg;
  float finalDeg;
};

TestResult results[SERVO_COUNT];
DynamixelShield dxl;

void runTest(uint8_t index) {
  const uint8_t id = IDS[index];
  TestResult &result = results[index];

  result.found = dxl.ping(id);
  if (!result.found) {
    return;
  }

  // Begin from torque-off and capture the actual position.
  dxl.torqueOff(id);
  delay(50);
  result.startDeg = dxl.getPresentPosition(id, UNIT_DEGREE);
  if (result.startDeg < 0.0 || result.startDeg > MAX_DEGREES[index]) {
    return;
  }

  // Move away from the nearer software endpoint.
  if (result.startDeg <= MAX_DEGREES[index] / 2.0) {
    result.targetDeg = result.startDeg + TEST_DELTA_DEG;
  } else {
    result.targetDeg = result.startDeg - TEST_DELTA_DEG;
  }

  bool torqueLimitOk =
      dxl.writeControlTableItem(TORQUE_LIMIT, id, TEST_TORQUE_LIMIT);
  bool speedOk =
      dxl.writeControlTableItem(MOVING_SPEED, id, TEST_MOVING_SPEED);
  bool holdCurrentOk =
      dxl.setGoalPosition(id, result.startDeg, UNIT_DEGREE);
  result.configured = torqueLimitOk && speedOk && holdCurrentOk;

  if (!result.configured || !dxl.torqueOn(id)) {
    dxl.torqueOff(id);
    return;
  }

  delay(500);
  result.movedOut = dxl.setGoalPosition(id, result.targetDeg, UNIT_DEGREE);
  delay(3000);
  result.movedBack = dxl.setGoalPosition(id, result.startDeg, UNIT_DEGREE);
  delay(3000);
  result.finalDeg = dxl.getPresentPosition(id, UNIT_DEGREE);

  // Always leave this actuator relaxed after its test.
  dxl.torqueOff(id);
  delay(250);
}

void printReport() {
  Serial.println(F("=== DYNAMIXEL SLOW TEST ==="));
  Serial.println(F("All test torque is now OFF."));
  Serial.println(F("id,found,configured,out_cmd,back_cmd,start_deg,target_deg,final_deg"));
  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    Serial.print(IDS[i]);
    Serial.print(',');
    Serial.print(results[i].found);
    Serial.print(',');
    Serial.print(results[i].configured);
    Serial.print(',');
    Serial.print(results[i].movedOut);
    Serial.print(',');
    Serial.print(results[i].movedBack);
    Serial.print(',');
    Serial.print(results[i].startDeg, 2);
    Serial.print(',');
    Serial.print(results[i].targetDeg, 2);
    Serial.print(',');
    Serial.println(results[i].finalDeg, 2);
  }
  Serial.println(F("=== END TEST ==="));
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  dxl.begin(DXL_BAUD);
  dxl.setPortProtocolVersion(DXL_PROTOCOL);

  for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    runTest(i);
  }

  // Solid L LED means all four attempts are over and all torque is off.
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
