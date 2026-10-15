#include <DynamixelShield.h>

// Read-only discovery sketch for an Arduino Uno + ROBOTIS DYNAMIXEL Shield.
// It sends only PING instructions. It never enables torque or requests motion.

constexpr uint8_t DXL_DIRECTION_PIN = 2;
constexpr uint8_t MAX_RESULTS = 16;

const uint32_t BAUD_RATES[] = {57600, 115200, 1000000, 2000000, 3000000};
constexpr uint8_t BAUD_COUNT = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

struct Result {
  uint8_t protocol;
  uint8_t id;
  uint16_t model;
  uint32_t baud;
};

Result results[MAX_RESULTS];
uint8_t resultCount = 0;
bool resultOverflow = false;

DynamixelShield dxl;

bool alreadyRecorded(uint8_t protocol, uint8_t id, uint32_t baud) {
  for (uint8_t i = 0; i < resultCount; ++i) {
    if (results[i].protocol == protocol &&
        results[i].id == id &&
        results[i].baud == baud) {
      return true;
    }
  }
  return false;
}

void recordResult(uint8_t protocol, uint8_t id, uint32_t baud,
                  uint16_t model) {
  if (alreadyRecorded(protocol, id, baud)) {
    return;
  }

  if (resultCount >= MAX_RESULTS) {
    resultOverflow = true;
    return;
  }

  results[resultCount++] = {protocol, id, model, baud};
}

void scanBus() {
  for (uint8_t protocol = 1; protocol <= 2; ++protocol) {
    dxl.setPortProtocolVersion((float)protocol);

    for (uint8_t baudIndex = 0; baudIndex < BAUD_COUNT; ++baudIndex) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      const uint32_t baud = BAUD_RATES[baudIndex];
      dxl.begin(baud);

      // IDs 0..253 are device IDs. 254 is the broadcast ID.
      for (uint16_t id = 0; id < DXL_BROADCAST_ID; ++id) {
        if (dxl.ping((uint8_t)id)) {
          recordResult(protocol, (uint8_t)id, baud,
                       dxl.getModelNumber((uint8_t)id));
        }
      }
    }
  }
}

void printResults() {
  Serial.println(F("=== DYNAMIXEL SAFE SCAN ==="));
  Serial.println(F("This sketch sent PING only; it did not command motion."));

  if (resultCount == 0) {
    Serial.println(F("NO SERVOS FOUND"));
    Serial.println(F("Check servo power, shield power switch, cabling, and SW2."));
  } else {
    Serial.print(F("FOUND "));
    Serial.print(resultCount);
    Serial.println(F(" RESPONSE(S)"));
    Serial.println(F("protocol,id,baud,model_number"));

    for (uint8_t i = 0; i < resultCount; ++i) {
      Serial.print(results[i].protocol);
      Serial.print(',');
      Serial.print(results[i].id);
      Serial.print(',');
      Serial.print(results[i].baud);
      Serial.print(',');
      Serial.println(results[i].model);
    }
  }

  if (resultOverflow) {
    Serial.println(F("WARNING: More than 16 responses; output was truncated."));
  }
  Serial.println(F("=== END SCAN ==="));
  Serial.println();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // SW2 must be in DYNAMIXEL position while this runs.
  scanBus();

  // Solid LED means scanning is complete. Move SW2 to UPLOAD now.
  digitalWrite(LED_BUILTIN, HIGH);

  // Leave the DYNAMIXEL transceiver in receive mode so subsequent report text
  // is not placed on the servo bus.
  pinMode(DXL_DIRECTION_PIN, OUTPUT);
  digitalWrite(DXL_DIRECTION_PIN, LOW);

  Serial.end();
  delay(100);
  Serial.begin(115200);
}

void loop() {
  // Repeat so the report appears even if SW2 is moved after setup completes.
  printResults();
  delay(2000);
}
