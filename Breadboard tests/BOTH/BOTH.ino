const int CHARGE_PIN = 4;    // Digital output D4 to drive the RC circuit
const int SENSE_PIN = A7;    // Analog input A7 to measure capacitor voltage
const int inputPin = 8;     // Digital input pin D8 for the separate button

int currentState = LOW;     // Current reading from the input pin
int lastState = LOW;        // Previous reading from the input pin

// State tracking for capacitance threshold to prevent repeating "Hi"
bool isAboveThreshold = false;

// Circuit component parameters
const double RESISTANCE = 100000; // 100k Ohm resistor

// Voltage threshold for 1 Time Constant (Tau ≈ 63.2% for charging)
const int TAU_CHARGE_ADC = 647;    // 1023 * (1 - e^-1) = 646.5

void setup() {
  Serial.begin(9600);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Start with capacitor fully discharged

  pinMode(inputPin, INPUT);     // Configured per your reference code
}

void loop() {
  unsigned long startTime, totalTime;
  double capacitance;

  // --- 1. SEPARATE BUTTON CHECK ---
  currentState = digitalRead(inputPin);
  if (currentState == HIGH && lastState == LOW) {
    Serial.println("Hello");
  }
  lastState = currentState;

  // --- 2. MEASUREMENT METHOD ---
  // Ensure capacitor is fully discharged (with a timeout so it never hangs)
  digitalWrite(CHARGE_PIN, LOW);
  unsigned long dischargeStart = millis();
  while (analogRead(SENSE_PIN) > 5 && (millis() - dischargeStart < 250)) {
    // Polling discharge
  }

  // Set pin HIGH and record start time immediately
  digitalWrite(CHARGE_PIN, HIGH);
  startTime = micros();

  // Wait until voltage reaches 1 time constant (63.2%)
  while (analogRead(SENSE_PIN) < TAU_CHARGE_ADC) {
    // Polling ADC
  }

  totalTime = micros() - startTime; // Elapsed time in microseconds

  // Calculate C = Time / Resistance (in Farads)
  capacitance = ((double)totalTime / 1000000.0) / RESISTANCE;
  double capacitance_nF = capacitance * 1e9; // Convert to nanofarads (nF)

  // --- 3. EDGE-DETECTION THRESHOLD CHECK ---
  if (capacitance_nF > 2.0) {
    // Only print "Hi" if it was NOT previously above the threshold
    if (!isAboveThreshold) {
      Serial.println("Hi");
      isAboveThreshold = true;
    }
  } else {
    // Reset state when capacitance drops back to 2 nF or below
    isAboveThreshold = false;
  }

  // Immediately start discharging for the next cycle
  digitalWrite(CHARGE_PIN, LOW);
  delay(10); // Short reset delay
}