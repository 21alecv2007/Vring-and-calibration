const int CHARGE_PIN = 4;   // Digital output D4 to drive the RC circuit
const int SENSE_PIN = A7;   // Analog input A7 to measure capacitor voltage

// Circuit component parameters
const double RESISTANCE = 100000; //test resisitor = 23650

// Voltage thresholds for 1 Time Constant (Tau ≈ 63.2% for charging, 36.8% for discharging)
// 5V Arduino: ADC range is 0 - 1023
const int TAU_CHARGE_ADC = 647;    // 1023 * (1 - e^-1) = 646.5
const int TAU_DISCHARGE_ADC = 376; // 1023 * (e^-1) = 376.4
const int FULL_CHARGE_ADC = 1022;  // Threshold for full 5V (~99.9% of 1023)

void setup() {
  Serial.begin(9600);
  pinMode(CHARGE_PIN, OUTPUT);
  digitalWrite(CHARGE_PIN, LOW); // Start with capacitor fully discharged
}

void loop() {
  unsigned long startTime, totalTime;
  double capacitance;

  // Ensure capacitor is fully discharged before starting
  digitalWrite(CHARGE_PIN, LOW);
  while (analogRead(SENSE_PIN) > 5) {
    delay(10);
  }

  // --- CHARGING TEST ---
  Serial.println("Starting Charge Cycle...");
  
  // Set pin HIGH and record start time immediately
  digitalWrite(CHARGE_PIN, HIGH);
  startTime = micros();

  // Wait until voltage reaches 1 time constant (63.2%)
  while (analogRead(SENSE_PIN) < TAU_CHARGE_ADC) {
    // Polling ADC
  }

  totalTime = micros() - startTime; // Elapsed time in microseconds

  // Calculate C = Time / Resistance
  // Convert microseconds to seconds (divide by 1,000,000)
  capacitance = ((double)totalTime / 1000000.0) / RESISTANCE;

  Serial.print("Charge Time: ");
  Serial.print(totalTime);
  Serial.print(" us | Estimated Capacitance: ");
  Serial.print(capacitance * 1e6, 3); // Convert to microfarads (uF)
  Serial.println(" uF");
  delay(100);

}