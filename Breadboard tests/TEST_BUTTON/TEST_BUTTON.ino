const int inputPin = 8;    // Digital input pin D3
int currentState = LOW;    // Current reading from the input pin
int lastState = LOW;       // Previous reading from the input pin

void setup() {
  pinMode(inputPin, INPUT);
  Serial.begin(9600);      // Initialize Serial communication
}

void loop() {
  currentState = digitalRead(inputPin);

  // Check for a transition from LOW to HIGH (rising edge)
  if (currentState == HIGH && lastState == LOW) {
    Serial.println("Hello");
  }

  // Update lastState to remember the reading for the next iteration
  lastState = currentState;
  delay(10);
}