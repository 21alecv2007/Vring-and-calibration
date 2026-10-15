const int BUTTON_PIN = 4; // Pin connected to the button

void setup() {
  Serial.begin(9600);
  // Enable the internal pull-up resistor
  pinMode(BUTTON_PIN, INPUT_PULLUP); 
}

void loop() {
  int buttonState = digitalRead(BUTTON_PIN);

  // Logic is inverted with INPUT_PULLUP:
  // LOW  = Button is pressed (connected to GND)
  // HIGH = Button is released (pulled up to 5V/3.3V)
  if (buttonState == LOW) {
    Serial.println("Button Pressed!");
  } else {
    Serial.println("Button Released.");
  }
  

}
