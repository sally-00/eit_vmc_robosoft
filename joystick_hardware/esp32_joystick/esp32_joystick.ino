// KY-023 / PS2 joystick reader for a classic ESP32-WROVER board.
// Change these GPIO numbers if your actual wires differ.
constexpr int PIN_VRX = 32;  // VRx -> ADC1 GPIO32
constexpr int PIN_VRY = 33;  // VRy -> ADC1 GPIO33
constexpr int PIN_SW = 14;   // SW  -> GPIO14; switch closes to GND
constexpr uint32_t SAMPLE_PERIOD_MS = 10;  // nominal 100 samples/s

uint32_t next_sample_ms = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);  // raw range 0..4095
  analogSetPinAttenuation(PIN_VRX, ADC_11db);
  analogSetPinAttenuation(PIN_VRY, ADC_11db);
  pinMode(PIN_SW, INPUT_PULLUP);
  delay(200);
  Serial.println("esp_ms,x_raw,y_raw,pressed");
  next_sample_ms = millis();
}

void loop() {
  const uint32_t now = millis();
  if (static_cast<int32_t>(now - next_sample_ms) < 0) return;
  next_sample_ms += SAMPLE_PERIOD_MS;
  if (static_cast<int32_t>(now - next_sample_ms) >= 0) {
    next_sample_ms = now + SAMPLE_PERIOD_MS;
  }

  const int x = analogRead(PIN_VRX);
  const int y = analogRead(PIN_VRY);
  const int pressed = digitalRead(PIN_SW) == LOW ? 1 : 0;

  Serial.print(now);
  Serial.print(',');
  Serial.print(x);
  Serial.print(',');
  Serial.print(y);
  Serial.print(',');
  Serial.println(pressed);
}
