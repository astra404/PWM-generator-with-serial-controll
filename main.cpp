#include <Arduino.h>
#define PWM_PIN 9
#define LED_PIN 13

float pwm_freq = 1000;     // Default 1kHz
float duty_percent = 50;   // Default 50%
bool pwm_enabled = false;
unsigned long last_report = 0;



void setPwmFreqAndDuty(float freq, float duty, bool enable) {
  if (!enable) {
    // Disable PWM: set pin LOW, turn off timer
    TCCR1A = 0;
    TCCR1B = 0;
    digitalWrite(PWM_PIN, LOW);
    return;
  }
    

  // Timer1 Fast PWM, ICR1 as TOP, OC1A (Pin 9) output
  uint16_t prescalers[] = {1, 8, 64, 256, 1024};
  uint16_t best_prescaler = 1, top = 0;
  bool found = false;
  for (int i = 0; i < 5; i++) {
    float calc_top = (16000000.0 / (prescalers[i] * freq)) - 1;
    if (calc_top <= 65535) {
      best_prescaler = prescalers[i];
      top = (uint16_t)calc_top;
      found = true;
      break;
    }
  }
  if (!found) {
    best_prescaler = 1024;
    top = 65535;
  }
  // Configure Timer1
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A |= (1 << COM1A1); // Non-inverting, OC1A = Pin 9
  TCCR1A |= (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << WGM12); // Mode 14: Fast PWM, ICR1 as TOP
  switch (best_prescaler) {
    case 1:    TCCR1B |= (1 << CS10); break;
    case 8:    TCCR1B |= (1 << CS11); break;
    case 64:   TCCR1B |= (1 << CS11) | (1 << CS10); break;
    case 256:  TCCR1B |= (1 << CS12); break;
    case 1024: TCCR1B |= (1 << CS12) | (1 << CS10); break;
  }
  ICR1 = top;
  OCR1A = (uint16_t)(top * duty / 100.0);
}


void setup() {
  Serial.begin(9600);
  pinMode(PWM_PIN, OUTPUT);
  setPwmFreqAndDuty(pwm_freq, duty_percent, pwm_enabled);
  Serial.println("Commands: F x,x - set freq (Hz), D x,x - set duty (%), E - enable, X - disable");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() < 1) return;
    char type = toupper(cmd[0]);
    if (type == 'F' || type == 'D') {
      int spaceIdx = cmd.indexOf(' ');
      if (spaceIdx < 0) return;
      String valueStr = cmd.substring(spaceIdx + 1);
      valueStr.replace(',', '.');
      float value = valueStr.toFloat();
      if (type == 'F') {
        if (value < 1) value = 1;
        pwm_freq = (value < 1000) ? (value * 1000) : value;
        Serial.print("Set PWM freq: "); Serial.println(pwm_freq);
        setPwmFreqAndDuty(pwm_freq, duty_percent, pwm_enabled);
      } else if (type == 'D') {
        if (value < 0.1) value = 0.1;
        if (value > 100) value = 100;
        duty_percent = value;
        Serial.print("Set Duty: "); Serial.println(duty_percent);
        setPwmFreqAndDuty(pwm_freq, duty_percent, pwm_enabled);
      }
    } else if (type == 'E') {
      pwm_enabled = true;
      setPwmFreqAndDuty(pwm_freq, duty_percent, pwm_enabled);
      Serial.println("PWM Enabled");
    } else if (type == 'X') {
      pwm_enabled = false;
      setPwmFreqAndDuty(pwm_freq, duty_percent, pwm_enabled);
      Serial.println("PWM Disabled");
    }
  }

  // Report status every 1s
  if (millis() - last_report > 1000) {
    last_report = millis();
    if (pwm_enabled) {
      Serial.print("PWM ON at ");
      Serial.print(pwm_freq / 1000.0, 2);
      Serial.print(" kHz and ");
      Serial.print(duty_percent, 1);
      Serial.println("%");
      digitalWrite(13,HIGH);
    } else {
      Serial.println("PWM OFF");
      digitalWrite(13,LOW);
    }
  }
}
