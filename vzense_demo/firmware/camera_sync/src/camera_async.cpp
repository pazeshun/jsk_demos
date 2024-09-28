#include <Arduino.h>

enum Mode {
  SYNC,
  ASYNC,
};

Mode currentMode = ASYNC;
bool currentTurn = false;
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
}


void loop() {
  // digitalWrite(LED_BUILTIN, HIGH);
  if (currentMode == SYNC) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
  } else {
    digitalWrite(2 + int(currentTurn), HIGH);
    currentTurn = !currentTurn;
  }
  delay(1);
  // digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  delay(34);
}
