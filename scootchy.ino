#include "pitches.h"
#include <Servo.h>

Servo rpmServo;

int melody[] = { NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6 };

volatile unsigned long lastEnginePulse;
volatile float revolutionsPerMinute;
volatile unsigned long lastBlinkerStateChange;
int blinkerState;

//in Arduino mega, pins 2,3,18,19,20,21 are used for interupts
int enginePulseInput = 3;
int rightBlinkerInput = 4;
int leftBlinkerInput = 5;
int keyInput = 6;

int runOutput = 7;
int speedometerOutput = 9;
int rightBlinkerOutput = 28;
int leftBlinkerOutput = 30;
int buzzerOutput = 12;

int runningLightRedOutput = 22;
int runningLightGreenOutput = 24;
int runningLightBlueOutput = 26;

// todo - speedometer - servo

unsigned long maxServoRotation = 180;

void setup() {
  Serial.begin(9600);
  Serial.println("Stating up!");

  lastEnginePulse = millis();
  lastBlinkerStateChange = millis();
  blinkerState = LOW;

  pinMode(rightBlinkerInput, INPUT_PULLUP);
  pinMode(leftBlinkerInput, INPUT_PULLUP);
  pinMode(keyInput, INPUT_PULLUP);
  pinMode(enginePulseInput, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(enginePulseInput), calculateRevolutionsPerMinute, FALLING);

  pinMode(runOutput, OUTPUT);
  pinMode(rightBlinkerOutput, OUTPUT);
  pinMode(leftBlinkerOutput, OUTPUT);
  pinMode(runningLightRedOutput, OUTPUT);
  pinMode(runningLightGreenOutput, OUTPUT);
  pinMode(runningLightBlueOutput, OUTPUT);

  rpmServo.attach(speedometerOutput);
  rpmServo.write(maxServoRotation);

  playStartUpTones();
  initServos();
}

void loop() {
  updateBlikerState();
  updateRunningLight();
  updateSpeedometer();
  updateTachometer();
}

void initServos() {
  int duration = 500;
  rpmServo.write(0);
  delay(duration);
  rpmServo.write(maxServoRotation);
}

void updateBlikerState() {
  unsigned long now = millis();
  unsigned long duration = 300;

  int leftBlinker = !digitalRead(leftBlinkerInput);
  int rightBlinker = !digitalRead(rightBlinkerInput);

  if ((now - lastBlinkerStateChange) > duration) {
    blinkerState = !blinkerState;
    lastBlinkerStateChange = now;

    if (leftBlinker == HIGH) {
      if (blinkerState == HIGH) {
        turnBlinkerOn(leftBlinkerOutput);
      } else {
        turnBlinkerOff(leftBlinkerOutput);
      }
    }

    if (rightBlinker == HIGH) {
      if (blinkerState == HIGH) {
        turnBlinkerOn(rightBlinkerOutput);
      } else {
        turnBlinkerOff(rightBlinkerOutput);
      }
    }
  }

  if (leftBlinker == LOW && rightBlinker == LOW) {
    turnBlinkersOff();
  }
}

void turnBlinkerOn(int blinkerOutput) {
  int toneDuration = 25;
  digitalWrite(blinkerOutput, HIGH);
  tone(buzzerOutput, melody[0], toneDuration);
}

void turnBlinkerOff(int blinkerOutput) {
  int toneDuration = 25;
  digitalWrite(blinkerOutput, LOW);
  tone(buzzerOutput, melody[0], toneDuration);
}

void turnBlinkersOff() {
  digitalWrite(leftBlinkerOutput, LOW);
  digitalWrite(rightBlinkerOutput, LOW);
}

void updateRunOutput() {
  int key = !digitalRead(keyInput);
  digitalWrite(runOutput, key);
}

void updateRunningLight() {
  int key = !digitalRead(keyInput);
  if (key) {
    digitalWrite(runningLightRedOutput, LOW);
    digitalWrite(runningLightGreenOutput, HIGH);
    digitalWrite(runningLightBlueOutput, LOW);
  } else {
    digitalWrite(runningLightRedOutput, HIGH);
    digitalWrite(runningLightGreenOutput, LOW);
    digitalWrite(runningLightBlueOutput, LOW);
  }
}

void updateSpeedometer() {
  // todo
}

void updateTachometer() {
  unsigned long now = millis();
  unsigned long duration = 500;
  unsigned long maxRevolutionsPerMinute = 10000;
  unsigned long maxRotation = 180;
  if ((now - lastEnginePulse) > duration) {
    rpmServo.write(maxServoRotation);
  } else {
    float rotation = maxServoRotation - ((revolutionsPerMinute / maxRevolutionsPerMinute) * maxRotation);
    rpmServo.write(rotation);
  }
}

void calculateRevolutionsPerMinute() {
  unsigned long now = millis();
  unsigned long millisecondsInMinute = 60000;
  unsigned long duration = now - lastEnginePulse;
  if (duration > 0) {
    revolutionsPerMinute = millisecondsInMinute / duration;
    lastEnginePulse = now;
  }
}

void playStartUpTones() {
  int duration = 150;
  for (int thisNote = 0; thisNote < 7; thisNote++) {
    tone(buzzerOutput, melody[thisNote], duration);
    delay(duration);
  }
  tone(buzzerOutput, melody[7], (duration * 2));
}
