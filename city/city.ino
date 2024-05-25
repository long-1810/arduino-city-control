#include <TimeLib.h>
#include <Servo.h>
#include "IRremote.h"

const int TRAIN_BARRIER_PIN = 11;
const int WARNING_LIGHT_PIN = 12;
const int LIGHT_DETECT_PIN = 0;
const int STREET_LIGHT_PIN = 8;
const int TRAFFIC_DATA = 10;
const int TRAFFIC_LATCH = 7;
const int TRAFFIC_CLOCK = 9;
const int IR_RECEIVER = 5;

// Train barrier
Servo trainBarrier;
const int initialBarrierPos = 0;
const int openBarrierPos = 90;
bool isBarrierClosed;


// Street lights + House lights
int lightLevel;
const int NIGHT_LEVEL_THRESHOLD = 100;

// Traffic lights
enum TRAFFIC_LIGHT_STATE {
  RED,
  YELLOW,
  GREEN
};
TRAFFIC_LIGHT_STATE upLightState, leftLightState;
const int RED_AND_GREEN_DURATION = 10;
const int YELLOW_DURATION = 2;
int trafficLightInitSession;  // in time_t
int trafficLightCurrentTime;  // Time since beginning of session

// IR
IRrecv irrecv(IR_RECEIVER);
uint32_t last_decodedRawData = 0;
int lastIRSession, currentIRSession;

void setup() {
  pinMode(TRAIN_BARRIER_PIN, OUTPUT);
  pinMode(WARNING_LIGHT_PIN, OUTPUT);
  pinMode(STREET_LIGHT_PIN, OUTPUT);
  pinMode(TRAFFIC_CLOCK, OUTPUT);
  pinMode(TRAFFIC_DATA, OUTPUT);
  pinMode(TRAFFIC_LATCH, OUTPUT);
  trainBarrier.attach(TRAIN_BARRIER_PIN);

  isBarrierClosed = false;
  upLightState = RED;
  leftLightState = GREEN;

  trafficLightInitSession = now();
  lastIRSession = now();

  irrecv.enableIRIn();
}

void loop() {
  checkLights();
  checkTrafficLights();

  if (irrecv.decode()) {
    translateIR();
    irrecv.resume();
  }

  updateTime();
  delay(1000);
}

void translateIR() {
    if (irrecv.decodedIRData.flags) {
      irrecv.decodedIRData.decodedRawData = last_decodedRawData;
    }
    //map the IR code to the remote key
    switch (irrecv.decodedIRData.decodedRawData) {
      case 0xBA45FF00:  // POWER
        isBarrierClosed = !isBarrierClosed;
        checkBarrier();
        break;
    }

    last_decodedRawData = irrecv.decodedIRData.decodedRawData;
}

void updateTime() {
  trafficLightCurrentTime = now() - trafficLightInitSession;
}

void resetTrafficSession() {
  trafficLightCurrentTime = 0;
  trafficLightInitSession = now();
}

void checkTrafficLights() {
  // Traffic light logic start
  if ((upLightState != YELLOW && leftLightState != YELLOW) && (trafficLightCurrentTime >= RED_AND_GREEN_DURATION)) {
    if (upLightState == RED) {
      upLightState = RED;
      leftLightState = YELLOW;
      resetTrafficSession();
    } else if (upLightState == GREEN) {
      upLightState = YELLOW;
      leftLightState = RED;
      resetTrafficSession();
    }
  } else if (upLightState == YELLOW && (trafficLightCurrentTime >= YELLOW_DURATION)) {
    upLightState = RED;
    leftLightState = GREEN;
    resetTrafficSession();
  } else if (leftLightState == YELLOW && (trafficLightCurrentTime >= YELLOW_DURATION)) {
    upLightState = GREEN;
    leftLightState = RED;
    resetTrafficSession();
  }
  // Traffic light logic end

  updateShiftRegister(TRAFFIC_LATCH, TRAFFIC_DATA, TRAFFIC_CLOCK, formatTrafficLightData());
}

byte formatTrafficLightData() {
  byte result;
  switch (upLightState) {
    case RED:
      result += B10000000;
      break;
    case YELLOW:
      result += B01000000;
      break;
    case GREEN:
      result += B00100000;
      break;
  }
  switch (leftLightState) {
    case RED:
      result += B00010000;
      break;
    case YELLOW:
      result += B00001000;
      break;
    case GREEN:
      result += B00000100;
      break;
  }
  return result;
}

void checkLights() {
  lightLevel = analogRead(LIGHT_DETECT_PIN);

  if (lightLevel < NIGHT_LEVEL_THRESHOLD) {
    turnOnStreetLights(STREET_LIGHT_PIN);
  } else if (lightLevel >= NIGHT_LEVEL_THRESHOLD) {
    turnOffStreetLights(STREET_LIGHT_PIN);
  }
}

void turnOnStreetLights(int streetLightPin) {
  digitalWrite(streetLightPin, HIGH);
}

void turnOffStreetLights(int streetLightPin) {
  digitalWrite(streetLightPin, LOW);
}


void checkBarrier() {
  if (isBarrierClosed) {
    digitalWrite(WARNING_LIGHT_PIN, HIGH);
    trainBarrier.write(initialBarrierPos);
  } else if (!isBarrierClosed) {
    digitalWrite(WARNING_LIGHT_PIN, LOW);
    trainBarrier.write(openBarrierPos);
  }
}

void updateShiftRegister(int latchPin, int dataPin, int clockPin, byte data) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, data);
  digitalWrite(latchPin, HIGH);
}