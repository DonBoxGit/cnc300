/*  CNC WPC Fix Module
       ver. 1.0.1
*/
#define DEBUG
#include <EncButton.h>
#include "fastArduino.h"

#define RESET_PIN 13
#define CLOCKWISE_BTN_SIGNAL_OUTPUT 6
#define STOP_BTN_SIGNAL_OUTPUT  7
#define COUNTERCLOCKWISE_BTN_SIGNAL_OUTPUT 8

#define DIRECTION_INPUT_PIN A0
#define FOOT_CONTROL_INPUT_PIN A1

#define CLICK_DURATION  100

/* Foot button object create */
EncButton<EB_TICK, FOOT_CONTROL_INPUT_PIN> foot_btn(INPUT_PULLUP);

void generateClickTopin(const int);
bool foot_btn_flag = false;

void setup() {
#ifdef DEBUG
  Serial.begin(9600);
#endif

  /* Output pins initialization */
  far::pinMode(RESET_PIN, OUTPUT);
  far::pinMode(CLOCKWISE_BTN_SIGNAL_OUTPUT, OUTPUT);
  far::pinMode(STOP_BTN_SIGNAL_OUTPUT, OUTPUT);
  far::pinMode(COUNTERCLOCKWISE_BTN_SIGNAL_OUTPUT, OUTPUT);

  /* Input pins initialization */
  far::pinMode(DIRECTION_INPUT_PIN, INPUT);
#ifdef DEBUG
  Serial.println("Ports initialization completed.");
#endif

  /* Making a reset of the CNC controller */
  far::digitalWrite(RESET_PIN, LOW);
  _delay_ms(CLICK_DURATION);
  far::digitalWrite(RESET_PIN, HIGH);
  _delay_ms(CLICK_DURATION);
#ifdef DEBUG
  Serial.println("Reset of the CNC controller done.");
#endif
}

void loop() {
  foot_btn.tick();

  if (foot_btn.press()) {
    foot_btn_flag = !foot_btn_flag;
    if (foot_btn_flag && far::digitalRead(DIRECTION_INPUT_PIN)) {
      generateClickToPin(CLOCKWISE_BTN_SIGNAL_OUTPUT);
#ifdef DEBUG
      Serial.println("Right CNC button pressed");
#endif
    }
    if (foot_btn_flag && !far::digitalRead(DIRECTION_INPUT_PIN)) {
      generateClickToPin(COUNTERCLOCKWISE_BTN_SIGNAL_OUTPUT);
#ifdef DEBUG
      Serial.println("Left CNC button pressed");
#endif
    }
    if (!foot_btn_flag) {
      generateClickToPin(STOP_BTN_SIGNAL_OUTPUT);
#ifdef DEBUG
      Serial.println("Stop CNC button pressed");
#endif
    }
  }
}

void generateClickToPin(const int pin) {
  far::digitalWrite(pin, HIGH);
  _delay_ms(CLICK_DURATION);
  far::digitalWrite(pin, LOW);
}
