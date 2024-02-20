#include <Arduino.h>
#include <SPI.h>
#include "esp32-hal-adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Pin definitions
#define BUTTON1     8
#define BUTTON2     3
#define ENCODER_A   47
#define ENCODER_B   20
#define MISO_PIN    12
#define MOSI_PIN    11
#define SCLK_PIN    13
#define NCS_PIN     10

// Sensor register definitions
#define PRODUCT_ID          0x00
#define REVISION_ID         0x01
#define DELTA_X_L           0x03
#define DELTA_X_H           0x04
#define DELTA_Y_L           0x05
#define DELTA_Y_H           0x06

// Scroll wheel encoder variables
uint8_t posCurrent = 0;
uint8_t posPrevious = 0;
uint8_t scroll = 0;

//flag to track buttons' state
bool button1Pressed = false;
bool button2Pressed = false;

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Setup encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
}

void loop() {
  // Read analog values from buttons
  uint16_t button1Voltage = analogRead(BUTTON1);
  uint16_t button2Voltage = analogRead(BUTTON2);

  // Check if Button1 is pressed
  if (button1Voltage > 2000) {
    if (!button1Pressed) {
      Serial.println("Button1 Pressed");
      button1Pressed = true;
    }
  } 
  else {
    if (button1Pressed && button1Voltage < 3000) {
      Serial.println("Button1 Released");
      button1Pressed = false;
    }
  }
  // Check if Button 2 is pressed
  if (button2Voltage > 2000) {
    if (!button2Pressed) {
      Serial.println("Button2 Pressed");
      button2Pressed = true;
    }
  } 
  else {
    if (button2Pressed && button2Voltage < 3000) {
      Serial.println("Button2 Released");
      button2Pressed = false;
    }
  }

  // Read encoder state
  uint8_t Astate = digitalRead(ENCODER_A);
  uint8_t Bstate = digitalRead(ENCODER_B);

  // Update encoder position
  if ((Astate == LOW) && (Bstate == LOW)) {
    posCurrent = 0;
  } 
  else if ((Astate == HIGH) && (Bstate == LOW)) {
    posCurrent = 1;
  } 
  else if ((Astate == HIGH) && (Bstate == HIGH)) {
    posCurrent = 2;
  }

  // Update scroll value based on the change in position
  int8_t posDifference = posCurrent - posPrevious;
  if (posDifference == 2 || posDifference == -1) {
    if (scroll < 255) {
      Serial.println("Scrolled up");
    }
  } 
  else if (posDifference == 1 || posDifference == -2) {
    if (scroll > 0) {
      Serial.println("Scrolled down");
    }
  }
  else if (posDifference == 3 || posDifference  == -3){
    Serial.println("Data lost");
  }

  // Update previous position
  posPrevious = posCurrent;

  // Add a delay for stability
  vTaskDelay(pdMS_TO_TICKS(1000));
}