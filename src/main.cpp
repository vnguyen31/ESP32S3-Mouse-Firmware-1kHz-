#include "USB.h"
#include "USBHIDMouse.h"
#include <Arduino.h>
#include <SPI.h>
#include "esp32-hal-adc.h"
#include "srom.h"
#include "pgmspace.h"
#include "esp_event.h"

// Pin definitions
#define BUTTON1     8
#define BUTTON2     3
#define ENCODER_A   47
#define ENCODER_B   21
#define MISO_PIN    12
#define MOSI_PIN    11
#define SCLK_PIN    13
#define NCS_PIN     10

// Sensor register definitions
#define Product_ID  0x00
#define Revision_ID 0x01
#define Motion  0x02
#define Delta_X_L 0x03
#define Delta_X_H 0x04
#define Delta_Y_L 0x05
#define Delta_Y_H 0x06
#define SQUAL 0x07
#define Raw_Data_Sum  0x08
#define Maximum_Raw_data  0x09
#define Minimum_Raw_data  0x0A
#define Shutter_Lower 0x0B
#define Shutter_Upper 0x0C
#define Control 0x0D
#define Config1 0x0F
#define Config2 0x10
#define Angle_Tune  0x11
#define Frame_Capture 0x12
#define SROM_Enable 0x13
#define Run_Downshift 0x14
#define Rest1_Rate_Lower  0x15
#define Rest1_Rate_Upper  0x16
#define Rest1_Downshift 0x17
#define Rest2_Rate_Lower  0x18
#define Rest2_Rate_Upper  0x19
#define Rest2_Downshift 0x1A
#define Rest3_Rate_Lower  0x1B
#define Rest3_Rate_Upper  0x1C
#define Observation 0x24
#define Data_Out_Lower  0x25
#define Data_Out_Upper  0x26
#define Raw_Data_Dump 0x29
#define SROM_ID 0x2A
#define Min_SQ_Run  0x2B
#define Raw_Data_Threshold  0x2C
#define Config5 0x2F
#define Power_Up_Reset  0x3A
#define Shutdown  0x3B
#define Inverse_Product_ID  0x3F
#define LiftCutoff_Tune3  0x41
#define Angle_Snap  0x42
#define LiftCutoff_Tune1  0x4A
#define Motion_Burst  0x50
#define LiftCutoff_Tune_Timeout 0x58
#define LiftCutoff_Tune_Min_Length  0x5A
#define SROM_Load_Burst 0x62
#define Lift_Config 0x63
#define Raw_Data_Burst  0x64
#define LiftCutoff_Tune2  0x65

//ADC variables
volatile uint16_t button1Voltage;
volatile uint16_t button2Voltage;

// Scroll wheel encoder variables
uint8_t posCurrent = 0;
uint8_t posPrevious = 0;
uint8_t scroll = 0;
uint8_t Astate = 0;
uint8_t Bstate = 0;

//flag to track buttons' state
bool button1Pressed = false;
bool button2Pressed = false;

//SPI constants
static const int spi_Clock = 4000000;
SPIClass *fspi = NULL;
uint16_t cur_time;
uint16_t prev_time;
uint16_t elapsed_time;
uint16_t deltaX = 0;
uint16_t deltaY = 0;
int16_t DELTAX = 0;
int16_t DELTAY = 0;

//Sensor variables
int8_t posDifference;
int16_t xydata[4];
int32_t xydata_total[2];

//USB
USBHIDMouse AnalogMouse;

// Declare spiRead function prototype
uint8_t spiRead(uint8_t address);

//SPI read function
uint8_t spiread(uint8_t address){
  //chip select
  digitalWrite(NCS_PIN, LOW); 

  //send address
  fspi->transfer(address); 

  //read data from register
  uint8_t regValue = fspi->transfer(0x00);

  // Release chip select
  digitalWrite(NCS_PIN, HIGH);
  return regValue;
}

//SPI write function
void spiwrite(uint8_t address, uint8_t data) {
  // Prepare the data array
  uint8_t txdata[2] = {static_cast<uint8_t>(address | 0x80), data};
  
  // Chip select
  digitalWrite(NCS_PIN, LOW);

  // Send data
  fspi->transfer(txdata, 2); // Transfer the array with a length of 2 bytes

  // Chip select high
  digitalWrite(NCS_PIN, HIGH);
}

//Start initialization on PMW3389
static void PMW3389_init(const uint8_t DPI){
  //srom 
  digitalWrite(NCS_PIN, HIGH);
  delay(3);

  //shut down first
  digitalWrite(NCS_PIN, LOW);
  spiwrite(0x3b, 0xb6);
  digitalWrite(NCS_PIN, HIGH);
  delay(300);

  //reset serial port 
  digitalWrite(NCS_PIN, LOW);
  delayMicroseconds(40);
  digitalWrite(NCS_PIN, HIGH);
  delayMicroseconds(40);

  //powerup reset
  digitalWrite(NCS_PIN, LOW);
  spiwrite(0x3a, 0x5a);
  digitalWrite(NCS_PIN, HIGH);
  delay(50);

  //read motion registers
  spiread(0x02);
  spiread(0x03);
  spiread(0x04);
  spiread(0x05);
  spiread(0x06);

  //srom download
  spiwrite(0x10, 0x00);
  spiwrite(0x13, 0x1d);
  digitalWrite(NCS_PIN, HIGH);
  delay(10);
  digitalWrite(NCS_PIN, LOW);
  spiwrite(0x13, 0x18);
  //burst download
  digitalWrite(NCS_PIN, LOW);
  fspi->transfer(SROM_Load_Burst | 0x80);
  delayMicroseconds(15);
  //send all bytes
  unsigned char c;
  for (uint16_t i = 0; i < firmware_length; i++){
    c = (unsigned char)pgm_read_byte(firmware_data + i);
    fspi->transfer(c);
    delayMicroseconds(15);
  }
  spiread(SROM_ID);
  Serial.println(SROM_ID);
  digitalWrite(NCS_PIN, HIGH);
  delayMicroseconds(200);

  //sensor configurations, settings:
  digitalWrite(NCS_PIN, LOW);
  spiwrite(0x10, 0x00);
  spiwrite(0x0d, 0x00); 
	spiwrite(0x11, 0x00); 
	spiwrite(0x0f, DPI); 
	spiwrite(0x63, 0x03); 
	spiwrite(0x2b, 0x10); 
	spiwrite(0x2c, 0x0a); 
	digitalWrite(NCS_PIN, HIGH);
	delayMicroseconds(200);
}

//display motion sensor registers:
void register_display(void){
  int reg_addr[7] = {0x00, 0x3F, 0x2A, 0x02};
  const char* reg_names[] = {"Product_ID", "Inverse_Product_ID", "SROM_Version", "Motion"};
  byte reg_results;

  digitalWrite(NCS_PIN, LOW);

  int reg_counter = 0;
  for (reg_counter = 0; reg_counter < 4; reg_counter++) {
    fspi->transfer(reg_addr[reg_counter]);
    delay(1);
    Serial.println("---");
    Serial.println(reg_names[reg_counter]);
    Serial.println(reg_addr[reg_counter], HEX);
    reg_results = fspi->transfer(0);
    Serial.println(reg_results, BIN);
    Serial.println(reg_results, HEX);
    delay(1);
  }
  digitalWrite(NCS_PIN, HIGH);
}

//function to convert two's complement
int twoscomp_convert(int data){
  if(data & 0x80){
    data = -1 * ((data ^ 0xff) + 1);
  }
  return data;
}


//Single Setup Part////////////////////////////////////////////////////////////
void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  //ADC configurations
  analogSetClockDiv(1);

  // Setup encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  //SPI initialization
  fspi = new SPIClass(FSPI);
  fspi->begin(SCLK_PIN, MISO_PIN, MOSI_PIN);
  pinMode(NCS_PIN, OUTPUT);
  fspi->setFrequency(spi_Clock);
  fspi->beginTransaction(SPISettings(spi_Clock, MSBFIRST, SPI_MODE3));

  //PMW3389 initialization
  PMW3389_init(7);

  //display initialization results:
  register_display();

  //initialize the USBHID interface
  AnalogMouse.begin();
  USB.begin();
  Serial.println("Human Interface Device initialized!");
  delay(2000);

}


////////////////////////////MAIN LOOP///////////////////////////////////////////
void loop() {

  // Read analog values from buttons
  button1Voltage = analogRead(BUTTON1);
  button2Voltage = analogRead(BUTTON2);

  // Check if Button1 is pressed
  if (button1Voltage > 2000) {
    if (!button1Pressed) {
      Serial.println("Button1 Pressed");
      button1Pressed = true;
      AnalogMouse.press(MOUSE_RIGHT);
    }
  } 
  else {
    if (button1Pressed && button1Voltage < 3000) {
      Serial.println("Button1 Released");
      button1Pressed = false;
      AnalogMouse.release(MOUSE_RIGHT);
    }
  }

  // Check if Button 2 is pressed
  if (button2Voltage > 2000) {
    if (!button2Pressed) {
      Serial.println("Button2 Pressed");
      button2Pressed = true;
      AnalogMouse.press(MOUSE_LEFT);
    }
  } 
  else {
    if (button2Pressed && button2Voltage < 3000) {
      Serial.println("Button2 Released");
      button2Pressed = false;
      AnalogMouse.release(MOUSE_LEFT);
    }
  }



  // Read encoder state
  Astate = digitalRead(ENCODER_A);
  Bstate = digitalRead(ENCODER_B);

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
  posDifference = posCurrent - posPrevious;
  if (posDifference == 2 || posDifference == -1) {
    scroll += 1;
    Serial.println("scroll up");
  } 
  else if (posDifference == 1 || posDifference == -2) {
    scroll -= 1;
    Serial.println("scroll down");
  }
  else if (posDifference == 3 || posDifference  == -3){
    Serial.println("Data lost");
  }

  // Update previous position
  posPrevious = posCurrent;


  //SENSOR STUFF:
  digitalWrite(NCS_PIN, LOW);
  spiwrite(Motion, 0x01);
  spiread(Motion);

  xydata[0] = (uint8_t)spiread(Delta_X_L);
  xydata[1] = (uint8_t)spiread(Delta_X_H);
  xydata[2] = (uint8_t)spiread(Delta_Y_L);
  xydata[3] = (uint8_t)spiread(Delta_Y_H);

  xydata_total[0] = (xydata[1] << 8) | xydata[0];
  xydata_total[1] = (xydata[3] << 8) | xydata[2];

  DELTAX -= xydata_total[0];
  DELTAY -= xydata_total[1];


  if(xydata_total[0] != 0 || xydata_total[1] != 0 || scroll != 0){
    Serial.print("x = ");
    Serial.print(DELTAX);
    Serial.print(" | ");
    Serial.print("y = ");
    Serial.println(DELTAY);
    AnalogMouse.move(DELTAX, DELTAY, scroll);
  }  

  DELTAX = DELTAY = scroll = 0;
}