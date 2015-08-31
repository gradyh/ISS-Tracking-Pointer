/*************************************************** 
  This is a library for our Adafruit 16-channel PWM & Servo driver

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These displays use I2C to communicate, 2 pins are required to  
  interface. For Arduino UNOs, thats SCL -> Analog 5, SDA -> Analog 4

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
 /*****************************
  This program was ported from https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library.
  I also added some functions.
  Shundo Kishi
 *****************************/

#ifndef _ADAFRUIT_PWMServoDriver_H
#define _ADAFRUIT_PWMServoDriver_H

#include "mbed.h"
#include <cmath>

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

#define ALLLED_ON_L 0xFA
#define ALLLED_ON_H 0xFB
#define ALLLED_OFF_L 0xFC
#define ALLLED_OFF_H 0xFD


class Adafruit_PWMServoDriver {
 public:
  Adafruit_PWMServoDriver(PinName sda, PinName scl, int addr = 0x80); //0b 1_000000_(R/W) <- default slave adress
  void i2c_probe(void);
  void begin(void);
  void setI2Cfreq(int freq);
  void reset(void);
  void setPWMFreq(float freq);
  void setPrescale(uint8_t prescale);
  void setPWM(uint8_t num, uint16_t on, uint16_t off);
  void setDuty(uint8_t num, uint16_t duty);
  uint8_t readMODE1(void);
  uint8_t readPRESCALE(void);
  void printRegister(uint8_t REG);

 private:
  int _i2caddr;
  I2C i2c;

  uint8_t read8(char addr);
  void write8(char addr, char d);
};

#endif