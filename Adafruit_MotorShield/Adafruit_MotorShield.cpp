/******************************************************************
 This is the library for the Adafruit Motor Shield V2 for Arduino. 
 It supports DC motors & Stepper motors with microstepping as well
 as stacking-support. It is *not* compatible with the V1 library!

 It will only work with https://www.adafruit.com/products/1483
 
 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source hardware
 by purchasing products from Adafruit!
 
 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, check license.txt for more information.
 All text above must be included in any redistribution.
 
 Note: Port to mbed by Grady Hillhouse 2015
 ******************************************************************/

#include "Adafruit_MotorShield.h"
#include "mbed.h"

#if (MICROSTEPS == 8)
uint8_t microstepcurve[] = {0, 50, 98, 142, 180, 212, 236, 250, 255};
#elif (MICROSTEPS == 16)
uint8_t microstepcurve[] = {0, 25, 50, 74, 98, 120, 141, 162, 180, 197, 212, 225, 236, 244, 250, 253, 255};
#endif

Adafruit_MotorShield::Adafruit_MotorShield(uint8_t addr) : _pwm(Adafruit_PWMServoDriver(I2C_SDA, I2C_SCL, addr)) {
  _addr = addr;
}

void Adafruit_MotorShield::begin(uint16_t freq) {
  _pwm.begin();
  _freq = freq;
  _pwm.setPWMFreq(_freq);  // This is the maximum PWM frequency
  for (uint8_t i=0; i<16; i++) 
    _pwm.setPWM(i, 0, 0);
}

void Adafruit_MotorShield::setPWM(uint8_t pin, uint16_t value) {
  if (value > 4095) {
    _pwm.setPWM(pin, 4096, 0);
  } else 
    _pwm.setPWM(pin, 0, value);
}

void Adafruit_MotorShield::setPin(uint8_t pin, bool value) {
  if (value == 0)
    _pwm.setPWM(pin, 0, 0);
  else
    _pwm.setPWM(pin, 4096, 0);
}

Adafruit_DCMotor *Adafruit_MotorShield::getMotor(uint8_t num) {
  if (num > 4) return NULL;

  num--;

  if (dcmotors[num].motornum == 0) {
    // not init'd yet!
    dcmotors[num].motornum = num;
    dcmotors[num].MC = this;
    uint8_t pwm, in1, in2;
    if (num == 0) {
      pwm = 8; in2 = 9; in1 = 10;
    } else if (num == 1) {
      pwm = 13; in2 = 12; in1 = 11;
    } else if (num == 2) {
      pwm = 2; in2 = 3; in1 = 4;
    } else if (num == 3) {
      pwm = 7; in2 = 6; in1 = 5;
    }
    dcmotors[num].PWMpin = pwm;
    dcmotors[num].IN1pin = in1;
    dcmotors[num].IN2pin = in2;
  }
  return &dcmotors[num];
}


Adafruit_StepperMotor *Adafruit_MotorShield::getStepper(uint16_t steps, uint8_t num) {
  if (num > 2) return NULL;

  num--;

  if (steppers[num].steppernum == 0) {
    // not init'd yet!
    steppers[num].steppernum = num;
    steppers[num].revsteps = steps;
    steppers[num].MC = this;
    uint8_t pwma, pwmb, ain1, ain2, bin1, bin2;
    if (num == 0) {
      pwma = 8; ain2 = 9; ain1 = 10;
      pwmb = 13; bin2 = 12; bin1 = 11;
    } else if (num == 1) {
      pwma = 2; ain2 = 3; ain1 = 4;
      pwmb = 7; bin2 = 6; bin1 = 5;
    }
    steppers[num].PWMApin = pwma;
    steppers[num].PWMBpin = pwmb;
    steppers[num].AIN1pin = ain1;
    steppers[num].AIN2pin = ain2;
    steppers[num].BIN1pin = bin1;
    steppers[num].BIN2pin = bin2;
  }
  return &steppers[num];
}


/******************************************
               MOTORS
******************************************/

Adafruit_DCMotor::Adafruit_DCMotor(void) {
  MC = NULL;
  motornum = 0;
  PWMpin = IN1pin = IN2pin = 0;
}

void Adafruit_DCMotor::run(uint8_t cmd) {
  switch (cmd) {
  case FORWARD:
    MC->setPin(IN2pin, 0);  // take low first to avoid 'break'
    MC->setPin(IN1pin, 1);
    break;
  case BACKWARD:
    MC->setPin(IN1pin, 0);  // take low first to avoid 'break'
    MC->setPin(IN2pin, 1);
    break;
  case RELEASE:
    MC->setPin(IN1pin, 0);
    MC->setPin(IN2pin, 0);
    break;
  }
}

void Adafruit_DCMotor::setSpeed(uint8_t speed) {
  MC->setPWM(PWMpin, speed*16);
}

/******************************************
               STEPPERS
******************************************/

Adafruit_StepperMotor::Adafruit_StepperMotor(void) {
  revsteps = steppernum = currentstep = 0;
}
/*

uint16_t steps, Adafruit_MotorShield controller)  {

  revsteps = steps;
  steppernum = 1;
  currentstep = 0;

  if (steppernum == 1) {
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    
    // enable both H bridges
    pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);

    // use PWM for microstepping support
    MC->setPWM(1, 255);
    MC->setPWM(2, 255);

  } else if (steppernum == 2) {
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B) &
      ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // all motor pins to 0

    // enable both H bridges
    pinMode(5, OUTPUT);
    pinMode(6, OUTPUT);
    digitalWrite(5, HIGH);
    digitalWrite(6, HIGH);

    // use PWM for microstepping support
    // use PWM for microstepping support
    MC->setPWM(3, 255);
    MC->setPWM(4, 255);
  }
}
*/

void Adafruit_StepperMotor::setSpeed(uint16_t rpm) {
  printf("Steps per rev: %d\n", revsteps);
  printf("RPM: %d\n", rpm);

  usperstep = 60000000 / ((uint32_t)revsteps * (uint32_t)rpm);
  steppingcounter = 0;
}

void Adafruit_StepperMotor::release(void) {
  MC->setPin(AIN1pin, 0);
  MC->setPin(AIN2pin, 0);
  MC->setPin(BIN1pin, 0);
  MC->setPin(BIN2pin, 0);
  MC->setPWM(PWMApin, 0);
  MC->setPWM(PWMBpin, 0);
}

void Adafruit_StepperMotor::step(uint16_t steps, uint8_t dir,  uint8_t style) {
  //printf("Stepping %d steps in the %d direction using style %d.\n", steps, dir, style);
  
  uint32_t uspers = usperstep;
  uint8_t ret = 0;

  if (style == INTERLEAVE) {
    uspers /= 2;
  }
 else if (style == MICROSTEP) {
    uspers /= MICROSTEPS;
    steps *= MICROSTEPS;
  }

  while (steps--) {
    ret = onestep(dir, style);
    wait_us(uspers);
    steppingcounter += (uspers % 1000);
    if (steppingcounter >= 1000) {
      wait_us(1000);
      steppingcounter -= 1000;
    }
  }
}

uint8_t Adafruit_StepperMotor::onestep(uint8_t dir, uint8_t style) {
  uint8_t a, b, c, d;
  uint8_t ocrb, ocra;

  ocra = ocrb = 255;
  
  // next determine what sort of stepping procedure we're up to
  if (style == SINGLE) {
    if ((currentstep/(MICROSTEPS/2)) % 2) { // we're at an odd step, weird
      if (dir == FORWARD) {
    currentstep += MICROSTEPS/2;
      }
      else {
    currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next even step
      if (dir == FORWARD) {
    currentstep += MICROSTEPS;
      }
      else {
    currentstep -= MICROSTEPS;
      }
    }
  } else if (style == DOUBLE) {
    if (! (currentstep/(MICROSTEPS/2) % 2)) { // we're at an even step, weird
      if (dir == FORWARD) {
    currentstep += MICROSTEPS/2;
      } else {
    currentstep -= MICROSTEPS/2;
      }
    } else {           // go to the next odd step
      if (dir == FORWARD) {
    currentstep += MICROSTEPS;
      } else {
    currentstep -= MICROSTEPS;
      }
    }
  } else if (style == INTERLEAVE) {
    if (dir == FORWARD) {
       currentstep += MICROSTEPS/2;
    } else {
       currentstep -= MICROSTEPS/2;
    }
  } 

  if (style == MICROSTEP) {
    if (dir == FORWARD) {
      currentstep++;
    } else {
      // BACKWARDS
      currentstep--;
    }

    currentstep += MICROSTEPS*4;
    currentstep %= MICROSTEPS*4;

    ocra = ocrb = 0;
    if ( (currentstep >= 0) && (currentstep < MICROSTEPS)) {
      ocra = microstepcurve[MICROSTEPS - currentstep];
      ocrb = microstepcurve[currentstep];
    } else if  ( (currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2)) {
      ocra = microstepcurve[currentstep - MICROSTEPS];
      ocrb = microstepcurve[MICROSTEPS*2 - currentstep];
    } else if  ( (currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3)) {
      ocra = microstepcurve[MICROSTEPS*3 - currentstep];
      ocrb = microstepcurve[currentstep - MICROSTEPS*2];
    } else if  ( (currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4)) {
      ocra = microstepcurve[currentstep - MICROSTEPS*3];
      ocrb = microstepcurve[MICROSTEPS*4 - currentstep];
    }
  }

  currentstep += MICROSTEPS*4;
  currentstep %= MICROSTEPS*4;


  //printf("current step: %d\n", currentstep);
  //printf(" pwmA = %d\n", ocra); 
  //printf(" pwmB = %d\n", ocrb); 

  MC->setPWM(PWMApin, ocra*16);
  MC->setPWM(PWMBpin, ocrb*16);
  

  // release all
  uint8_t latch_state = 0; // all motor pins to 0

  //Serial.println(step, DEC);
  if (style == MICROSTEP) {
    if ((currentstep >= 0) && (currentstep < MICROSTEPS))
      latch_state |= 0x03;
    if ((currentstep >= MICROSTEPS) && (currentstep < MICROSTEPS*2))
      latch_state |= 0x06;
    if ((currentstep >= MICROSTEPS*2) && (currentstep < MICROSTEPS*3))
      latch_state |= 0x0C;
    if ((currentstep >= MICROSTEPS*3) && (currentstep < MICROSTEPS*4))
      latch_state |= 0x09;
  } else {
    switch (currentstep/(MICROSTEPS/2)) {
    case 0:
      latch_state |= 0x1; // energize coil 1 only
      break;
    case 1:
      latch_state |= 0x3; // energize coil 1+2
      break;
    case 2:
      latch_state |= 0x2; // energize coil 2 only
      break;
    case 3:
      latch_state |= 0x6; // energize coil 2+3
      break;
    case 4:
      latch_state |= 0x4; // energize coil 3 only
      break; 
    case 5:
      latch_state |= 0xC; // energize coil 3+4
      break;
    case 6:
      latch_state |= 0x8; // energize coil 4 only
      break;
    case 7:
      latch_state |= 0x9; // energize coil 1+4
      break;
    }
  }

  //printf("Latch: 0x%X\n", latch_state);

  if (latch_state & 0x1) {
   // Serial.println(AIN2pin);
    MC->setPin(AIN2pin, 1);
  } else {
    MC->setPin(AIN2pin, 0);
  }
  if (latch_state & 0x2) {
    MC->setPin(BIN1pin, 1);
   // Serial.println(BIN1pin);
  } else {
    MC->setPin(BIN1pin, 0);
  }
  if (latch_state & 0x4) {
    MC->setPin(AIN1pin, 1);
   // Serial.println(AIN1pin);
  } else {
    MC->setPin(AIN1pin, 0);
  }
  if (latch_state & 0x8) {
    MC->setPin(BIN2pin, 1);
   // Serial.println(BIN2pin);
  } else {
    MC->setPin(BIN2pin, 0);
  }

  return currentstep;
}