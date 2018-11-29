// https://learn.adafruit.com/neopixels-and-servos/overview

#include <Wire.h>
#include "Adafruit_MPR121.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "Timer.h"

#if defined(__AVR_ATtiny85__)
#error "This code is for ATmega boards, see other example for ATtiny."
#endif
#include <Adafruit_TiCoServo.h>

#ifndef _BV
#define _BV(bit) (1 << (bit))
#endif

//cap touch sensor
Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

//servo
#define servoPin  9
Adafruit_TiCoServo myservo;
int pos = 0;

//NeoPixel ring
#define ringPin       3
#define NUMPixel      16
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUMPixel, ringPin, NEO_GRB + NEO_KHZ800);
int angleDegree = 0;
float sinVal = 0;
float brightness = 0;
const float pi = 3.141593;
const int delayVal = 2;
int lightCount = 0;

//2 motors and lamp
#define motorPin1  6
#define motorPin2  7
#define lampPin 2

//Overall lgihts
#define overallLightPin 10
#define NUMPixel2 8
Adafruit_NeoPixel overallLight = Adafruit_NeoPixel(NUMPixel2, overallLightPin, NEO_RGBW + NEO_KHZ800);

//Buzzer
#define buzzerPin  11


//debounce
int capState;
int lastCapState = 0;
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 20;


//timer
int currentTime;
int lastTime = 0;
const int timeFrame = 3000;
int alarm = 0;

Timer timerTest;





void setup() {
  Serial.begin(9600);
  myservo.attach(servoPin);
  ring.begin();
  ring.show();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(lampPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);

  overallLight.begin();
  overallLight.show();

  while (!Serial) {
    delay(10);
  }

  Serial.println("capacitive touch sensor test");

  // Default address is 0x5A, if tied to 3.3V its 0x5B
  // If tied to SDA its 0x5C and if SCL then 0x5D
  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121 found!");


  timerTest.pulse(motorPin1, 3 * 1000, HIGH);

}

void loop() {
  //  int touchNum = capReading();
  //  Serial.println(touchNum);

     int touchNum = getCapState();

  //  int test = timer();
  //  Serial.println(test);


  //    digitalWrite(motorPin2, HIGH);
  //     digitalWrite(motorPin1, HIGH);
  //doorTurning();


    switch (touchNum) {
      case 1:
        //turning the door
        doorTurning();
        break;
      case 2:
        //lighting up the circular mirror light
        ringLight();
        break;
      case 3:
        digitalWrite(motorPin1, HIGH);
        digitalWrite(lampPin, HIGH);
//          timerTest.update();

        break;
      case 4:
        digitalWrite(motorPin2, HIGH);
        break;
      case 5:
        phoneRing();
        break;
      case 6:
        //overall light
        lightStripOn();
        break;
      default:
        for (int i = 0; i < NUMPixel; i++) {
          ring.setPixelColor(i, ring.Color(0, 0, 0));
          ring.show();
        }
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, LOW);
        digitalWrite(lampPin, LOW);
  
        for (int i = 0; i < NUMPixel2; i++) {
          overallLight.setPixelColor(i, overallLight.Color(0, 0, 0, 0));
          overallLight.show();
        }
  
        noTone(buzzerPin);
  
        break;
    }
}

int capReading() {
  currtouched = cap.touched();
  int touchedPin;

  for (uint8_t i = 0; i < 12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      //      Serial.print(i); Serial.println(" touched");
      touchedPin = i;
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      // Serial.print(i); Serial.println(" released");
      touchedPin = 0;
    }
  }

  lasttouched = currtouched;

  return touchedPin;

  delay(500);
}

int getCapState() {
  int reading = capReading();
  //  Serial.print("reading:  ");
  //  Serial.print(reading);

  if (reading != lastCapState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != capState) {
      capState = reading;
      //      if (capState == HIGH) {
      Serial.print("  state:  ");
      Serial.println(capState);
      return capState;

      //      }
    }
  }
  lastCapState = reading;
}

void timer() {
  currentTime = millis();
  //  Serial.println(currentTime);
  if (currentTime - lastTime >= timeFrame) {
    Serial.println("times up");
    //    return 1;
    alarm = 1;
    lastTime = currentTime;
  }
}

//void motor1Turning() {
//  timer();
//  if (alarm == 1) {
//    alarm = 0;
//  } else if (alarm
//}



void doorTurning() {
  for (pos = 0; pos <= 180; pos += 1) {
    // in steps of 1 degree
    myservo.write(pos);
    delay(15);
  }
  for (pos = 180; pos >= 0; pos -= 1) {
    myservo.write(pos);
    delay(15);
  }
}

void ringLight() {
  for (angleDegree = 0; angleDegree <= 360; angleDegree++) {
    sinVal = abs(sin(angleDegree * (pi / 180)));
    brightness = map(sinVal * 100, 0, 100, 50, 200);
    brightness *= 0.001;
    //    Serial.print(sinVal);
    //    Serial.print("  +  ");
    //    Serial.println(brightness);

    for (int i = 0; i < NUMPixel; i++) {
      ring.setPixelColor(i, ring.Color(brightness * 255, brightness * 170, brightness * 20));
      ring.show();
    }
  }

  delay(delayVal);

  for (angleDegree = 360; angleDegree >= 0; angleDegree--) {
    sinVal = abs(sin(angleDegree * (pi / 180)));
    brightness = map(sinVal * 100, 100, 0, 200, 50);
    brightness *= 0.001;
    //    Serial.println(sinVal);

    for (int i = 0; i < NUMPixel; i++) {
      ring.setPixelColor(i, ring.Color(brightness * 255, brightness * 170, brightness * 20));
      ring.show();
    }
  }

  delay(delayVal);

}

//overall light
void lightStripOn() {
  for (int i = 0; i < NUMPixel2; i++) {
    overallLight.setPixelColor(i, overallLight.Color(0, 0, 0, 120));
    overallLight.show();
  }
  delay(4000);
}

//buzzer
void phoneRing() {
  tone(buzzerPin, 1000);
  delay(10);
  noTone(buzzerPin);
  delay(10);
}
