#include "application.h"
#include "Timer.h"
#include "Frame.h"
#include "LedAnimator.h"


// these constants won't change:
int ledPin = D7;      // led connected to digital pin 13
int knockSensor = A0; // the piezo is connected to analog pin 0
int threshold = 1000;  // threshold value to decide when the detected sound is a knock or not


// these variables will change:
int sensorReading = 0;      // variable to store the value read from the sensor pin
int motionDiff = 0;
int ledState = LOW;         // variable used to store the last LED status, to toggle the light


int redPin = D0;
int bluePin = D1;
int greenPin = A1;

int min = 0;
int max = 255;
int brightness = min;
int fadeAmount = 1;

int divider = 0;
int divLimit = 0;

int cycles = 2;

int basePin = A2;
int baseProximity = 0;
bool baseInRange = false;
bool onBase = false;


Frame breath[3] = {
  Frame(0, 0,0,0),
  Frame(500, 255, 255, 255),
  Frame(500, 0,0,0)
};

Frame flash[5] = {
  Frame(1, 255,0,0),
  Frame(999, 255,0,0),
  Frame(1000, 0, 255, 0),
  Frame(1999, 0, 255, 0),
  Frame(2000, 255, 0, 0)
};

int const RAIN_SIZE = 13;
Frame rain[RAIN_SIZE] = {
  Frame(0, 0,0,0),
  Frame(1000, 255, 255, 255),
  Frame(1000, 0,0,0),
  Frame(1000, 255, 0, 0),
  Frame(1000, 0,0,0),
  Frame(1000, 0, 255, 0),
  Frame(1000, 0,0,0),
  Frame(1000, 0, 0, 255),
  Frame(1000, 0, 0, 0),
  Frame(1000, 255, 0, 0),
  Frame(1000, 0, 255, 0),
  Frame(1000, 0, 0, 255),
  Frame(1000, 0, 0, 0)
};

Frame slowBreath[3] = {
  Frame(0, 0,0,0),
  Frame(2000, 255, 255, 255),
  Frame(2000, 0,0,0)
};

int const CRAZY_SPEED = 100;
int const CRAZY_SIZE = 26;
Frame crazy[CRAZY_SIZE] = {
  Frame(0, 0,0,0),
  // flash red/green for a while
  Frame(CRAZY_SPEED, 0, 255, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 255, 0, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 0, 255, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 255, 0, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 0, 255, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 255, 0, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 0, 255, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 255, 0, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 0, 255, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(CRAZY_SPEED, 255, 0, 0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(1000, 0,0,0),
  // then green for all clear
  Frame(CRAZY_SPEED, 0,255,0),
  Frame(2000, 0,255,0),
  Frame(CRAZY_SPEED, 0,0,0),
  Frame(1000, 0,0,0)
};

// todo:
//
// slow breathing
// constant on when picked up
//


LedAnimator led = LedAnimator(redPin,greenPin,bluePin);

int setThreshold(String s);
int setMin(String s);
int setMax(String s);
int setDivLimit(String s);
void detectKnock(void);
void detectBase(void);
void animate(void);

void testDiff(void);

void setup() {
  Spark.variable("sensor", &sensorReading, INT);
  Spark.variable("motionDiff", &motionDiff, INT);
  Spark.variable("threshold", &threshold, INT);
  Spark.variable("baseProximity", &baseProximity, INT);
  Spark.variable("onBase", &onBase, INT); 
  Spark.variable("baseInRange", &baseInRange, INT);

  Spark.function("setThreshold", setThreshold);
  pinMode(ledPin, OUTPUT);
  pinMode(knockSensor, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  pinMode(basePin, INPUT_PULLDOWN);

  Serial.begin(9600);

  delay(1000);
  Serial.println("Starting...");

  led.startAnimation(breath, 3, 1);
  while(led.running()) { 
    delay(10);
    led.step();
  }
}

void loop() {
  detectBase();

  if(baseInRange) {
    if(onBase ) {
      Serial.println("on base");
      if(!led.running()){
        led.startAnimation(slowBreath, 3, -1);
      }
    } else{
      Serial.println("in range but not on");
      int onBaseThreshold = 4000;
      int brightness = (baseProximity / onBaseThreshold) * 255;
      led.stop();
      led.setColor(brightness ,brightness ,brightness );
    }
  } else {
    Serial.println("base not in range");
    detectKnock();
    if(!led.running()){
      Serial.println("anim not running");
      led.startAnimation(breath, 3, -1);
    }
  }

  led.step();

  delay(10);
}

void detectKnock(){
  int newReading  = analogRead(knockSensor);

  motionDiff = newReading - sensorReading;
  motionDiff = motionDiff < 0 ? -motionDiff : motionDiff;

  if (motionDiff >= threshold) {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    Serial.println("Knock!");
    Serial.println(motionDiff);
    // start monster detection
    led.startAnimation(crazy, CRAZY_SIZE, 1);
  }
  sensorReading = newReading;
}

void detectBase(){
  baseProximity = analogRead(basePin);

  baseInRange = baseProximity > 100;
  onBase = baseProximity  > 4000;

  // Serial.println("Base");
  // Serial.println(baseProximity);
  // Serial.println(baseInRange);
  // Serial.println(onBase);
}

int setThreshold(String s)
{
  threshold = s.toInt();
  return threshold;
}
