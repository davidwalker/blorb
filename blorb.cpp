#include <application.h>
#include "Timer.h"


// these constants won't change:
int ledPin = D7;      // led connected to digital pin 13
int knockSensor = A0; // the piezo is connected to analog pin 0
int threshold = 2000;  // threshold value to decide when the detected sound is a knock or not


// these variables will change:
int sensorReading = 0;      // variable to store the value read from the sensor pin
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


// int breath = {
//   {    0,   0,   0,   0},
//   { 1000, 255,   0,   0},
//   { 2000,   0,   0,   0}
// };
//
//


class Frame {
  public:
    Frame(long time, int r, int g, int b);
    long time;
    int color[3];
};

Frame::Frame(long t, int r, int g, int b){
  time = t;
  color[0] = r;
  color[1] = g;
  color[2] = b;
}


Frame breath[3] = {
  Frame(0, 0,0,0),
  Frame(1000, 255, 255, 255),
  Frame(2000, 0,0,0)
};


class LedAnimator{
  private:
    int nextFrame = 0;
    long startFrameTime;
    Frame* currentAnimation;
    int numFrames;
    int currentBrightness[3] = {0,0,0};
    int startBrightness[3] = {0,0,0};

    int pins[3] = {0,0,0};

    void update();

  public:
    LedAnimator(int redPin, int greenPin, int bluePin);
    void startAnimation(Frame* frames, int frameCount);
    void step();
};

LedAnimator::LedAnimator(int redPin, int greenPin, int bluePin){
  pins[0] = redPin;
  pins[1] = greenPin;
  pins[2] = bluePin;
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  update();
}

void LedAnimator::update(){
  for (int i = 0; i < 3 ; i++){
    analogWrite(pins[i], currentBrightness[i]);
  }
}

void LedAnimator::startAnimation(Frame* frames, int frameCount){
  currentAnimation = frames;
  startFrameTime = millis();
  nextFrame = 0;
  numFrames = frameCount;
  for (int i = 0; i < 3 ; i++){
    startBrightness[i] = currentBrightness[i];
  }
  step();
}

void LedAnimator::step(){
  Serial.println("step");
  long currentTime = millis();
  long targetTime = currentAnimation[nextFrame].time;
  bool advance = false;

  long timeSinceStart = currentTime - startFrameTime;
  Serial.println(timeSinceStart);
  if(timeSinceStart >= targetTime)
  {
    Serial.println("advance");
    advance = true;
    timeSinceStart = targetTime;
  }

  for (int i = 0; i < 3 ; i++){
    int targetBrightness = currentAnimation[nextFrame].color[i];
    int desiredBrightness = (targetBrightness - startBrightness[i]) * ((float)timeSinceStart / (float)targetTime);
    currentBrightness[i] = desiredBrightness;
  }

  update();

  Serial.println("b");
  Serial.println(pins[0]);

  if(advance)
  {
    nextFrame++;
    startFrameTime = currentTime;
    if(nextFrame >= numFrames) {
      nextFrame = 0;
    }
    for (int i = 0; i < 3 ; i++){
      startBrightness[i] = currentBrightness[i];
    }
    Serial.println("frame");
    Serial.println(nextFrame);
  }
}


LedAnimator led = LedAnimator(redPin,greenPin,bluePin);

int setThreshold(String s);
int setMin(String s);
int setMax(String s);
int setDivLimit(String s);
void animate(void);


void testDiff(void);

void setup() {
 Spark.variable("sensor", &sensorReading, INT);
 Spark.variable("threshold", &threshold, INT);
 Spark.variable("brightness", &brightness, INT);
 Spark.variable("fadeAmount", &fadeAmount, INT);
 Spark.variable("min", &min, INT);
 Spark.variable("max", &max, INT);
 Spark.variable("cycles", &cycles, INT);

 Spark.function("setThreshold", setThreshold);
 Spark.function("setDivLimit", setDivLimit);
 Spark.function("setMin", setMin);
 Spark.function("setMax", setMax);
 pinMode(ledPin, OUTPUT); // declare the ledPin as as OUTPUT

 pinMode(redPin, OUTPUT);
 pinMode(greenPin, OUTPUT);
 pinMode(bluePin, OUTPUT);

 Serial.begin(9600);       // use the serial port

 delay(10000);
 Serial.println("Starting in 10...");

 Serial.println(breath[1].time);
 Serial.println(breath[1].color[0]);
 Serial.println(breath[1].color[1]);
 Serial.println(breath[1].color[2]);
 Serial.println("length");
 Serial.println(sizeof(breath));
 

 delay(10000);
 


 led.startAnimation(breath, 3);

 testDiff();
}

void loop() {
  led.step();

  delay(10);
  //
  //
  // sensorReading = analogRead(knockSensor);
  //
  // if(sensorReading < 2030 || sensorReading > 2050) {
  //   Serial.println(sensorReading);
  // }
  //
  // // if the sensor reading is greater than the threshold:
  // if (sensorReading <= threshold) {
  //   // toggle the status of the ledPin:
  //   ledState = !ledState;
  //   // update the LED pin itself:
  //   digitalWrite(ledPin, ledState);
  //   // send the string "Knock!" back to the computer, followed by newline
  //   //trigger = true;
  //   Serial.println("Knock!");
  //   if(cycles == 0) {
  //     cycles = 6;
  //   }
  // }
  //
  // if(cycles > 0){
  //   //animate();
  // }
  //
  // delay(1);  // delay to avoid overloading the serial port buffer
}

void testleds(){
  Serial.println(greenPin);
  analogWrite(greenPin, 255);
  delay(1000);
  analogWrite(greenPin, 0);

  Serial.println(redPin);
  analogWrite(redPin, 255);
  delay(1000);
  analogWrite(redPin, 0);

  Serial.println(bluePin);
  analogWrite(bluePin, 255);
  delay(1000);
  analogWrite(bluePin, 0);

}

int setThreshold(String s)
{
  threshold = s.toInt();
  return threshold;
}
int setMin(String s)
{
  min = s.toInt();
  return min;
}
int setMax(String s)
{
  max = s.toInt();
  return max;
}
int setDivLimit(String s)
{
  divLimit = s.toInt();
  return divLimit;
}

void animate(){
  divider++;
  if (divider < divLimit)
    return;
  divider = 0;

  brightness += fadeAmount;

  if(brightness <= min || brightness >= max){ 
    fadeAmount = -fadeAmount;
    cycles--;
  }

  analogWrite(redPin, brightness);
  analogWrite(greenPin, brightness);
  analogWrite(bluePin, brightness);
}



void testDiff(){
  int currentBrightness = 0;
  int currentTime = 500;

  int startTime = 0;
  int targetBrightness = 200;
  int targetTime = 1000;

  int desiredBrightness = (targetBrightness - currentBrightness) * ((float)currentTime - startTime) / (targetTime-startTime);

  int expected = 100;

  Serial.println("Test results");
  Serial.println(expected);
  Serial.println(desiredBrightness);
  Serial.println(expected == desiredBrightness);
  Serial.println("end");
}




