#include "LedAnimator.h"
#include "Application.h"

LedAnimator::LedAnimator(int redPin, int greenPin, int bluePin){
  pins[0] = redPin;
  pins[1] = greenPin;
  pins[2] = bluePin;
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  update();
}

void LedAnimator::setColor(int r, int g, int b){
  currentBrightness[0] = r;
  currentBrightness[1] = g;
  currentBrightness[2] = b;
  update();
}

void LedAnimator::stop(){
  cycles = 0;
}

void LedAnimator::update(){
  for (int i = 0; i < 3 ; i++){
    analogWrite(pins[i], currentBrightness[i]);
  }
}

void LedAnimator::startAnimation(Frame* frames, int frameCount, int cycleCount){
  currentAnimation = frames;
  startFrameTime = millis();
  currentFrame = 0;
  numFrames = frameCount;
  cycles = cycleCount;
  // for (int i = 0; i < 3 ; i++){
  //   startBrightness[i] = currentBrightness[i] = frames[currentFrame].color[i];
  // }
  step();
}

bool LedAnimator::running(){
  return cycles != 0;
}

void LedAnimator::step(){
  if(!running()){
    return;
  }
  //Serial.println("step");
  long currentTime = millis();
  long targetTime = currentAnimation[currentFrame].time;
  long timeSinceStart = currentTime - startFrameTime;
  //Serial.println(timeSinceStart);
  if(timeSinceStart >= targetTime)
  {
    // reached end time for current frame so advance
    // Serial.println("advance");
    // Serial.println(currentFrame);
    for (int i = 0; i < 3 ; i++){
      startBrightness[i] = currentBrightness[i] = currentAnimation[currentFrame].color[i];
      // Serial.println(startBrightness[i]);
    }
    update();
    currentFrame++;
    startFrameTime = currentTime;
    if(currentFrame >= numFrames) {
      currentFrame = 0;
      if(cycles>0)
        cycles--;
    }
    return;
  }

  for (int i = 0; i < 3 ; i++){
    int targetBrightness = currentAnimation[currentFrame].color[i];
    if(targetBrightness == startBrightness[i])
      continue;

    int desiredBrightness = (targetBrightness - startBrightness[i]) * ((float)timeSinceStart / (float)targetTime);
    currentBrightness[i] = desiredBrightness;
  }

  update();

  // Serial.println("b");
  // Serial.println(pins[0]);
}

