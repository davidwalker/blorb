#include "Frame.h"

#ifndef LEDANIMATOR_H
#define LEDANIMATOR_H

class LedAnimator{
  private:
    int currentFrame = 0;
    long startFrameTime;
    Frame* currentAnimation;
    int numFrames;
    int currentBrightness[3] = {0,0,0};
    int startBrightness[3] = {0,0,0};

    int pins[3] = {0,0,0};

    int cycles = -1;

    void update();

  public:
    LedAnimator(int redPin, int greenPin, int bluePin);
    void startAnimation(Frame* frames, int frameCount, int cycleCount);
    void step();
    bool running();
    void setColor(int r, int g, int b);
    void stop();
};

#endif
