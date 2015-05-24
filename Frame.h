#ifndef FRAME_H
#define FRAME_H

class Frame {
  public:
    Frame(long time, int r, int g, int b);
    long time;
    int color[3];
};

#endif
