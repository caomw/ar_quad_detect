#ifndef __CLINESEGMENTCLIP_H__
#define __CLINESEGMENTCLIP_H__

#include "commonDef.h"

class cLineSegmentClip
{
 public:
  void SetBoundary(int xMin,int yMin,int xMax, int yMax)
  {
    xmin = xMin;
    ymin = yMin;
    xmax = xMax;
    ymax = yMax;
  }

  bool CohenSutherlandLineClip(edgePixel &start, edgePixel &end);

 private:
  int ComputeOutCode(float x, float y);

 private:
  int xmin;
  int ymin;
  int xmax;
  int ymax;

  static  const int INSIDE = 0; // 0000
  static  const int LEFT = 1;   // 0001
  static  const int RIGHT = 2;  // 0010
  static  const int BOTTOM = 4; // 0100
  static  const int TOP = 8;    // 100

};

#endif

