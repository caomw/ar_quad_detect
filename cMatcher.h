#ifndef __CMATCHER_H__
#define __CMATCHER_H__
#include "commonDef.h"

#ifdef __DEBUG_INFO__
#include "cVisualizer.h"  
#endif

class cMatch
{
 public:
  void init();
  void release();
  void update_match(const QuadrilateralVector & quads);
  QuadrilateralVector QuadList;
 private:
  QuadrilateralVector _prev_quads;
  unsigned int _running_id;
   
#ifdef __DEBUG_INFO__
 private:
  cVisualizer * _debug;
 public:
  void set_debug_img(const cv::Mat & img);
#endif
  
};

#endif
