#ifndef __CVISUALIZER_H__
#define __CVISUALIZER_H__

#define CV_COLOR_RED cv::Scalar(0,0,255,0)
#define CV_COLOR_BLUE cv::Scalar(255,0,0,0)
#define CV_COLOR_GREEN cv::Scalar(0,255,0,0)
#define CV_COLOR_WHITE cv::Scalar(255,255,255,0)
#define CV_COLOR_YELLOW cv::Scalar(0,255,255,0)

#include <cstdlib>
#include <string>
#include <opencv2/core/core.hpp>
#include "commonDef.h"

class cVisualizer
{
 public:
  cVisualizer(const std::string name=std::string());
  void set_img(const cv::Mat & in);
  void show_img(bool wait = true);
  void clear_img();

  void draw_edgepixel(edgePixel pixel, cv::Scalar color, int scale = 2);
  void draw_edgepixel_vector(EdgePixelVector list, cv::Scalar color, int scale = 2);

  void draw_connecting_linesegment(edgePixel start, edgePixel end,  cv::Scalar color, int scale = 2);

  void draw_linesegment(lineSegment line, cv::Scalar color, int scale = 2);
  void draw_linesegment_vector(LineSegmentVector list,cv::Scalar color, int scale = 2);
  
  void draw_arrow(lineSegment line, cv::Scalar color, int scale=2);
  void draw_linesegment_vector_arrows(LineSegmentVector list,cv::Scalar color, int scale = 2);
    
  void draw_quadrilateral(quadrilateral q, cv::Scalar color, int scale = 2);
  void draw_quadrilateral_vector(QuadrilateralVector list, bool mono_colored = false,int scale = 2);

 public:
  cv::Mat _img;

 private:
  std::string _window_name;
  //  cv::Mat _img;
  cv::Mat _scratch;
};

#endif
