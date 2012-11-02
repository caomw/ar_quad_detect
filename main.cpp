#define  __PROFILER__

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "timeprofiler.h"
#include "cVisualizer.h"
#include "cQuadDetector.h"
#include "cMatcher.h"

int main (int argc, char * const argv[])
{

  bool useCamera = true;
  char * filename;
  if(argc >=2)
    {
      filename = argv[1];
      useCamera = false;
    }
  cv::VideoCapture cap;
  if( useCamera )
    {
      cap.open(0);
    }	
  else
    {
      cap.open(filename);
    }
  if(!cap.isOpened())
    {
      return -1;
    }
#ifndef __DEBUG_INFO__
  cVisualizer vz("Quad Detector");
#endif
  cv::Mat color_frame,gray_frame;
  cQuadDetector qd;
  cMatch matcher;
  qd.init(480,640);
  matcher.init();
  int i = 0;
  while (1)
    {		
      cap>>color_frame;
      if(color_frame.empty())
	break;
      if( i++ < 0)
	continue;
      
#ifdef __DEBUG_INFO__
      qd.set_debug_img(color_frame);
      matcher.set_debug_img(color_frame);
#endif

      cv::cvtColor(color_frame,gray_frame,CV_RGB2GRAY);
      
      PROFILE_START(DETECT_OPT);
      qd.detect(gray_frame);
      matcher.update_match(qd.QuadList);
      PROFILE_END(DETECT_OPT);
      
#ifndef __DEBUG_INFO__
      vz.set_img(color_frame);
      /*
      for(int i = 0 ; i < qd.h_regions; ++i)
	{
	  for(int j = 0; j < qd.w_regions; ++j)
	    {
	      vz.draw_edgepixel_vector(qd.EdgePixelList[i][j],CV_COLOR_GREEN);
	    }
	}
      //*/
      //vz.draw_linesegment_vector(qd.LineList,CV_COLOR_YELLOW);
      //vz.draw_quadrilateral_vector(qd.QuadList);
      vz.draw_quadrilateral_vector(matcher.QuadList);
      //vz.show_img(false);
      vz.show_img();
#endif
    }	
  matcher.release();
  qd.release();
  return 0;
}
