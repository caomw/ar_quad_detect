#include "cVisualizer.h"
#include <opencv2/highgui/highgui.hpp>

#define SCALE 10

void cVisualizer::draw_quadrilateral_vector(QuadrilateralVector list,bool mono_colored, int scale)
{
  cv::Scalar c[] = {CV_COLOR_RED,CV_COLOR_GREEN,CV_COLOR_BLUE,CV_COLOR_YELLOW};
  const int count = 4;
  const int quad_count = list.size();
  for(int i = 0 ; i < quad_count; ++i)
    {
      if(!mono_colored)
	{
	  draw_quadrilateral(list[i],c[list[i].id%count],scale);
	}
      else
	{
	  draw_quadrilateral(list[i],CV_COLOR_WHITE,scale);
	}
    }   
}
void cVisualizer::draw_quadrilateral(quadrilateral q,cv::Scalar color, int scale)
{
  cv::Point2i start,end;
  start.x = q.c[0].x;
  start.y = q.c[0].y;
  end.x = q.center.x;
  end.y = q.center.y;
  cv::line(_scratch,start,end,color,scale);
  std::stringstream id_to_text;
  id_to_text<<q.id;
  cv::putText(_scratch,id_to_text.str(),end,cv::FONT_HERSHEY_SIMPLEX,0.5,CV_COLOR_WHITE);
  for(int i = 1; i <=4 ; ++i)
    {
      end.x = q.c[i%4].x;
      end.y = q.c[i%4].y;
      cv::line(_scratch,start,end,color,scale);
      start = end;
    }
  for(int i = 0; i <4 ; ++i)
    {
      start.x = q.c[i].x;
      start.y = q.c[i].y;
      if(q.c[i].is_accurate)
	{
	  cv::circle(_scratch,start,5,CV_COLOR_GREEN);
	}
      else
	{
	  cv::circle(_scratch,start,5,CV_COLOR_RED);
	}
    }
 }

void cVisualizer::draw_edgepixel(edgePixel p, cv::Scalar color, int scale )
{
  cv::Point2i start,end;
  float nx,ny;
  normalize_gx_gy(p.gx,p.gy,nx,ny);
  start.x = p.x;
  start.y = p.y;
  end.x = start.x + SCALE*nx;
  end.y = start.y + SCALE*ny;
  cv::line(_scratch,start,end,CV_COLOR_WHITE);
  cv::circle(_scratch,start,scale,color);
}

void cVisualizer::draw_edgepixel_vector(EdgePixelVector list,cv::Scalar color,int scale)
{
   for(EdgePixelVector::const_iterator it = list.begin(); it != list.end(); ++it) 
     {
       draw_edgepixel(*(it),color,scale);
     }
}

void cVisualizer::draw_connecting_linesegment(edgePixel start, edgePixel end,  cv::Scalar color, int scale)
{
  lineSegment l;
  l.start = start;
  l.end = end;
  draw_linesegment(l,color,scale);
}

void cVisualizer::draw_linesegment(lineSegment line, cv::Scalar color, int scale)
{
  cv::Point2i start,end;
  start.x = line.start.x;
  start.y = line.start.y;
  end.x = line.end.x;
  end.y = line.end.y;
  cv::line(_scratch,start,end,color,scale);
  draw_edgepixel(line.start,CV_COLOR_BLUE,2);
  draw_edgepixel(line.end,CV_COLOR_RED,2);
}

void cVisualizer::draw_arrow(lineSegment line, cv::Scalar color, int scale) 
{
    cv::Point2i start,end;
    start.x = line.start.x;
    start.y = line.start.y;
    end.x = line.end.x;
    end.y = line.end.y;
    cv::line(_scratch,start,end,color,scale);
    
    start.x = line.end.x;
    start.y = line.end.y;
    end.x = start.x + ( 8.0f* ( -line.n_xdiff + line.n_ydiff ) );
    end.y = start.y + ( 8.0f* ( -line.n_ydiff - line.n_xdiff ) );
    cv::line(_scratch,start,end,color,scale);
    
    end.x = start.x + ( 8.0f* ( -line.n_xdiff - line.n_ydiff ) );
    end.y = start.y + ( 8.0f* ( -line.n_ydiff + line.n_xdiff ) );
    cv::line(_scratch,start,end,color,scale);
}

void cVisualizer::draw_linesegment_vector_arrows(LineSegmentVector list,cv::Scalar color, int scale)
{
    for(LineSegmentVector::const_iterator it = list.begin(); it != list.end();++it)
    {
        draw_arrow(*(it),color,scale);
    }
}

void cVisualizer::draw_linesegment_vector(LineSegmentVector list,cv::Scalar color, int scale)
{
 for(LineSegmentVector::const_iterator it = list.begin(); it != list.end();++it)
   {
     draw_linesegment(*(it),color,scale);
     //show_img();
   }
}

#ifdef MOBILE_VISUALIZER
//no highgui stuff
cVisualizer::cVisualizer(const std::string name)
{
    _window_name = name;
}
void cVisualizer::set_img(const cv::Mat & in)
{
    _scratch = in;
}
void cVisualizer::show_img(bool wait)
{
}

void cVisualizer::clear_img()
{
}
#else
cVisualizer::cVisualizer(const std::string name)
{
  _window_name = name;
  cv::namedWindow(_window_name, CV_WINDOW_AUTOSIZE);
}

void cVisualizer::set_img(const cv::Mat & in)
{
  in.copyTo(_img);
  _img.copyTo(_scratch);
}

void cVisualizer::show_img(bool wait)
{
  cv::imshow(_window_name,_scratch);
  if(wait)
    {
      int key = cv::waitKey ();
      if (key == 'q' || key == 'Q')
	exit(0);
    }
  else
    {
      cv::waitKey(1);
    }
}

void cVisualizer::clear_img()
{
  _img.copyTo(_scratch);
}
#endif
