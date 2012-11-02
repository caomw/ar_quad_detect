#ifndef __CQUADDETECTOR_H__
#define __CQUADDETECTOR_H__

//#include <vector>
#include <opencv2/core/core.hpp>
#include "commonDef.h"
#include "cLineSegmentClip.h"


#ifdef __DEBUG_INFO__
#include "cVisualizer.h"  
#endif

class IterativeNumber{
	int i,j,pos,dir;
	int size;
public:
	IterativeNumber():dir(1){}
	void reset(const int _size){
		size = _size,pos=_size>>1;
		i = j = pos;
	}

	bool get_iterative_indices(int &s,int &e){
	  if(i>0) i--;
	  if(j<size-1) j++;

	  s = i;
	  e = j;

	  if(i<=0 && j>=size-1){
		  pos+=dir;
		  //change pos till end +ve
		  if(pos<size-1 && pos>0){
			  i=j=pos;
		  }else{ //change pos till end +ve
			  if(pos<=0){
				  return false;
			  }else{
				  pos=(size>>1);
				  dir=-1;
				  pos+=dir;
				  i=j=pos;
			  }
		  }
	  }
	  return true;
	}
};
class cQuadDetector
{
 public:
  void init(const int height, const int width);
  void release();
  void detect(const cv::Mat& input);

 public:
  EdgePixelVector **EdgePixelList;
  LineSegmentVector LineList;
  QuadrilateralVector QuadList;
    //making a copy of linesegment for display purpose in mobile mode  
#ifdef MOBILE_VISUALIZER
  LineSegmentVector LineListCopy;
#endif
    
  IterativeNumber itr_number;
  unsigned int h_regions;
  unsigned int w_regions;
  
 private:
  void find_edgepixels(const cv::Mat& input);
  void convert_edge_pixels_to_line_segments_iterative(EdgePixelVector &edge_pixels, LineSegmentVector &lines);
  void convert_edge_pixels_to_line_segments_random(EdgePixelVector &edge_pixels, LineSegmentVector &lines);
  void compute_gradient_at_pixel(unsigned char * data, const unsigned int width_step,float &gx, float &gy);
  void orient_start_end(edgePixel &start, edgePixel &end);
  void find_min_max_index(const EdgePixelVector pixels,const int xdiff, const int ydiff,int &min_index, int &max_index);
  void find_best_line_segment(const EdgePixelVector pixels, edgePixel & start, edgePixel & end);
  void get_random_indices(const int size, int &s, int &e);
  bool is_on_line(const int xdiff, const int ydiff, const float length,edgePixel start,edgePixel p);
  int calculate_edge_kernel_x(unsigned char * pixel_ptr,const int step_size);
  int calculate_edge_kernel_y(unsigned char * pixel_ptr);

  bool is_edge_kernel_acceptable(const int x, const int y);
  int is_edge_kernel_acceptable_x(const int x, const int y);
  int is_edge_kernel_acceptable_y(const int x, const int y);

  bool are_lines_connected(edgePixel start, edgePixel end);
  bool is_connecting_line_compatible(lineSegment line_one, lineSegment line_two);
  void merge_line_segments(LineSegmentVector &lines);
  edgePixel extend_from_to(edgePixel start, edgePixel end);
  void extend_line_segments(LineSegmentVector &lines);
  inline bool angle_compatible(float a, float b);
  inline bool are_lines_compatible(lineSegment line_one, lineSegment line_two);

  void find_all_quad(LineSegmentVector & lines,QuadrilateralVector & quads);
  void find_quad(lineSegment &line_one, bool from_start,LineSegmentVector &lines, LineSegmentVector &quad, int &length);
  bool are_lines_compatible_for_quad(lineSegment & line_one, lineSegment & line_two);

 private:
  EdgePixelVectorPtr max_support_edgepixels;
  EdgePixelVectorPtr cur_support_edgepixels;
  CandidateSegmentVector candidates;
  LineSegmentVector quad_line_segments;

  cLineSegmentClip clipper;

  unsigned char * img_ptr;
  int img_step_size;
  int h_max;
  int w_max;
  
#ifdef __DEBUG_INFO__
 private:
  cVisualizer * _debug;
 public:
  void set_debug_img(const cv::Mat & img);
#endif
  
};

#endif
