#include <algorithm>
#include "cQuadDetector.h"


void cQuadDetector::init(const int height, const int width)
{
  h_regions =  height/REGION_SIZE +1;
  w_regions =  width/REGION_SIZE +1;
  h_max = height - BORDERPIXELS;
  w_max = width - BORDERPIXELS;

  EdgePixelList = new EdgePixelVector* [h_regions];
  for(int i = 0; i< h_regions ; ++i)
    {
      EdgePixelList[i] = new EdgePixelVector[w_regions];
      for(int j = 0; j < w_regions; ++j)
	{
	  EdgePixelList[i][j].reserve(EDGEPIXEL_RESERVE_SIZE);
	}
    }

  LineList.reserve(LINESEGMENT_RESERVE_SIZE);
  
  max_support_edgepixels = new EdgePixelVector();
  cur_support_edgepixels = new EdgePixelVector();
  
  max_support_edgepixels->reserve(EDGEPIXEL_ON_LINE_RESERVE_SIZE);
  cur_support_edgepixels->reserve(EDGEPIXEL_ON_LINE_RESERVE_SIZE);

  candidates.reserve(CANDIDATE_SEGMENT_RESERVE_SIZE);
  quad_line_segments.reserve(QUAD_SEGMENT_RESERVE_SIZE);
  QuadList.reserve(QUAD_RESERVE_SIZE);

  //NOTE: Setting conservative boundary. Ideally it just needs to be (BORDERPIXELS,BORDERPIXELS,w_max,h_max);
  clipper.SetBoundary(2*BORDERPIXELS,2*BORDERPIXELS,w_max-BORDERPIXELS,h_max-BORDERPIXELS);
  
  fast_srand_seed(23123);
#ifdef __DEBUG_INFO__
  _debug = new cVisualizer("Debug Info");
#endif

}

void cQuadDetector::release()
{
  delete max_support_edgepixels;
  delete cur_support_edgepixels;
  for(int i = 0; i< h_regions ; ++i)
    {
      delete [] EdgePixelList[i];
    }
  delete [] EdgePixelList;

#ifdef __DEBUG_INFO__
  delete _debug;
#endif

}

#ifdef __DEBUG_INFO__
void cQuadDetector::set_debug_img(const cv::Mat & img)
{
  _debug->set_img(img);
}
#endif

bool cQuadDetector::are_lines_compatible_for_quad(lineSegment & line_one, lineSegment & line_two)
{
   if(are_lines_compatible(line_one,line_two))
    {
      return false;
    }

  int distance = distance_squared(line_one.end,line_two.start);
  if(distance > LINE_SEP_FOR_QUAD_DISTANCE_THRESHOLD)
    {
      return false;
    }
  float projection =  line_one.n_xdiff*line_two.n_ydiff - line_one.n_ydiff*line_two.n_xdiff;
  if((projection) < ACCEPTABLE_ANGLE_PROJECTION)
    {
      return false;
    }
  return true;
}

void cQuadDetector::find_all_quad(LineSegmentVector & lines, QuadrilateralVector & quads)
{

  if(lines.size() >0 )
    {
      do
	{
	  lineSegment segment = lines[0];
	  lines[0] = lines[lines.size() -1];
	  lines.resize(lines.size() -1);

	  int length = 1;

	  quad_line_segments.clear();
	  find_quad(segment,true,lines,quad_line_segments,length);
	  quad_line_segments.push_back(segment);

	  if(length < 4)
	    {
	      find_quad(segment,false,lines,quad_line_segments,length);	      
	    }

	  if(length > 2)
	    {
	      quadrilateral q;
	      find_intersection(quad_line_segments[0],quad_line_segments[1],q.c[0]);
	      find_intersection(quad_line_segments[1],quad_line_segments[2],q.c[1]);
	      q.c[0].is_accurate = true;
	      q.c[1].is_accurate = true;
	      if(length == 4)
		{		 
		  find_intersection(quad_line_segments[2],quad_line_segments[3],q.c[2]);
		  find_intersection(quad_line_segments[3],quad_line_segments[0],q.c[3]);
		  q.c[2].is_accurate = true;
		  q.c[3].is_accurate = true;
		}
	      else
		{
		  q.c[2].x = (float)quad_line_segments[2].end.x;
		  q.c[2].y = (float)quad_line_segments[2].end.y;
		  q.c[3].x = (float)quad_line_segments[0].start.x;
		  q.c[3].y = (float)quad_line_segments[0].start.y;
		  q.c[2].is_accurate = false;
		  q.c[3].is_accurate = false;
		}
	      if(is_quad_convex(q))
		{
		  q.center.x = (q.c[0].x + q.c[1].x +q.c[2].x + q.c[3].x)/4.0f;
		  q.center.y = (q.c[0].y + q.c[1].y +q.c[2].y + q.c[3].y)/4.0f;
		  quads.push_back(q);
		}
	    }
	}while(lines.size());
    }
}

void cQuadDetector::find_quad(lineSegment &line_one, bool from_start,LineSegmentVector &lines, LineSegmentVector &quad, int &length)
{
  for(int i = 0; i < lines.size(); ++i)
    {
      bool compatible;
      lineSegment segment = lines[i];
      if(from_start)
	{
	  compatible = are_lines_compatible_for_quad(segment,line_one);
	}
      else
	{
	  compatible = are_lines_compatible_for_quad(line_one,segment);
	}
      if(compatible)
	{
	  length++;
	  lines[i] = lines[lines.size()-1];
	  lines.resize(lines.size()-1);

	  if(length > 3)
	    {
	      quad.push_back(segment);
	      return;
	    }
	  if(!from_start)
	    {
	      quad.push_back(segment);
	    }
	  find_quad(segment,from_start,lines,quad,length);
	  if(from_start)
	    {
	      quad.push_back(segment);
	    }
	  return;
	}
    }
}


void cQuadDetector::extend_line_segments(LineSegmentVector & lines)
{
  const int line_count = lines.size();
  for(int i = 0 ; i < line_count ; ++i)
    {
      lineSegment &line = lines[i];
     
      const int dx = line.n_xdiff * LINE_SEGMENT_EXTENSION_SCALE;
      const int dy = line.n_ydiff * LINE_SEGMENT_EXTENSION_SCALE;

      edgePixel extended_end;
      extended_end.x = line.end.x + dx;
      extended_end.y = line.end.y + dy;

      //NOTE: Because the clipping region can be smaller than the region inside 
      //BORDERPIXELS there may be lines outside the clipping region
      if(clipper.CohenSutherlandLineClip(line.end,extended_end))
	{		
	  extended_end = extend_from_to(line.end,extended_end);
	  line.end.x = extended_end.x;
	  line.end.y = extended_end.y;
	}

      edgePixel extended_start;
      extended_start.x = line.start.x - dx;
      extended_start.y = line.start.y - dy;

      if(clipper.CohenSutherlandLineClip(line.start,extended_start))
	{
      	  extended_start = extend_from_to(line.start,extended_start);
	  line.start.x = extended_start.x;
	  line.start.y = extended_start.y;
	}
    }
}


/*TODO: change to account for change in line direction */
edgePixel cQuadDetector::extend_from_to(edgePixel start, edgePixel end)
{
  signed char ix;
  signed char iy;
  unsigned char * pixel_ptr;
  int total = 0;
  int acceptable = 0;
  int x1 = start.x;
  int x2 = end.x;
  int y1 = start.y;
  int y2 = end.y;

  // if x1 == x2 or y1 == y2, then it does not matter what we set here
  int delta_x = (x2 > x1?(ix = 1, x2 - x1):(ix = -1, x1 - x2)) << 1;
  int delta_y = (y2 > y1?(iy = 1, y2 - y1):(iy = -1, y1 - y2)) << 1;

  if (delta_x >= delta_y)
    {
      // error may go below zero
      int error = delta_y - (delta_x >> 1);
 
      while (x1 != x2)
        {
	  if (error >= 0)
            {
	      if (error || (ix > 0))
                {
		  y1 += iy;
		  error -= delta_x;
                }
            }
 
	  x1 += ix;
	  error += delta_y;
	  /*
	  switch(is_edge_kernel_acceptable_y(x1,y1))
	    {
	    case -1:
	      --y1;
	      break;
	    case 1:
	      ++y1;
	      break;
	    case 0:
	      break;
	    default:
	      end.x = x1- ix;
	      end.y = y1;
	      return end;
	    }
#ifdef __DEBUG_INFO__
	  TRACE("%d,%d",x1,y1);
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +2) = 255;
#endif
	  //*/
	  //*
 	  if(!is_edge_kernel_acceptable(x1,y1))
 	    {
 	      //TODO: update gx,gy
 	      end.x = x1- ix;
 	      end.y = y1;
 	      return end;
 	    }
	  //*/
        }
    }
  else
    {
      // error may go below zero
      int error = delta_x - (delta_y >> 1);
 
      while (y1 != y2)
        {
	  if (error >= 0)
            {
	      if (error || (iy > 0))
                {
		  x1 += ix;
		  error -= delta_y;
                }
            }
 
	  y1 += iy;
	  error += delta_x;

	  /*
	  switch(is_edge_kernel_acceptable_x(x1,y1))
	    {
	    case -1:
	      --x1;
	      break;
	    case 1:
	      ++x1;
	      break;
	    case 0:
	      break;
	    default:
	      end.x = x1;
	      end.y = y1 -iy;
	      return end;
	    }
#ifdef __DEBUG_INFO__
	  TRACE("%d,%d",x1,y1);
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +2) = 255;
#endif
	  //*/
	  //*
	  if(!is_edge_kernel_acceptable(x1,y1))
 	    {
 	      //TODO: update gx,gy
 	      end.x = x1;
 	      end.y = y1 -iy;
 	      return end;
 	    }
	  //*/
        }
    }

  return end;
}


int cQuadDetector::is_edge_kernel_acceptable_x(const int x, const int y)
{

  unsigned char * pixel_ptr = img_ptr +(y)*img_step_size + x;
  
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD)     
    {
      return 0;
    }

  pixel_ptr =  img_ptr + (y)*img_step_size + (x-1);
  int x_edge,y_edge; 
  x_edge = calculate_edge_kernel_x(pixel_ptr,img_step_size);
  y_edge = calculate_edge_kernel_y(pixel_ptr);
  const int minus_one = x_edge > y_edge ? x_edge: y_edge;

  pixel_ptr = img_ptr +(y)*img_step_size + (x+1);  
  x_edge = calculate_edge_kernel_x(pixel_ptr,img_step_size);
  y_edge = calculate_edge_kernel_y(pixel_ptr);
  const int plus_one = x_edge > y_edge ? x_edge: y_edge;
  
  int inc,max_kernel;
  (plus_one > minus_one)? inc = 1,max_kernel = plus_one:inc = -1, max_kernel = minus_one;
  if(max_kernel > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return inc;
    }
  
  return -100;
}


int cQuadDetector::is_edge_kernel_acceptable_y(const int x, const int y)
{

  unsigned char * pixel_ptr = img_ptr +(y)*img_step_size + x;
  
  if(calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return 0;
    }
  
  pixel_ptr =  img_ptr + (y-1)*img_step_size + (x);
  int x_edge,y_edge; 
  x_edge = calculate_edge_kernel_x(pixel_ptr,img_step_size);
  y_edge = calculate_edge_kernel_y(pixel_ptr);
  const int minus_one = x_edge > y_edge ? x_edge: y_edge;

  pixel_ptr = img_ptr +(y+1)*img_step_size + (x+1);
  x_edge = calculate_edge_kernel_x(pixel_ptr,img_step_size);
  y_edge = calculate_edge_kernel_y(pixel_ptr);
  const int plus_one = x_edge > y_edge ? x_edge: y_edge;

  int inc,max_kernel;
  (plus_one > minus_one)? inc = 1,max_kernel = plus_one:inc = -1, max_kernel = minus_one;
  if(max_kernel > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return inc;
    }

  return -100;
}



bool cQuadDetector::are_lines_connected(edgePixel start, edgePixel end)
{

  if(distance_squared(start,end) <=DISTANCE_MERGE_THRESHOLD)
    {
      return true;
    }

  signed char ix;
  signed char iy;
  unsigned char * pixel_ptr;
  int total = 0;
  int acceptable = 0;
  int x1 = start.x;
  int x2 = end.x;
  int y1 = start.y;
  int y2 = end.y;

  // if x1 == x2 or y1 == y2, then it does not matter what we set here
  int delta_x = (x2 > x1?(ix = 1, x2 - x1):(ix = -1, x1 - x2)) << 1;
  int delta_y = (y2 > y1?(iy = 1, y2 - y1):(iy = -1, y1 - y2)) << 1;

  if (delta_x >= delta_y)
    {
      // error may go below zero
      int error = delta_y - (delta_x >> 1);
 
      while (x1 != x2)
        {
	  if (error >= 0)
            {
	      if (error || (ix > 0))
                {
		  y1 += iy;
		  error -= delta_x;
                }
            }
 
	  x1 += ix;
	  error += delta_y;

	  if(is_edge_kernel_acceptable(x1,y1))
	    {
	      ++acceptable;
	    }
	  ++total;
	  /*
#ifdef __DEBUG_INFO__
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1) = 255;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +2) = 0;
#endif
	  */
        }
    }
  else
    {
      // error may go below zero
      int error = delta_x - (delta_y >> 1);
 
      while (y1 != y2)
        {
	  if (error >= 0)
            {
	      if (error || (iy > 0))
                {
		  x1 += ix;
		  error -= delta_y;
                }
            }
 
	  y1 += iy;
	  error += delta_x;

	  if(is_edge_kernel_acceptable(x1,y1))
	    {
	      ++acceptable;
	    }
	  ++total;
	  /*
#ifdef __DEBUG_INFO__
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1) = 255;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +1) = 0;
	  *(_debug->_img.data + y1*_debug->_img.step + 3*x1 +2) = 0;
#endif
	  */
        }
    }
  //TRACE("%d : %d",acceptable,total);
  return 4*acceptable >= 3*total;
  //return true;
}

bool cQuadDetector::is_edge_kernel_acceptable(const int x, const int y)
{
  unsigned char * pixel_ptr =  img_ptr + (y-1)*img_step_size + (x-1);
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD ||
     calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD )
    {
      return true;
    }

  pixel_ptr = img_ptr +(y)*img_step_size + x;
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD ||
     calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return true;
    }

  pixel_ptr = img_ptr +(y+1)*img_step_size + (x+1);
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD ||
     calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return true;
    }

  pixel_ptr = img_ptr +(y-1)*img_step_size + x+1;
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD ||
     calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return true;
    }

  pixel_ptr = img_ptr +(y+1)*img_step_size + (x-1);
  if(calculate_edge_kernel_x(pixel_ptr,img_step_size) >  EDGEPIXEL_NEAR_THRESHOLD ||
     calculate_edge_kernel_y(pixel_ptr) > EDGEPIXEL_NEAR_THRESHOLD)
    {
      return true;
    }
  
  return false;
}



int cQuadDetector::calculate_edge_kernel_x(unsigned char * pixel_ptr,const int step_size)
{
  int value;
  pixel_ptr -= 2*step_size;
  value      = -2*(*(pixel_ptr));
  pixel_ptr +=       step_size;
  value     -=  4*(*(pixel_ptr));
  pixel_ptr +=     2*step_size;
  value     +=  4*(*(pixel_ptr));
  pixel_ptr +=       step_size;
  value     +=  2*(*(pixel_ptr));
  //TRACE("Value %d :",value);
  return abs(value);
}

int cQuadDetector::calculate_edge_kernel_y(unsigned char * pixel_ptr)
{
  int value;
  pixel_ptr -= 2;
  value      = -2*(*(pixel_ptr));
  ++pixel_ptr;
  value     -=  4*(*(pixel_ptr));
  pixel_ptr+=2;
  value     +=  4*(*(pixel_ptr));
  ++pixel_ptr;
  value     +=  2*(*(pixel_ptr));
  //TRACE("Value %d :",value);
  return abs(value);
}

inline bool cQuadDetector::are_lines_compatible(lineSegment line_one, lineSegment line_two)
{
  float projection = line_one.n_xdiff*line_two.n_xdiff + line_one.n_ydiff*line_two.n_ydiff;
  return projection >SLOPE_MERGE_THRESHOLD;
}

bool cQuadDetector::is_connecting_line_compatible(lineSegment line_one, lineSegment line_two)
{
  float n_conn_x, n_conn_y;
  int conn_x = line_two.start.x - line_one.end.x;
  int conn_y = line_two.start.y - line_one.end.y;
  normalize_gx_gy(conn_x,conn_y,n_conn_x,n_conn_y);
  return (n_conn_x*line_one.n_xdiff + n_conn_y*line_one.n_ydiff) > SLOPE_MERGE_THRESHOLD;
}

void cQuadDetector::merge_line_segments(LineSegmentVector &lines)
{
  lineSegment line_one,line_two;
  lineSegment line_first;
  const int line_count = lines.size();
  for( int i = 0; i < line_count; ++i)
    {
      lines[i].is_merged = false;
    }
  for( int i = 0; i < line_count; ++i)
    {
      line_one = lines[i];
      if(line_one.is_merged)
	continue;

      candidates.clear();
      for( int j = i+1; j < line_count;++j)
	{
	  line_two = lines[j];
	  if(line_two.is_merged)
	    continue;
	  if(are_lines_compatible(line_one,line_two))
	    {
	      int d_1_e_2_s = distance_squared(line_one.end,line_two.start);
	      int d_2_e_1_s = distance_squared(line_two.end,line_one.start);
	      candidateSegment c;
	      if(d_2_e_1_s < d_1_e_2_s)
		{
		  if(is_connecting_line_compatible(line_two,line_one))
		    {
		      c.index = j;
		      c.distance = d_2_e_1_s;
		      c.is_flipped = true;
		      candidates.push_back(c);
		    }
		}
	      else
		{
		   if(is_connecting_line_compatible(line_one,line_two))
		    {
		      c.index = j;
		      c.distance = d_1_e_2_s;
		      c.is_flipped = false;
		      candidates.push_back(c);
		    }
		}
	    }
	}
      
      if(candidates.size() == 0)
	continue;

      std::sort(candidates.begin(),candidates.end(),compare_candidate_segments);
      const int candidate_count = candidates.size();
      for(int j = 0; j < candidate_count;++j)
	{
	  candidateSegment c = candidates[j];
	  const lineSegment other = lines[c.index];
	  bool lines_connected;
	  if(c.is_flipped)
	    {
	      lines_connected = are_lines_connected(line_one.start, other.end);
	    }
	  else
	    {
	      lines_connected = are_lines_connected(line_one.end,other.start);
	    }
	  if(lines_connected)
	    {	     
	      if(c.is_flipped)
		{
		  line_one.start = other.start;
		}
	      else
		{
		  line_one.end = other.end;
		}
	      lines[c.index].is_merged = true;
	    }
	}

      /*TODO: is this necessary ?*/     
      const int xdiff = line_one.end.x - line_one.start.x;
      const int ydiff = line_one.end.y - line_one.start.y;
      normalize_gx_gy(xdiff,ydiff, line_one.n_xdiff, line_one.n_ydiff);
      
      lines[i] = line_one;
    }

  lines.erase(std::remove_if(lines.begin(),lines.end(),is_line_segment_merged),lines.end());

}

void cQuadDetector::convert_edge_pixels_to_line_segments_iterative(EdgePixelVector &edgepixels,LineSegmentVector &lines)
{
  do
    {
      int start_index, end_index,iteration;
      edgePixel start,end;
      int max_xdiff,max_ydiff;
      const int num_of_edgepixels = edgepixels.size();
      const int  num_of_edgepixels2 = num_of_edgepixels>>1;
      max_support_edgepixels->clear();
      itr_number.reset(num_of_edgepixels);
      bool itr_flag;
      for(int i =0; i < RANSAC_LINE_TRIALS &&
      	  	  max_support_edgepixels->size() < num_of_edgepixels2; ++i)
	{
	  iteration = 0;
	  do
	    {
	      //get_random_indices(num_of_edgepixels,start_index,end_index);
	      itr_flag=itr_number.get_iterative_indices(start_index,end_index);
	      ++iteration;
	    }while(itr_flag && !angle_compatible(edgepixels[start_index].angle,edgepixels[end_index].angle)
	    		&& iteration < RANSAC_MAX_ITER);
	
	  if(iteration > RANSAC_MAX_ITER)
	    break;

	  cur_support_edgepixels->clear();

	  start = edgepixels[start_index];
	  end   = edgepixels[end_index];

	  const int xdiff = end.x - start.x;
	  const int ydiff = end.y - start.y;
	  const float length = sqrt(xdiff*xdiff + ydiff*ydiff);
	  for(int i = 0; i < num_of_edgepixels; ++i)
	    {
	      edgePixel cur = edgepixels[i];
	      if(angle_compatible(start.angle,cur.angle)&&is_on_line(xdiff,ydiff,length,start,cur))
		{
		  cur_support_edgepixels->push_back(cur);
		}
	    }
	  if(cur_support_edgepixels->size() > max_support_edgepixels->size())
	    {
	      max_xdiff = xdiff;
	      max_ydiff = ydiff;
	      std::swap(max_support_edgepixels,cur_support_edgepixels);
	    }	  
	}
	
      if(max_support_edgepixels->size() < MIN_EDGEPIXEL_ON_LINE)
	break;

      find_min_max_index(*max_support_edgepixels,max_xdiff,max_ydiff,start_index,end_index);

      lineSegment line;

      start = max_support_edgepixels->at(start_index);
      end   = max_support_edgepixels->at(end_index);

      /*
#ifdef __DEBUG_INFO__
      lineSegment tmp;
      tmp.start = start;
      tmp.end = end;
      _debug->draw_linesegment(tmp,CV_COLOR_RED,5);
      //_debug->draw_edgepixel(start,CV_COLOR_RED,5);
      //_debug->draw_edgepixel(end,CV_COLOR_RED,5);
#endif
      //find_best_line_segment(*max_support_edgepixels,start,end);
#ifdef __DEBUG_INFO__
      tmp.start = start;
      tmp.end = end;
      _debug->draw_linesegment(tmp,CV_COLOR_GREEN);
#endif
      //*/
      float nx,ny,n_xdiff,n_ydiff;
      const int xdiff = end.x - start.x;
      const int ydiff = end.y - start.y;
      normalize_gx_gy(xdiff,ydiff,n_xdiff,n_ydiff);
      normalize_gx_gy(start.gx,start.gy,nx,ny);
      const float dot_product = -ny*n_xdiff + nx*n_ydiff;
      if(dot_product > 0.0f)
	{
	  line.start = end;
	  line.end = start;
	  line.n_xdiff = - n_xdiff;
	  line.n_ydiff = - n_ydiff;
	}
      else
	{
	  line.start = start;
	  line.end = end;
	  line.n_xdiff = n_xdiff;
	  line.n_ydiff = n_ydiff;
	}

      lines.push_back(line);

      ItemInOtherVectorPred trueIfItemInOtherVecPred(max_support_edgepixels);
      EdgePixelVector::iterator erase_begin = std::remove_if( edgepixels.begin(), edgepixels.end(), trueIfItemInOtherVecPred);
      edgepixels.erase(erase_begin, edgepixels.end());
    }while(edgepixels.size() >= MIN_EDGEPIXEL_ON_LINE);
  /*
#ifdef __DEBUG_INFO__
  _debug->show_img();
#endif
  //*/
}

void cQuadDetector::convert_edge_pixels_to_line_segments_random(EdgePixelVector &edgepixels,LineSegmentVector &lines)
{
  do
    {
      int start_index, end_index,iteration;
      edgePixel start,end;
      int max_xdiff,max_ydiff;
      const int num_of_edgepixels = edgepixels.size();
      max_support_edgepixels->clear();
      for(int i =0; i < RANSAC_LINE_TRIALS; ++i)
	{
	  iteration = 0;
	  do
	    {
	      get_random_indices(num_of_edgepixels,start_index,end_index);
	      start = edgepixels[start_index];
	      end   = edgepixels[end_index];
	      ++iteration;
	    }while(!angle_compatible(start.angle,end.angle) && iteration < RANSAC_MAX_ITER);

	  if(iteration > RANSAC_MAX_ITER)
	    break;

	  cur_support_edgepixels->clear();
	  const int xdiff = end.x - start.x;
	  const int ydiff = end.y - start.y;
	  const double length = sqrt(xdiff*xdiff + ydiff*ydiff);
	  for(int i = 0; i < num_of_edgepixels; ++i)
	    {
	      edgePixel cur = edgepixels[i];
	      if(angle_compatible(start.angle,cur.angle)&&is_on_line(xdiff,ydiff,length,start,cur))
		{
		  cur_support_edgepixels->push_back(cur);
		}
	    }
	  if(cur_support_edgepixels->size() > max_support_edgepixels->size())
	    {
	      max_xdiff = xdiff;
	      max_ydiff = ydiff;
	      std::swap(max_support_edgepixels,cur_support_edgepixels);
	    }
	}

      if(max_support_edgepixels->size() < MIN_EDGEPIXEL_ON_LINE)
	break;

      find_min_max_index(*max_support_edgepixels,max_xdiff,max_ydiff,start_index,end_index);
      //TODO: Re-estimate line using the support_edge_pixels to improve accuracy
      lineSegment line;

      start = max_support_edgepixels->at(start_index);
      end   = max_support_edgepixels->at(end_index);

      /*
#ifdef __DEBUG_INFO__
	
#endif
	//find_best_line_segment(*max_support_edgepixels,start,end);
#ifdef __DEBUG_INFO__
	
#endif
	//*/
      float nx,ny,n_xdiff,n_ydiff;
      const int xdiff = end.x - start.x;
      const int ydiff = end.y - start.y;
      normalize_gx_gy(xdiff,ydiff,n_xdiff,n_ydiff);
      normalize_gx_gy(start.gx,start.gy,nx,ny);
      const float dot_product = -ny*n_xdiff + nx*n_ydiff;
      if(dot_product > 0.0f)
	{
	  line.start = end;
	  line.end = start;
	  line.n_xdiff = - n_xdiff;
	  line.n_ydiff = - n_ydiff;
	}
      else
	{
	  line.start = start;
	  line.end = end;
	  line.n_xdiff = n_xdiff;
	  line.n_ydiff = n_ydiff;
	}

      lines.push_back(line);

      ItemInOtherVectorPred trueIfItemInOtherVecPred(max_support_edgepixels);
      EdgePixelVector::iterator erase_begin = std::remove_if( edgepixels.begin(), edgepixels.end(), trueIfItemInOtherVecPred);
      edgepixels.erase(erase_begin, edgepixels.end());
    }while(edgepixels.size() >= MIN_EDGEPIXEL_ON_LINE);
}

void cQuadDetector::find_best_line_segment(const EdgePixelVector pixels, edgePixel & start, edgePixel & end)
{
  //NOTE: DONOT use as least square for the line y=ax+b fails for vertical lines
  //NOTE: For non-vertical lines it not provide significant advantage
  //TODO: Optimization pending
  float sum_x  = 0.0f;
  float sum_y  = 0.0f;
  float sum_xy = 0.0f;
  float sum_x2 = 0.0f;
  const int size = pixels.size();
  for(int i = 0; i < size; ++i)
    {
      float x = (float) pixels[i].x;
      float y = (float) pixels[i].y;
      sum_x += x;
      sum_y += y;
      sum_xy += x*y;
      sum_x2 += x*x;
    }
  //float avg_x = sum_x/float(size);
  //float avg_y = sum_y/float(size);
  float den = size*sum_x2 - sum_x*sum_x;
  //float den = sum_x2 - size*avg_x*avg_x;
  float a,b;

  if(!size || !den)
    {
      a = 0;
      b = size? sum_y/(float)size: 0;
      TRACE("A: %f, B: %f",a,b);
    }
  else
    {
      a = (size*sum_xy - sum_x*sum_y)/den;
      b = (sum_y - sum_x*a)/(float)size;
      //a = (sum_xy - size*avg_x*avg_y)/den;
      //b = (avg_y*sum_x2-avg_x*sum_xy)/den;
      TRACE("REGULAR: A: %f, B: %f",a,b);
    }

#ifdef __DEBUG_INFO__
  for(int i = 0; i < size; ++i)
    {
      float x = (float) pixels[i].x;
      float y = (float) pixels[i].y;
      float err = a * x + b - y ;
      TRACE("(%d,%d) : %4.5f",pixels[i].x,pixels[i].y,err);
    }
#endif

  float x_new,y_new;
  x_new = (start.x + a*start.y - a*b)/(a*a +1);
  y_new = x_new*a+b;

  start.x = (int)x_new;
  start.y = (int)y_new;

  x_new = (end.x + a*end.y - a*b)/(a*a +1);
  y_new = x_new*a+b;

  end.x = (int)x_new;
  end.y = (int)y_new;
  //*/
}

//TODO: Is it better to return the start and end point instead of there index?
void cQuadDetector::find_min_max_index(const EdgePixelVector pixels,const int xdiff, const int ydiff,int &min_index, int &max_index)
{
  int value;
  int max_value,min_value;
  const int size = pixels.size();
  if(abs(xdiff)<abs(ydiff))
    {
      min_value = max_value = pixels[0].y;
      min_index = max_index = 0;
      for(int i = 1 ; i < size; ++i)
	{
	  value = pixels[i].y;
	  if(value > max_value)
	    {
	      max_index = i;
	      max_value = value;
	    }
	  else if(value < min_value)
	    {
	      min_index = i;
	      min_value = value;
	    }
	}
    }
  else
    {
      min_value = max_value = pixels[0].x;
      min_index = max_index = 0;
      for(int i = 1 ; i < size; ++i)
	{
	  value = pixels[i].x;
	  if(value > max_value)
	    {
	      max_index = i;
	      max_value = value;
	    }
	  else if(value < min_value)
	    {
	      min_index = i;
	      min_value = value;
	    }
	}
    }
} 

inline bool cQuadDetector::is_on_line(const int xdiff, const int ydiff, const float length,edgePixel start,edgePixel p)
{
  float distance = (xdiff*(start.y-p.y) -ydiff*(start.x - p.x));
  distance /= length;
  return fabs(distance) < ALLOWED_DISTANCE; 
}

inline bool cQuadDetector::angle_compatible(float a, float b)
{
  float diff = fabs(a - b);
  if(diff > M_PI)
    {
      diff = 2*M_PI - diff;
    }
  return diff < TOLERANCE_IN_RADIAN;
}

void cQuadDetector::get_random_indices(const int size,int &s,int &e)
{ 
  //s = rand()%size;
  //e = rand()%size;

  s = fast_srand()%size;
  e = fast_srand()%size;
  if (e == s)
  {
    ++e;
    if(e >= size)
      {
	e -=2;
      }
  }
}

void cQuadDetector::find_edgepixels(const cv::Mat& src)
{
  const int row_max = h_max;//src.rows - BORDERPIXELS;
  const int col_max = w_max;//src.cols - BORDERPIXELS;
  const int step_size = src.step;
  const int step_size_pixelskip = PIXELSKIP*src.step;
  /*
    NOTE: Interesting observation promoting following variables to the classes
    private member has distinct effect of increasing access time and hence 
    execution time
  */
  int value, value_prev, value_prev_prev;
  int r,c;
  unsigned char * pixel_ptr;
  unsigned char * curr_pixel_ptr;
  unsigned char * col_ptr;
  unsigned char * row_ptr;
  unsigned char * src_ptr;
  int gx,gy;
  int r_cell,c_cell;
 
  /*
    NOTE: This only works for value of BORDERPIXEL 2 as they equal to the prior number
    of cols needed for calculation. Else it needs to point to the pixel in the 2 cols 
    prior to the BORDERPIXEL.
  */
  src_ptr = src.data +(BORDERPIXELS)*src.step + (BORDERPIXELS - 2);
  for(r=BORDERPIXELS;r<row_max;r+=PIXELSKIP)
    {
      row_ptr = src_ptr;
      r_cell = r/REGION_SIZE;
      value_prev = 0;
      value_prev_prev = 0;

      for(c=BORDERPIXELS;c<col_max;++c)
	{
	  /*TODO: Assume that REGION_SIZE is in 2powerN and use bit shift instead*/
	  c_cell = c/REGION_SIZE;
	  /*TODO: divide the full equation by 2 and half the threshold*/
	  /*TODO: use bit shift operator instead of relying on compiler*/
	  pixel_ptr  =       row_ptr;
	  value      = -2*(*(pixel_ptr));
	  ++pixel_ptr;
	  row_ptr    =       pixel_ptr;
	  value     -=  4*(*(pixel_ptr));
	  ++pixel_ptr;
	  curr_pixel_ptr = pixel_ptr++;
	  value     +=  4*(*(pixel_ptr));
	  ++pixel_ptr;
	  value     +=  2*(*(pixel_ptr));
	  value      = abs(value);
	  
	  if(value > EDGEPIXEL_THRESHOLD)
	    {
	      if(value_prev > 0 && value_prev > value_prev_prev && value_prev > value)
		{
		  edgePixel ep;
		  ep.x = c;
		  ep.y = r;
		  compute_gradient_at_pixel(curr_pixel_ptr,step_size,ep.gx,ep.gy);
		  ep.angle = atan2(ep.gy,ep.gx);
		  EdgePixelList[r_cell][c_cell].push_back(ep);
		}
	    }
	  else
	    {
	      value = 0;
	    }
	  value_prev_prev = value_prev;
	  value_prev = value;
	}
      src_ptr+=step_size_pixelskip;
    }
  /*
    NOTE: This only works for value of BORDERPIXEL 2 as they equal to the prior number
    of rows needed for calculation. Else it needs to point to the pixel in the 2 rows 
    prior to the BORDERPIXEL.
  */
  src_ptr = src.data +(BORDERPIXELS - 2)*src.step + (BORDERPIXELS);
  for(c=BORDERPIXELS;c<col_max;c+=PIXELSKIP)
    {
      col_ptr = src_ptr;
      c_cell = c/REGION_SIZE;
      
      for(r=BORDERPIXELS;r<row_max;++r)
	{
	  r_cell = r/REGION_SIZE;

	  pixel_ptr  =       col_ptr;
	  value      = -2*(*(pixel_ptr));
	  pixel_ptr +=       step_size;
	  col_ptr    =       pixel_ptr;
	  value     -=  4*(*(pixel_ptr));
	  pixel_ptr +=       step_size;
	  curr_pixel_ptr =   pixel_ptr;
	  pixel_ptr +=       step_size;
	  value     +=  4*(*(pixel_ptr));
	  pixel_ptr +=       step_size;
	  value     +=  2*(*(pixel_ptr));
	  value = abs(value);
	  
	  if(value > EDGEPIXEL_THRESHOLD)
	    {
	      if(value_prev > 0 && value_prev > value_prev_prev && value_prev > value)
		{
		  edgePixel ep;
		  ep.x = c;
		  ep.y = r;
		  compute_gradient_at_pixel(curr_pixel_ptr,step_size,ep.gx,ep.gy);
		  ep.angle = atan2(ep.gy,ep.gx);
		  EdgePixelList[r_cell][c_cell].push_back(ep);
		}
	    }
	  else
	    {
	      value = 0;
	    }
	  value_prev_prev = value_prev;
	  value_prev = value;
	}
      src_ptr+=PIXELSKIP;
    }
}

void cQuadDetector::compute_gradient_at_pixel(unsigned char * data, const unsigned int width_step,float &gx,float &gy)
{
  /*
  ...................
  .	.     .	    .
  .  a	.  b  .	 c  .
  ...................
  .	.     .	    .
  .  d	.     .	 e  .
  ...................
  .	.     .	    .
  .  f  .  g  .	 h  .
  ...................
  */
  unsigned char * l_data = data-width_step-1;
  const unsigned char a = *(l_data++);
  const unsigned char b = *(l_data++);
  const unsigned char c = *(l_data);
  l_data = data-1;
  const unsigned char d = *(l_data++);
  ++l_data;
  const unsigned char e = *(l_data);
  l_data = data+width_step-1;
  const unsigned char f = *(l_data++);
  const unsigned char g = *(l_data++);
  const unsigned char h = *(l_data);
  /*TODO: check for overflow condition as subtracting unsigned char*/
  const int ah = a-h;
  const int cf = c-f;
  const int bg = b-g;
  const int de = d-e;
  /*TODO: check if bitshift is faster instead of multiply by 2*/
  /*TODO: verify x and y assignment*/
  gy = ah+cf+2*bg;
  gx = ah-cf+2*de;
}

#ifdef QUAD_VECTORIZE
void cQuadDetector::detect(const cv::Mat& src)
{
  TRACE("%s","Find edgePixel and Lines");
  quadVectorized.find_edgepixelsAndLineSegments(src,LineList);
  TRACE("%s","Merge lines");
  merge_line_segments(LineList);
  TRACE("%s","Extend lines");
  extend_line_segments(LineList);
  TRACE("%s","Clear QuadList");
  QuadList.clear();
  TRACE("%s","find Quad");
  find_all_quad(LineList,QuadList);
  TRACE("%s","Done");
}
#else
void cQuadDetector::detect(const cv::Mat& src)
{
  img_ptr = src.data;
  img_step_size = src.step;
  TRACE("%s","Clear EdgePixelList");
  for(int i = 0; i< h_regions ; ++i)
    {
      for(int j = 0; j < w_regions; ++j)
	{
	  EdgePixelList[i][j].clear();
	}
    }  
  TRACE("%s","Find edgePixel");
  find_edgepixels(src);
  TRACE("%s","Clear LineList");
  LineList.clear();
  TRACE("%s","Find lines");
  for(int i = 0; i< h_regions ; ++i)
    {
      for(int j = 0; j < w_regions; ++j)
	{
	  if(EdgePixelList[i][j].size() > MIN_EDGEPIXEL_ON_LINE )
	    {
	      convert_edge_pixels_to_line_segments_iterative(EdgePixelList[i][j],LineList);
	      //convert_edge_pixels_to_line_segments_random(EdgePixelList[i][j],LineList);
	    }
	}
    }
  TRACE("%s","Merge lines");
  //merge_line_segments(LineList);
  TRACE("%s","Extend lines");
  //extend_line_segments(LineList);
  TRACE("%s","Clear QuadList");
  QuadList.clear();
  TRACE("%s","find Quad");
  //making a copy of linesegment for display purpose in mobile mode  
#ifdef MOBILE_VISUALIZER
  LineListCopy = LineList; 
#endif
  find_all_quad(LineList,QuadList); 
  TRACE("%s","Done");
}
#endif
