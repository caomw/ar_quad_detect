#include "cLineSegmentClip.h"

bool cLineSegmentClip::CohenSutherlandLineClip(edgePixel &start, edgePixel &end)
{
  int x0 = start.x;
  int y0 = start.y;
  int x1 = end.x;
  int y1 = end.y;

  // compute outcodes for P0, P1, and whatever point lies outside the clip rectangle
  int outcode0 = ComputeOutCode(x0, y0);
  int outcode1 = ComputeOutCode(x1, y1);
  bool accept = false;
 
  while (true) 
    {
      if (!(outcode0 | outcode1)) 
	{
	  // Bitwise OR is 0. Trivially accept and get out of loop
	  accept = true;
	  break;
	} 
      else if (outcode0 & outcode1) 
	{ // Bitwise AND is not 0. Trivially reject and get out of loop
	  break;
	} 
      else 
	{
	  // failed both tests, so calculate the line segment to clip
	  // from an outside point to an intersection with clip edge
	  double x, y;
 
	  // At least one endpoint is outside the clip rectangle; pick it.
	  int outcodeOut = outcode0? outcode0 : outcode1;
 
	  // Now find the intersection point;
	  // use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)
	  if (outcodeOut & TOP) 
	    {          
	      // point is above the clip rectangle
	      x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
	      y = ymax;
	    } 
	  else if (outcodeOut & BOTTOM) 
	    { // point is below the clip rectangle
	      x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
	      y = ymin;
	    } 
	  else if (outcodeOut & RIGHT) 
	    {  // point is to the right of clip rectangle
	      y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
	      x = xmax;
	    } 
	  else if (outcodeOut & LEFT)
	    {   // point is to the left of clip rectangle
	      y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
	      x = xmin;
	    }
	  // Now we move outside point to intersection point to clip
	  // and get ready for next pass.
	  if (outcodeOut == outcode0) 
	    {
	      x0 = x;
	      y0 = y;
	      outcode0 = ComputeOutCode(x0, y0);
	    } 
	  else 
	    {
	      x1 = x;
	      y1 = y;
	      outcode1 = ComputeOutCode(x1, y1);
	    }
	}
    }
  if (accept) 
    {
      end.x = x1;
      end.y = y1;
    }
  return accept;
}

int cLineSegmentClip::ComputeOutCode(float x, float y)
{
  
  int code = INSIDE;          // initialised as being inside of clip window
 
  if (x < xmin)           // to the left of clip window
    code |= LEFT;
  else if (x > xmax)      // to the right of clip window
    code |= RIGHT;
  if (y < ymin)           // below the clip window
    code |= BOTTOM;
  else if (y > ymax)      // above the clip window
    code |= TOP;
 
  return code;
}

