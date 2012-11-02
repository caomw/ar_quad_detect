
#include <cmath>
//#include <alogrithm>
#include "commonDef.h"

template <class T> const T& max ( const T& a, const T& b ) {
  return (b<a)?a:b;  
}
template <class T> const T& min ( const T& a, const T& b ) {
  return (b>a)?a:b;  
}

//NOTE: Fast random number generator taken from
//http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/
static unsigned int g_rand_seed; 
 
//Used to seed the generator. 
void fast_srand_seed( int seed ) 
{ 
  g_rand_seed = seed; 
} 
  
//fastrand routine returns one integer, similar output value range as C lib. 
int fast_srand() 
{ 
  g_rand_seed = (214013*g_rand_seed+2531011); 
  return (g_rand_seed>>16)&0x7FFF; 
} 

//This code is purported to be from Quake3 and provides an approximation
//to the inverse square root of a number, 1/sqrtf(x).
inline float fast_inv_sqrt(float x) 
{
  float xhalf = 0.5f*x;
  long i = *(long*) &x;
  i = 0x5f3759df - (i>>1);    // original
  x = *(float*) &i;
  x = x*(1.5f-xhalf*x*x);        // Newton step
  //    x = x*(1.5f-xhalf*x*x);        // Newton step, repeating increases accuracy
  return x;
}

inline float inv_sqrt(int x)
{
  return 1.0f/(sqrt(x));
}

void normalize_gx_gy(int gx, int gy, float &nx, float &ny)
{
  /*TODO: if explicit normalization is necessary*/
  /*TODO: check if quake inverse sqrt is accurate for application*/
  int x2_y2 = gx*gx+gy*gy;
  float sqrt_inv = inv_sqrt(x2_y2);
  //Not significant speed advantage overall and inverses the gradient vector
  //float sqrt_inv = fast_inv_sqrt((float)x2_y2);
  nx=sqrt_inv*(float)gx;
  ny=sqrt_inv*(float)gy;
}

bool is_quad_convex(quadrilateral& quad)
{
  const float x1 = quad.c[0].x; const float y1 = quad.c[0].y;
  const float x2 = quad.c[2].x; const float y2 = quad.c[2].y;
  const float x3 = quad.c[1].x; const float y3 = quad.c[1].y;
  const float x4 = quad.c[3].x; const float y4 = quad.c[3].y;
  
  const float d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  if(d == 0)
    {
      return false;
    }
  const float pre  = (x1*y2 - y1*x2);
  const float post = (x3*y4 - y3*x4);
  const float x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d;
  const float y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d;

  if ( x < min(x1, x2) || x > max(x1, x2) || x < min(x3, x4) || x > max(x3, x4) ) 
    return false;
  if ( y < min(y1, y2) || y > max(y1, y2) || y < min(y3, y4) || y > max(y3, y4) ) 
    return false;

  return true;
}

void find_intersection(const lineSegment& line_one, const lineSegment& line_two, position & corner)
{
  const int x0 = line_one.start.x;
  const int y0 = line_one.start.y;
  const int x1 = line_one.end.x;
  const int y1 = line_one.end.y;

  const int x2 = line_two.start.x;
  const int y2 = line_two.start.y;
  const int x3 = line_two.end.x;
  const int y3 = line_two.end.y;
  
  float m1,m2;
  
  if ((x1-x0)!=0)
    m1 = (float)(y1-y0)/(float)(x1-x0);
  else
    m1 = (float)1e+10;   // close enough to infinity

  if ((x3-x2)!=0)
    m2 = (float)(y3-y2)/(float)(x3-x2);
  else
    m2 = (float)1e+10;   // close enough to infinity

  // compute constants

  const float a1 = m1;
  const float a2 = m2;

  const float b1 = -1;
  const float b2 = -1;

  const float c1 = (y0-m1*x0);
  const float c2 = (y2-m2*x2);

  // compute the inverse of the determinate

  const float det_inv = 1.0f/(a1*b2 - a2*b1);

  // use Kramers rule to compute xi and yi

  corner.x =((b1*c2 - b2*c1)*det_inv);
  corner.y =((a2*c1 - a1*c2)*det_inv);

}
