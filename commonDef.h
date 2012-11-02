#ifndef __COMMONDEF_H__
#define __COMMONDEF_H__

#include <vector>
#include <algorithm>

//#define __DEBUG_INFO__
#ifdef __DEBUG_INFO__

#include <stdio.h>
#define TRACE(fmt, ...) \
do {fprintf(stderr, "%s(): " fmt,__func__, __VA_ARGS__); fprintf(stderr,"\n");} while (0)
#else
#define TRACE(X,...)
#endif

#define PIXELSKIP 5
//NOTE: Do not set BORDERPIXELS less than 3 as
//as the kernel needs prior two pixels and prior one pixel for gradient calculation
#define BORDERPIXELS 5
#define LINE_SEGMENT_EXTENSION_SCALE 1000.0f

#define EDGEPIXEL_RESERVE_SIZE 50
#define LINESEGMENT_RESERVE_SIZE 100
#define CANDIDATE_SEGMENT_RESERVE_SIZE 50
#define EDGEPIXEL_ON_LINE_RESERVE_SIZE 20
#define QUAD_SEGMENT_RESERVE_SIZE 4
#define QUAD_RESERVE_SIZE 10
#define MIN_EDGEPIXEL_ON_LINE 4
#define RANSAC_MAX_ITER 100
#define RANSAC_LINE_TRIALS 25

#define EDGEPIXEL_THRESHOLD 250
#define EDGEPIXEL_NEAR_THRESHOLD 100
#define TOLERANCE_IN_RADIAN (5*0.0174532925)
#define ALLOWED_DISTANCE 2.5f
#define SLOPE_MERGE_THRESHOLD 0.98f
#define DISTANCE_MERGE_THRESHOLD 7*7
#define QUAD_CENTER_DISTANCE_THRESHOLD (100.0f*100.0f)
#define ACCEPTABLE_ANGLE_PROJECTION 0.6f
#define REGION_SIZE 64
#define LINE_SEP_FOR_QUAD_DISTANCE_THRESHOLD 15*15
#define QUAD_MISSING_FRAME_THRESHOLD 2
#define QUAD_FOUND_FRAME_THRESHOLD 2

typedef struct position_s
{
  float x;
  float y;
  bool is_accurate;
}position;

typedef struct quadrilateral_s
{
  position c[4];
  position center;
  unsigned int id;
  int frame_count;
  int missing_count;
  bool is_updated;
}quadrilateral;

typedef struct edgePixel_s
{
  int x;
  int y;
  float gx;
  float gy;
  float angle;
}edgePixel;

typedef struct lineSegment_s
{
  edgePixel start;
  edgePixel end;
  float n_xdiff;
  float n_ydiff;
  bool is_merged;
}lineSegment;

typedef struct candidateSegment_s
{
  int index;
  int distance;
  bool is_flipped;
}candidateSegment;
typedef std::vector<edgePixel> EdgePixelVector;

typedef std::vector<lineSegment> LineSegmentVector;
typedef std::vector<candidateSegment> CandidateSegmentVector;
typedef std::vector<quadrilateral> QuadrilateralVector;

typedef std::vector<edgePixel> * EdgePixelVectorPtr;
typedef std::vector<lineSegment> * LineSegmentVectorPtr;
typedef std::vector<quadrilateral> * QuadrilateralVectorPtr;

void fast_srand_seed( int seed );
int fast_srand();
 
void normalize_gx_gy(int gx,int gy,float &nx,float &ny);
inline float fast_inv_sqrt(float x);
inline float inv_sqrt(int x);

//TODO: convert all distance_squared function to template or macro
inline int distance_squared(const edgePixel &p1, const edgePixel &p2)
{
  int xdiff = p1.x - p2.x;
  int ydiff = p1.y - p2.y;
  return xdiff*xdiff + ydiff*ydiff;
}

inline float distance_squared_quad(const quadrilateral &q1, const quadrilateral &q2)
{
  int xdiff = q1.center.x - q2.center.x;
  int ydiff = q1.center.y - q2.center.y;
  return xdiff*xdiff + ydiff*ydiff;
}

inline float distance_squared_position(const position &p1, const position&p2)
{
  float xdiff = p1.x - p2.x;
  float ydiff = p1.y - p2.y;
  return xdiff*xdiff + ydiff*ydiff;
}
  
bool is_quad_convex(quadrilateral & quad);

inline bool compare_candidate_segments(const candidateSegment &one,const candidateSegment& two)
{
  return one.distance < two.distance;
}
inline bool is_line_segment_merged(const lineSegment &l)
{
  return l.is_merged;
}

void find_intersection(const lineSegment& line_one, const lineSegment& line_two, position & corner);

class EdgePixelMatchPred
{
private:
  const edgePixel _p;
public:
  EdgePixelMatchPred(const edgePixel &p):_p(p){}
  bool operator()(const edgePixel & pixel)const
  {
    return (pixel.x == _p.x && pixel.y == _p.y);
  }
};

class ItemInOtherVectorPred
{
private:
  const EdgePixelVector* _other;
public:
  ItemInOtherVectorPred(const  EdgePixelVector* other) : _other(other) {}

  bool operator()(const edgePixel& pixel) const
  {
    EdgePixelMatchPred equal_predicate(pixel);
    return (std::find_if(_other->begin(),_other->end(),equal_predicate) != _other->end());
  }
};

class QuadMissingInManyFrames
{
 public:
  bool operator()(const quadrilateral&q)const
  {
    return q.missing_count > QUAD_MISSING_FRAME_THRESHOLD;
  }
};

#endif
