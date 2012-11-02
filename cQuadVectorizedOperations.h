#ifndef __cquad_vectorized_operations__
#define __cquad_vectorized_operations__

#include <opencv2/core/core.hpp>
#include "commonDef.h"

using namespace cv;

struct EdgelList {
public:
	size_t size;
	bool areOrientationsCompatible(int idx1,int idx2){
		return ((slopex[idx1]*slopex[idx2] + slopey[idx1]*slopey[idx2]) > 0.38f); //cosf( 67.5f / 2 pi ) ; //
	}

	bool edgesOnSameLine(int ldx1,int ldx2,int edx){
		if(!areOrientationsCompatible(ldx1,edx)) return false;

		float cross = (float)(x[ldx2]-x[ldx1]) *(float)(y[edx]-y[ldx1]);
		cross -= (float)(y[ldx2]-y[ldx1]) *(float)(x[edx]-x[ldx1]);

		const float d1 = (float)(x[ldx1]-x[ldx2]);
		const float d2 = float(y[ldx1]-y[ldx2]);

		//float distance = cross / Vector2f(d1, d2).get_length();
		float distance = (cross*cross)/(d1*d1+d2*d2);

		return fabs(distance) < ALLOWED_DISTANCE;
	}

	std::vector<short> x,y;
	std::vector<float> slopex,slopey;

	void resize(const int size){
		x = std::vector<short>(size);
		y = std::vector<short>(size);
		slopex = std::vector<float>(size);
		slopey = std::vector<float>(size);
	}

	size_t capacity() const{
		return x.size();
	}
	EdgelList(){
		resize(0);
	}
};

struct cQuadMarkerParams{
	int RASTERSIZE,REGIONSIZE;
	int EDGE_THRESHOLD,WHITE_THRESHOLD;
	int EDGELS_ON_LINE;

	cQuadMarkerParams():EDGE_THRESHOLD(5),WHITE_THRESHOLD(10),RASTERSIZE(PIXELSKIP),REGIONSIZE(REGION_SIZE),
			EDGELS_ON_LINE(4){}
};

class cQuadMarkerVectorized{
private:
	cQuadMarkerParams params;
	cv::Mat edgelMap;
	cv::Mat edgelScratchH,edgelScratchV;
	EdgelList edgels; //per block edgels

	void detectEdgels(const cv::Mat& img,cv::Mat& edgelMap,const short edgelThreshold,const short rastersize);

	void computeEdgelGradient(const cv::Mat& img,const cv::Mat& edgelMap,EdgelList& edgels,int startX,int startY);
	void computeLineSegments(EdgelList& edgels,LineSegmentVector& lineSegments);
	void reset();

	void resizeEdgelsList(){
		if(edgels.capacity() != params.REGIONSIZE*params.REGIONSIZE){
			edgels.resize(params.REGIONSIZE*params.REGIONSIZE);
		}
	}
public:
	cQuadMarkerVectorized(){
		resizeEdgelsList();
	}
	void setParams(void* _params){
		params =  *((cQuadMarkerParams*)_params);
		resizeEdgelsList();
	}
	void find_edgepixelsAndLineSegments(const Mat& image,LineSegmentVector &lines);
};

#endif
