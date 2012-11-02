#include <iostream>
#include "cQuadVectorizedOperations.h"

using namespace cv;

void cQuadMarkerVectorized::detectEdgels(const cv::Mat& img,cv::Mat& edgelMap,const short edgelThreshold,const short rastersize){
	unsigned char* in = img.data;

	edgelMap.create(img.rows,img.cols,CV_8UC1);
	unsigned char* out = edgelMap.data;

	edgelScratchH.create(1,img.cols,CV_8UC1);
	unsigned char* ss = edgelScratchH.data;

	short left,right,current,prev1,prev2;
	int step_offset = rastersize*img.step;
	int i,j,h=img.rows,s=img.cols,k;

	for(i=0;i<h;i+=rastersize,
	in+=step_offset,out+=step_offset){
		for(j=0;j<s;j++)
			ss[j] = in[j]>>1;

		for(j=2,prev1=prev2=0;j<s-2;j++){
			left = (ss[j-2]) + (in[j-1]);
			right = (in[j+1]) + (ss[j+2]);
			current = (short)fabs(right-left);
			current=(current>edgelThreshold)?current:0;

			if(prev1>0 && prev1>prev2 && prev1>current)
				out[j-1] = 1;

			prev2 = prev1;
			prev1 = current;
		}
	}

	edgelScratchV.create(1,img.rows,CV_8UC1);
	ss = edgelScratchV.data;
	size_t step=img.step;
	in = img.data;
	out = edgelMap.data;

	for(j=0;j<s;j+=rastersize){

		for(i=0,k=j;i<h;i++,k+=step)
			ss[i] = in[k]>>1;

		for(i=2,k=j+2*step,prev1=prev2=0;i<h-2;i++,k+=step){
			left = ss[i-2] + in[k-step];
			right = in[k+step] + ss[i+2];
			current = (short)fabs(right-left);

			current = (current>edgelThreshold)?current:0;
			if(prev1>0 && prev1>prev2 && prev1>current)
				out[k-step] = 2;

			prev2 = prev1;
			prev1 = current;
		}
	}
}

void cQuadMarkerVectorized::computeEdgelGradient(const cv::Mat& img,const cv::Mat& edgelMap,EdgelList& edgels,int startX,int startY){
	int y,x;
	int endY = std::min(startY+params.REGIONSIZE,img.rows);
	int endX = std::min(startX+params.REGIONSIZE,img.cols);

	unsigned char* map = edgelMap.data + startY*edgelMap.step;
	unsigned char* im = img.data + startY*img.step;
	int step_offset = params.RASTERSIZE*edgelMap.step,step=img.step;
	short k=0;
	for(y=startY;y<endY;y+=params.RASTERSIZE,map+=step_offset,im+=step_offset){
		for(x=startX;x<endX;x++){
			if(map[x]){
				//compute gradient
				edgels.x[k] = x,edgels.y[k] = y;
				edgels.slopex[k] =  (im[x-step-1]>>2 + im[x-step]>>1 + im[x-step+1]>>2)-
						(im[x+step-1]>>2 + im[x+step]>>1 + im[x+step+1]>>2);
				edgels.slopey[k] = (im[x-step-1]>>2 + im[x-1]>>1 + im[x+step-1]>>2)-
						(im[x-step+1]>>2 + im[x+1]>>1 + im[x+step+1]>>2);
				k++;
			}
		}
	}

	map = edgelMap.data + startY*edgelMap.step;
	im = img.data + startY*edgelMap.step;
	int x1;
	for(x=startX;x<endX;x+=params.RASTERSIZE){
		for(y=startY,x1=x;y<endY;y++,x1+=step){
			if(map[x1]){
				//compute gradient
				edgels.x[k] = x,edgels.y[k] = y;
				edgels.slopex[k] =  (im[x1-step-1]>>2 + im[x1-step]>>1 + im[x1-step+1]>>2)-
						(im[x1+step-1]>>2 + im[x1+step]>>1 + im[x1+step+1]>>2);
				edgels.slopey[k] = (im[x1-step-1]>>2 + im[x1-1]>>1 + im[x1+step-1]>>2)-
						(im[x1-step+1]>>2 + im[x1+1]>>1 + im[x1+step+1]>>2);
				k++;
			}
		}
	}
	edgels.size = k;
}

void cQuadMarkerVectorized::computeLineSegments(EdgelList& edgels,LineSegmentVector &lineSegments){
	lineSegment lineSegmentInRun;

	int edgel_size = edgels.size;
	int support_edgelsInRun_size=0,support_edgels_size=0;
	std::vector<short> supportEdgels(edgel_size);
	std::vector<short> supportEdgelsInRun(edgel_size);
	int startEdgel,endEdgel;

	edgePixel start_edge,end_edge;
	srand(time(NULL));

	do {
		const int max_iterations = 50;
		int iteration,ir1,ir2,k;

		for (int i = 0; i < 15; i++) {
			iteration = 0;
			do {
				ir1 = (rand()%edgel_size);
				ir2 = (rand()%edgel_size);
				iteration++;
			} while ( ( ir1 == ir2 || !edgels.areOrientationsCompatible(ir1,ir2)) && iteration < max_iterations );

			if( iteration < max_iterations ) {
				//LineSegment lineSegment;
				//lineSegment.start = r1;
				//lineSegment.end = r2;
				//lineSegment.slope = r1.slope;
				support_edgels_size=0;
				for(k=0;k<edgel_size;k++) {
					if(edgels.edgesOnSameLine(ir1,ir2,k)){
						supportEdgels[support_edgels_size++]=k;
					}
					//if( lineSegment.atLine( edgels.at(o) ) ) {
					//	lineSegment.addSupport( edgels.at(o) );
					//}
				}

				if(support_edgels_size>support_edgelsInRun_size){
					support_edgelsInRun_size = support_edgels_size;
					supportEdgelsInRun = supportEdgels;
					startEdgel = ir1;
					endEdgel = ir2;
				}
				//if( lineSegment.supportEdgels.size() > lineSegmentInRun.supportEdgels.size() ) {
				//	lineSegmentInRun = lineSegment;
				//}
			}
		}

		// slope van de line bepalen
		//if( lineSegmentInRun.supportEdgels.size() >= EDGELSONLINE )
		if( support_edgelsInRun_size >= params.EDGELS_ON_LINE ){
			float u1 = 0;
			float u2 = 50000;

			//const Vector2f slope = (lineSegmentInRun.start.position - lineSegmentInRun.end.position);
			float slopex = edgels.x[startEdgel]-edgels.x[endEdgel],slopey = edgels.y[startEdgel]-edgels.y[endEdgel];
			// why ?
			//const Vector2f orientation = Vector2f(-lineSegmentInRun.start.slope.y, lineSegmentInRun.start.slope.x );
			float orientationx = -edgels.slopey[startEdgel],orientationy=edgels.slopex[startEdgel];

			if (abs (slopex) <= abs(slopey)) {
				//for (std::vector<short>::const_iterator it = supportEdgelsInRun.begin(); it!=supportEdgelsInRun.end(); ++it) {
				for(int j=0;j<support_edgelsInRun_size;j++){
					if(edgels.y[supportEdgelsInRun[j]]>u1){
						u1=edgels.y[supportEdgelsInRun[j]];
						startEdgel = supportEdgelsInRun[j];
					}
					if(edgels.y[supportEdgelsInRun[j]]<u2){
						u2=edgels.y[supportEdgelsInRun[j]];
						endEdgel = supportEdgelsInRun[j];
					}
					/*if ((*it).position.y > u1) {
  						u1 = (*it).position.y;
  						lineSegmentInRun.start = (*it);
  					}

  					if ((*it).position.y < u2) {
  						u2 = (*it).position.y;
  						lineSegmentInRun.end = (*it);
  					}*/
				}
			} else {
				//for (std::vector<Edgel>::iterator it = lineSegmentInRun.supportEdgels.begin(); it!=lineSegmentInRun.supportEdgels.end(); ++it) {
				for(int j=0;j<support_edgelsInRun_size;j++){
					if(edgels.x[supportEdgelsInRun[j]]>u1){
						u1=edgels.x[supportEdgelsInRun[j]];
						startEdgel = supportEdgelsInRun[j];
					}
					if(edgels.x[supportEdgelsInRun[j]]<u2){
						u2=edgels.x[supportEdgelsInRun[j]];
						endEdgel = supportEdgelsInRun[j];
					}
					/*if ((*it).position.x > u1) {
  						u1 = (*it).position.x;
  						lineSegmentInRun.start = (*it);
  					}

  					if ((*it).position.x < u2) {
  						u2 = (*it).position.x;
  						lineSegmentInRun.end = (*it);
  					}*/
				}
			}

			// switch startpoint and endpoint according to orientation of edge
			//if( dot( lineSegmentInRun.end.position - lineSegmentInRun.start.position, orientation ) < 0.0f ) {
			//	std::swap( lineSegmentInRun.start, lineSegmentInRun.end );
			//}
			float x1 = (edgels.x[endEdgel]-edgels.x[startEdgel]),y1=(edgels.y[endEdgel]-edgels.y[startEdgel]);
			float dot_product = x1*orientationx + y1*orientationy;
			if(dot_product<0)
				std::swap(startEdgel,endEdgel);

			start_edge.x = edgels.x[startEdgel],start_edge.y=edgels.y[startEdgel];
			start_edge.gx = edgels.slopex[startEdgel],start_edge.gy = edgels.slopey[startEdgel];

			end_edge.x = edgels.x[endEdgel],end_edge.y=edgels.y[endEdgel];
			end_edge.gx = edgels.slopex[endEdgel],end_edge.gy = edgels.slopey[endEdgel];

			lineSegmentInRun.start = start_edge;
			lineSegmentInRun.end = end_edge;

			//lineSegmentInRun.slope = (lineSegmentInRun.end.position - lineSegmentInRun.start.position).get_normalized();
			lineSegments.push_back( lineSegmentInRun );

			int i,j,last_pos=edgel_size-1;
			for(int i=0;i<=last_pos;){
				for(j=0;j<support_edgelsInRun_size;j++){
					if(i==supportEdgelsInRun[j])
						break;
				}
				if(j<support_edgelsInRun_size){
					std::swap(edgels.x[i],edgels.x[last_pos]);
					std::swap(edgels.y[i],edgels.y[last_pos]);
					std::swap(edgels.slopex[i],edgels.slopex[last_pos]);
					std::swap(edgels.slopey[i],edgels.slopey[last_pos]);
					last_pos--;
				}else i++;
			}
			edgel_size = last_pos+1;
			/*for(unsigned int i=0; i<lineSegmentInRun.supportEdgels.size(); i++) {
  				for (std::vector<Edgel>::iterator it = edgels.begin(); it!=edgels.end(); ++it) {
  					if( (*it).position.x == lineSegmentInRun.supportEdgels.at(i).position.x &&
  						(*it).position.y == lineSegmentInRun.supportEdgels.at(i).position.y ) {
  						edgels.erase( it );
  						break;
  					}
  				}
  			}*/
		}

	} while( support_edgelsInRun_size >= params.EDGELS_ON_LINE && edgel_size >= params.EDGELS_ON_LINE);
}

void cQuadMarkerVectorized::reset(){
	edgelMap.setTo(cv::Scalar(0,0,0));
}

void cQuadMarkerVectorized::find_edgepixelsAndLineSegments(const Mat& image,LineSegmentVector &lines){
	detectEdgels(image,edgelMap,params.EDGE_THRESHOLD,params.RASTERSIZE);
	int y,x;
	if(params.REGIONSIZE%params.RASTERSIZE)
		std::cerr<<"Warning: Regionsize and rastersize not coinciding"<<std::endl;

	for(y=0;y<image.rows;y+=params.REGIONSIZE){
		for(x=0;x<image.cols;x+=params.REGIONSIZE){
			computeEdgelGradient(image,edgelMap,edgels,x,y);
			computeLineSegments(edgels,lines);
		}
	}
}
