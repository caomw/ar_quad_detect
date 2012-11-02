#include "cMatcher.h"

void cMatch::init()
{
  _prev_quads.reserve(QUAD_RESERVE_SIZE);
  QuadList.reserve(QUAD_RESERVE_SIZE);
  _running_id = 0;
#ifdef __DEBUG_INFO__
  _debug = new cVisualizer("Match Debug Info");
#endif
}

void cMatch::release()
{
#ifdef __DEBUG_INFO__
  delete _debug;
#endif
}

#ifdef __DEBUG_INFO__
void cMatch::set_debug_img(const cv::Mat & img)
{
  _debug->set_img(img);
}
#endif


void cMatch::update_match(const QuadrilateralVector & quads)
 {
   //TODO: Add prediction to prev quads to get a better estimate of
   // quad center and to account for when detection fails.
   const int prev_quads_count = _prev_quads.size();
   const int curr_quads_count = quads.size();

   //NOTE: assume each quad is missing till it is found
   for(int j = 0; j < prev_quads_count; ++j)
     {
       ++ _prev_quads[j].missing_count;
        _prev_quads[j].is_updated = false;
     }

   for(int i = 0; i < curr_quads_count ; ++i)
     {
       const quadrilateral &qi = quads[i];
       int min_quad_index = -1;
       float min_center_distance = 9e9;
       float min_vertex_distance = 9e9;
       int min_vertex_index;

       //for(int j = 0; j < _prev_quads.size() ; ++j)
       //NOTE:the loop termination condition has to be the above line as we update the 
       //_prev_quads in the loop. But as we append newly found quads it should be 
       //alright to cheat and use only const size when we started for optimization purpose

       for(int j = 0; j < prev_quads_count; ++j)
	 {
	   const quadrilateral &qj =  _prev_quads[j];
	   if(!qj.is_updated)
	     {
	       const float quad_distance = distance_squared_quad(qi,qj);
	       TRACE("Distance:%f, Threshold %f",quad_distance,QUAD_CENTER_DISTANCE_THRESHOLD );
	       if(quad_distance < QUAD_CENTER_DISTANCE_THRESHOLD && quad_distance < min_center_distance)
		 {
		   const position &v0_prev = qj.c[0];
		   float this_quad_min_vertex_distance = 9e9;
	       
		   for(int v = 0; v< 4; ++v)
		     {
		       const float this_quad_vertex_distance = distance_squared_position(v0_prev,qi.c[v]);
		       //TRACE("Vertex %d : distance %f",v,this_quad_vertex_distance);
		       if(this_quad_vertex_distance < min_vertex_distance && this_quad_vertex_distance < this_quad_min_vertex_distance)
			 {
			   min_quad_index = j;
			   min_vertex_index = v;
			   min_vertex_distance = this_quad_vertex_distance;
			   this_quad_min_vertex_distance = this_quad_vertex_distance;
			 }
		     }
		 }
	     }
	 }
       
       if(min_quad_index != -1)//Found older matching quad
	 {
	   TRACE("%s (Min Vertex index :%d)","Match Found",min_vertex_index);
	   quadrilateral &q_prev =  _prev_quads[min_quad_index];
	   --q_prev.missing_count;
	   ++q_prev.frame_count;
	   q_prev.is_updated = true;
	   q_prev.center = qi.center;
	   //NOTE: Using switch to prevent use of MOD operator
	   switch(min_vertex_index)
	     {
	     case 0:
	       q_prev.c[0] = qi.c[0];
	       q_prev.c[1] = qi.c[1];
	       q_prev.c[2] = qi.c[2];
	       q_prev.c[3] = qi.c[3];
	       break;
	     case 1:
	       q_prev.c[0] = qi.c[1];
	       q_prev.c[1] = qi.c[2];
	       q_prev.c[2] = qi.c[3];
	       q_prev.c[3] = qi.c[0];
	       break;
	     case 2:
	       q_prev.c[0] = qi.c[2];
	       q_prev.c[1] = qi.c[3];
	       q_prev.c[2] = qi.c[0];
	       q_prev.c[3] = qi.c[1];
	       break;
	     case 3:
	       q_prev.c[0] = qi.c[3];
	       q_prev.c[1] = qi.c[0];
	       q_prev.c[2] = qi.c[1];
	       q_prev.c[3] = qi.c[2];
	     }
	 }
       else//new quad found
	 {
	   TRACE("%s","New Quad Found");
	   quadrilateral q = qi;
	   q.missing_count = 0;
	   q.frame_count =1;
	   q.is_updated = true;
	   q.id = _running_id++;
	   _prev_quads.push_back(q);
	 }
     }

   QuadMissingInManyFrames missing_predicate;
   _prev_quads.erase(std::remove_if(_prev_quads.begin(),_prev_quads.end(),missing_predicate),_prev_quads.end());
   const int quad_count = _prev_quads.size();
   QuadList.clear();
   for(int j = 0; j < quad_count; ++j)
     {
       if(_prev_quads[j].frame_count >= QUAD_FOUND_FRAME_THRESHOLD)
	 {
	   QuadList.push_back(_prev_quads[j]);
	 }
     }
#ifdef __DEBUG_INFO__
   //_debug->draw_quadrilateral_vector(quads,true);
   _debug->draw_quadrilateral_vector(QuadList);
   _debug->show_img();
#endif
 }
