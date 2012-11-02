#ifndef __TIMEPROFILER_H__
#define __TIMEPROFILER_H__
#define __PROFILER__

#ifdef __PROFILER__

#include <mach/mach_time.h>
/* Get the timebase info */
mach_timebase_info_data_t _info_profiler;
kern_return_t _err_profiler = mach_timebase_info(&_info_profiler);
double _timer_conversion = 1e-6*_info_profiler.numer/_info_profiler.denom;
#define PROFILE_START(tag)				\
  uint64_t start_time##tag = mach_absolute_time();
#define PROFILE_END(tag)						\
  static double total_time##tag = 0.0;					\
  static uint64_t counter##tag=0;			       		\
  counter##tag++;							\
  uint64_t end_time##tag = mach_absolute_time();			\
  double duration##tag = _timer_conversion*(end_time##tag - start_time##tag); \
  total_time##tag +=duration##tag;					\
  printf("\n%s = %1.3f mS Total=%1.3f mS iter = %lld Avg=%1.3f mS",#tag,duration##tag,total_time##tag,counter##tag,total_time##tag/counter##tag);
#else
#define PROFILE_START(tag)
#define PROFILE_END(tag)
#endif
#endif
