#include "RobotTimer.h"

rbtTime RobotTimer::getTime_us() {

	struct timeval nowTimeVal;
   gettimeofday(&nowTimeVal, 0);
	return (S_TO_US*(rbtTime)nowTimeVal.tv_sec + nowTimeVal.tv_usec);
}

rbtTime RobotTimer::getDeltaT_us(rbtTime start) {
	return getTime_us()-start;
}

void RobotTimer::delay_us(rbtTime time_us) {

	rbtTime start = getTime_us();
	while ((getTime_us()-start) < time_us);
}