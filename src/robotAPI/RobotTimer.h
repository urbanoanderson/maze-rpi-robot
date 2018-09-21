/* 
 * File:   RobotTimer.h
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 21:15
 */

#ifndef _ROBOT_TIMER_H_
#define	_ROBOT_TIMER_H_

#include <sys/time.h>

#define S_TO_US 1000000

typedef long long rbtTime;

class RobotTimer
{
	public:
		static rbtTime getTime_us();
		static rbtTime getDeltaT_us(rbtTime start);
		static void delay_us(rbtTime time);
};

#endif
