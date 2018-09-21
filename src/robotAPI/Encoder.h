/*
 * File:   Encoder.h
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 21:39
 */

#ifndef ENCODER_H
#define	ENCODER_H

#include <wiringPi.h>
#include <stdlib.h>
#include <math.h>
#include <sstream>
#include <iostream>
 
#include "Pins.h"
#include "RobotTimer.h"

#define LEFT_SIDE 0
#define RIGHT_SIDE 1
#define NBUF 4

class Encoder
{

	int pin;
	int direction;
	float speed;
	rbtTime lastTime;

	long steps;
	long lastStep;
	float buffer[NBUF]; 	//buffer circular
	int b;					//posicao no buffer

	public:
		Encoder();
		int setup(int pin, int side);
		void setDirection(int direction);
		void reset();	

		void updateSpeed(rbtTime now);
		void count();

		int getDirection() const;
		float getSpeed();

		long getSteps() const;
		long getDeltaSteps();
		float getAngle() const;
		float getDeltaAngle();

	private:
		float filter(float pulseSpeed);	
};

#endif

