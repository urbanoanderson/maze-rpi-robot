/* 
 * File:   Motor.h
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 23:22
 */

#ifndef MOTOR_H
#define	MOTOR_H

#include <wiringPi.h>
#include <iostream>
#include <cmath>

#include "Pins.h"
#include "Encoder.h"
#include "RobotTimer.h"
#include "PIDController.h"

#define PWM_RANGE 1024

class Motor
{
	public:
	    int pinA;
	    int pinB;
	    int pinE;

		float targetSpeed;
		float speed;
		float power;

		float minPower;

		rbtTime spdSamplingRate;
		rbtTime lastTime;
		float lastAngle;

		Encoder* encoder;
		PIDController pid;

	    int rangePower(float power);

	    Motor();
		int setup(int pinA, int pinB, int pinE, Encoder *encoder);
		void setTargetSpeed(float targetSpeed);
		float getTargetSpeed();
		void setSpdSamplingRate(rbtTime spr);
		void setMinPower(float minPower);
		void setPower(float power);
		void writePower();
		float getPower();
		float getSpeed();
		void stop();
		float controlSpeed();
};

#endif
