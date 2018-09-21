/* 
 * File:   Motor.cpp
 * Author: hans
 * 
 * Created on 4 de Junho de 2015, 23:22
 */

#include "Motor.h"

Motor::Motor()
{
}

int Motor::setup(int pinA, int pinB, int pinE, Encoder* encoder)
{
	this->pinA = pinA;
	this->pinB = pinB;
	this->pinE = pinE;
	this->encoder = encoder;

	pinMode(pinA, OUTPUT);
	pinMode(pinB, OUTPUT);
	pinMode(pinE, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(5000); //clock at ~10kHz

	setTargetSpeed(0);
	this->speed = 0;
	this->power = 0;
	this->minPower = 1;
	this->spdSamplingRate = 60000;
	pid.setParam(1.5, 5, 0.1);
   	pid.setHiLoInput(100,-100);
	pid.setHiLoOutput(100,-100);

	this->lastTime = 0;
	this->lastAngle = 0;

	return 0;
}

void Motor::setTargetSpeed(float targetSpeed)
{
	this->targetSpeed = targetSpeed;
	
	if (lastTime<=0)
		lastTime = RobotTimer::getTime_us();

	pid.setpoint(targetSpeed);
}

float Motor::getTargetSpeed()
{
	return this->targetSpeed;
}

void Motor::setSpdSamplingRate(rbtTime spr)
{
	this->spdSamplingRate = spr;
}

void Motor::setPower(float power)
{
	this->power = power;
}

void Motor::writePower()
{
	if(power>100)
		power = 100;
	else if(power<-100)
		power = -100;

	if (power>minPower)
	{
		 if (encoder->getDirection()!=1)
		 {
			digitalWrite(pinA, LOW);
			digitalWrite(pinB, HIGH);
			encoder->setDirection(1);
		 }

		 pwmWrite(pinE, rangePower(power));
	}

	else if (power<-minPower)
	{
		if(encoder->getDirection()!=-1)
		{
			digitalWrite(pinA, HIGH);
			digitalWrite(pinB, LOW);
			encoder->setDirection(-1);
		}

		pwmWrite(pinE, rangePower(-power));  
	}
	
	else
	{
		pwmWrite(pinE, rangePower(power));
		encoder->setDirection(0);
	}
}

void Motor::setMinPower(float minPower)
{
	this->minPower = minPower;
}

float Motor::getPower()
{
	return power;
}

float Motor::getSpeed()
{
	return speed;
}

void Motor::stop() 
{
	power = 0;
    pwmWrite(pinE, rangePower(power));
    encoder->setDirection(0);
}

float Motor::controlSpeed()
{
	speed = encoder->getSpeed();
	this->power += pid.input(speed);
        writePower();
	return speed;
}

int Motor::rangePower(float power)
{
	float pwr = power * (PWM_RANGE/100.0);

	if (pwr < 0)
		pwr = 0;
	else if (pwr > PWM_RANGE)
		pwr = PWM_RANGE;

	return pwr;
}


