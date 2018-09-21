/* 
 * File:   Sonar.cpp
 * Author: hans
 * 
 * Created on 4 de Junho de 2015, 20:26
 */

#include "Sonar.h"

Sonar::Sonar()
{
   divisor = DIVISOR;
}

int Sonar::setup(int pinTrigger, int pinEcho)
{
    //wiringPiISR(pinEcho, INT_EDGE_FALLING, &echo);
	this->pinTrigger = pinTrigger;
	this->pinEcho = pinEcho;

    pinMode(pinTrigger, OUTPUT);
    pinMode(pinEcho,INPUT);
    digitalWrite(pinTrigger, LOW);
    std::cout << "\rWaiting for sonar sensor to settle... ";
    delayMicroseconds(2000000);
    std::cout << "\rok" << std::endl;

    return 0;
}

void Sonar::setDivisor(float divisor)
{
  this->divisor = divisor;
}

float Sonar::measureDistance()
{
    rbtTime start, pulseTime, delta;

    //Send pulse
    digitalWrite(pinTrigger,LOW);
    delayMicroseconds(2);
    digitalWrite(pinTrigger,HIGH);
    delayMicroseconds(10);
    digitalWrite(pinTrigger,LOW);

    //wait for pulse:
    start = RobotTimer::getTime_us();
    do
    {
		pulseTime = RobotTimer::getTime_us();
		if (pulseTime-start > TIMEOUT)
            return -1.0;
    } while (digitalRead(pinEcho)==LOW);

    //wait for echo:
    do
    {
		delta = (RobotTimer::getTime_us() - pulseTime);
		if (delta > TIMEOUT)
            return -1.0;
    }while (digitalRead(pinEcho)==HIGH);

    return (delta*divisor) / 100.0;
}