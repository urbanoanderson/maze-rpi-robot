/*
 * File:   Encoder.cpp
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 21:39
 */

#include "Encoder.h"

Encoder *leftEncoder = NULL;
Encoder *rigthEncoder = NULL;

static int minDeltaT = 8500;
static int maxDeltaT = 250000;

void countLeft()
{
    static rbtTime lastTime = RobotTimer::getTime_us();
    rbtTime now = RobotTimer::getTime_us();
    
    rbtTime dt = now - lastTime;
    
    // Filtra pulsos expurios em menos de dt us
    if (dt>minDeltaT)
    {
    	leftEncoder->updateSpeed(now);
		leftEncoder->count();
        lastTime = now;
    }
}

void countRight()
{
    static rbtTime lastTime = RobotTimer::getTime_us();
    rbtTime now = RobotTimer::getTime_us();

    rbtTime dt = now - lastTime;

    // Filtra pulsos expurios em menos de dt us
    if (dt>minDeltaT)
    {
    	rigthEncoder->updateSpeed(now);
		rigthEncoder->count();
    	lastTime = now;
    }
}

Encoder::Encoder()
{
	for (int i = 0; i < NBUF; i++)
		buffer[i] = 0;

	b = 0;
}

void Encoder::updateSpeed(rbtTime now) //On Edge Interrupt
{ 
	rbtTime dt = now - lastTime;
	float pulseSpeed = (M_PI*S_TO_US)/(20*dt); //A complete turn has 40 steps (2*pi/40)

	//std::cout << "PI: " << M_PI << " dt: " << (20.0*dt) << " pulseSpeed: " << pulseSpeed;
	speed = filter(direction*pulseSpeed);

	lastTime = now;
}

float Encoder::filter(float pulseSpeed)
{	
	//Adiciona ao buffer
	buffer[b] = pulseSpeed;
	b = (b+1)%NBUF;

	float sum = 0.0;
	for (int i = 0; i < NBUF; i++)
		sum += buffer[i];

	return sum / NBUF;
}

float Encoder::getSpeed()
{
	rbtTime now = RobotTimer::getTime_us();
	rbtTime dt = now - lastTime;

	if (dt > maxDeltaT)
		speed = filter(0);

	return speed;
}

void Encoder::setDirection(int direction)
{
    if (direction >= -1 && direction <= 1)
        this->direction = direction;
}

int Encoder::setup(int pin, int side)
{
	reset();

	this->pin = pin;

	if (side == RIGHT_SIDE)
	{
		rigthEncoder = this;
		//if (sysconfgGPIOEdge(pin, "both")!=0) return -1;
		return wiringPiISR(pin, INT_EDGE_BOTH, &countRight);
	}

	else
	{
		leftEncoder = this;
		//if (sysconfgGPIOEdge(pin, "both")!=0) return -1;
		return wiringPiISR(pin, INT_EDGE_BOTH, &countLeft);
	}
}

void Encoder::count()
{
    steps += direction;
}

void Encoder::reset()
{
	steps = lastStep = 0;
	direction = 0;
	speed = 0;
	lastTime = RobotTimer::getTime_us();
}

int Encoder::getDirection() const {
    return direction;
}

long Encoder::getSteps() const {
    return steps;
}

long Encoder::getDeltaSteps()
{
    long delta = steps - lastStep;
    lastStep = steps;
    return delta;
}

float Encoder::getAngle() const {
    //A complete turn has 40 steps (2*pi/40)
    return steps * M_PI / 20.0;
}

float Encoder::getDeltaAngle()
{
    return getDeltaSteps() * M_PI / 20.0;
}