/* 
 * File:   Pins.h
 * Author: hans
 *
 * Created on 4 de Junho de 2015, 20:27
 */

#ifndef PINS_H
#define	PINS_H

#include <string>
#include <sstream>
#include <cstdlib>
#include <iostream>
#include <wiringPi.h>

using namespace std;

#define PIN_BCM 0
#define PIN_WPI_NUM 1

#define PIN_MODE  PIN_BCM //PIN_WPI_NUM

#define SONAR_FRONT_ECHO  		14 //BCM
#define SONAR_FRONT_TRIGGER  	15 //BCM
#define SONAR_LEFT_ECHO  		 5 //BCM
#define SONAR_LEFT_TRIGGER  	 6 //BCM
#define SONAR_RIGHT_ECHO  		19 //BCM
#define SONAR_RIGHT_TRIGGER  	26 //BCM

#define ENCODER_LEFT			10 //BCM
#define ENCODER_RIGHT			 9 //BCM

#define MOTOR_LEFT_A			23 //BCM
#define MOTOR_LEFT_B			24 //BCM
#define MOTOR_LEFT_E			18 //12 ou 18 BCM
#define MOTOR_RIGHT_A			17 //BCM
#define MOTOR_RIGHT_B			27 //BCM
#define MOTOR_RIGHT_E			13 //13 ou 19 BCM

int sysconfgGPIOEdge(int pinBCM, std::string mode); 

#endif

