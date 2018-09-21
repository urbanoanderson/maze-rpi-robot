/* 
 * File:   PIDController2.cpp
 * Author: lenovo02
 * 
 * Created on 5 de Março de 2015, 18:42
 */

#include "PIDController.h"

PIDController::PIDController(const float Kp_, const float Ki_, const float Kd_, const float Off_, const float ACT_)
{
    setParam(Kp_, Ki_, Kd_, Off_, ACT_);

    hiOutput = std::numeric_limits<float>::max();
    loOutput = -std::numeric_limits<float>::max();

    hiInput = std::numeric_limits<float>::max();
    loInput = -std::numeric_limits<float>::max();
}

PIDController::PIDController()
{
    setParam(1, 0, 0, 0, 1);
    SP = 0;
    PV = 0;

    hiOutput = std::numeric_limits<float>::max();
    loOutput = -std::numeric_limits<float>::max();

    hiInput = std::numeric_limits<float>::max();
    loInput = -std::numeric_limits<float>::max();
}

PIDController& PIDController::setLoInput(float loInput)
{
    this->loInput = loInput;
    return *this;
}

float PIDController::getLoInput() const {
    return loInput;
}

PIDController& PIDController::setHiInput(float hiInput)
{
    this->hiInput = hiInput;
    return *this;
}

float PIDController::getHiInput() const {
    return hiInput;
}

PIDController& PIDController::setLoOutput(float loOutput)
{
    this->loOutput = loOutput;
    return *this;
}

float PIDController::getLoOutput() const {
    return loOutput;
}

PIDController& PIDController::setHiOutput(float hiOutput)
{
    this->hiOutput = hiOutput;
    return *this;
}

float PIDController::getHiOutput() const {
    return hiOutput;
}

void PIDController::setACT(float ACT) {
    this->ACT = ACT;
}

float PIDController::getACT() const {
    return ACT;
}

float PIDController::revertACT() {
    ACT=ACT*(-1);
    return ACT;
}

PIDController::PIDController(const PIDController& orig)
{
    this->Kp = orig.Kp;
    this->Ki = orig.Ki;
    this->Kd = orig.Kd;
    this->Off = orig.Off;
    this->SP = orig.SP;
    this->PV = orig.PV;
    this->ACT = orig.ACT;
    this->m = orig.m;
    this->hiInput = orig.hiInput;
    this->loInput = orig.loInput;
    this->hiOutput = orig.hiOutput;
    this->loOutput = orig.loOutput;

    for (int i = 0; i < N; i++) 
    	e[i] = orig.e[i];
}

PIDController& PIDController::setParam(const float Kp_, const float Ki_, const float Kd_, const float Off_, const float ACT_, const float vi0)
{
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    Off = Off_;
    ACT = ACT_;

    //vi0-> Valor inicial para a integração
    for (int i = 0; i < N; i++)
    	e[i] = vi0;

    return *this;
}

PIDController& PIDController::setKp(float Kp_)
{
	Kp = Kp_;
	return *this;
}

PIDController& PIDController::setKi(float Ki_)
{
	Ki = Ki_;
	return *this;
}

PIDController& PIDController::setKd(float Kd_)
{
	Kd = Kd_;
	return *this;
}


PIDController& PIDController::setHiLoInput(const float hi, const float lo) 
{
    hiInput = hi;
    loInput = lo;

    return *this;
}

PIDController& PIDController::setHiLoOutput(const float hi, const float lo)
{
    hiOutput = hi;
    loOutput = lo;

    return *this;
}
