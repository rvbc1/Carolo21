/*
 * PID.cpp
 *
 *  Created on: 05.03.2020
 *      Author: Karolina
 */

#include <PID.h>


PID::PID(float kp, float ki, float kd) {

	this->kp = kp;
	if(kp == 0.0f){
		proportional_enable = false;
	} else {
		proportional_enable = true;
	}

	this->ki = ki;
	if(ki == 0.0f){
		integral_enable = false;
	} else {
		integral_enable = true;
	}

	this->kd = kd;
	if(kp == 0.0f){
		derivative_enable = false;
	} else {
		derivative_enable = true;
	}

	this->dt = 0;
	now_time = tools.GetMicros();
	before_time = tools.GetMicros();

	integral = 0.0f;
	proportional = 0.0f;
	derivative = 0.0f;
	output = 0.0f;
	set_point = 0.0f;
	measured = 0.0f;
	last_error = 0.0f;
}

float PID::calculate(){
	float error;
	float output = 0.0f;

	error = set_point - measured;


	if(proportional_enable){
		proportional = kp * error;
		output += proportional;
	}

	now_time = tools.GetMicros();
	dt = constrainf((now_time - before_time) * 1e-6F, (_dt/2), (_dt*2));
	before_time = now_time;

	if(integral_enable){
		integral += error * dt;
		integral = ki * integral;
		output += integral;
	}

	if(derivative_enable){
		derivative = (error-last_error)/dt;
		derivative = kd*derivative;
		output += derivative;
	}

	last_error = error;

	return output;
}

void PID::setKp(float kp){
	this->kp = kp;
}

void PID::setKi(float ki){
	this->ki = ki;
}

void PID::setKd(float kd){
	this->kd = kd;
}

void PID::setSP(float sp){
	set_point = sp;
}

float PID::getCV(float pv){
	measured = pv;
	return calculate();
}

float PID::getCV(float pv, float sp){
	measured = pv;
	set_point = sp;
	return calculate();
}

void PID::enableP(uint8_t enable) {
	proportional_enable = enable;
}

void PID::enableI(uint8_t enable) {
	integral_enable = enable;
}

void PID::enableD(uint8_t enable) {
	derivative_enable = enable;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

