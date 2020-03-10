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
	//	dt = constrainf((now - before) * 1e-6F, (_dt/2), (_dt*2));
}

void PID::calculate(){
	uint8_t error;
	uint8_t output;

	error=set_point-measured;

	proportional=kp*error;

	integral+=error*dt;
	integral=ki*integral;

	derivative = (error-last_error)/dt;
	derivative = kd*derivative;

	output=proportional+derivative+integral;


	last_error=error;

}

void PID::setKp(float KP){
	if(KP > 100) kp = 100;
	else kp = KP;
}

void PID::setKi(float KI){
	if(KI > 100) ki = 100;
	else ki = KI;
}

void PID::setKd(float KD){
	if(KD > 100) kd = 100;
	else kd = KD;
}

void PID::setDt(float DT){
	dt = DT;
}

void PID::measure(uint8_t M){
	measured=M;
}

void PID::set(float S){
	set_point=S;
}

float getCV(float pv, float sp){
	return 0.0f;
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

