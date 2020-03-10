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

PID::~PID() {
	// TODO Auto-generated destructor stub
}

