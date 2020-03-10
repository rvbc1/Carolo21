/*
 * PID.cpp
 *
 *  Created on: 05.03.2020
 *      Author: Karolina
 */

#include <PID.h>

PID::PID() {
	// TODO Auto-generated constructor stub
	kp=0;
	ki=0;
	kd=0;
	dt=1;
	last_error=0;

}

void PID::calculate(){
	uint8_t error;
	uint8_t output;

	error=set-measured;

	proportional=kp*error;

	integral+=error*dt;
	integral=ki*integral;

	derivative = (error-last_error)/dt;
	derivative = kd*derivative;

	output=proportional+derivative+integral;


	last_error=error;

}

void PID::setKp(uint8_t KP){
	if(KP > 100) kp = 100;
	else kp = KP;
}

void PID::setKi(uint8_t KI){
	if(KI > 100) ki = 100;
	else ki = KI;
}

void PID::setKd(uint8_t KD){
	if(KD > 100) kd = 100;
	else kd = KD;
}

void PID::setDt(uint8_t DT){
	dt = DT;
}

void PID::measure(uint8_t M){
	measured=M;
}

void PID::SET(uint8_t S){
	set=S;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

