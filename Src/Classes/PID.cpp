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

}

void PID::setKp(uint8_t KP){
	if(KP > 100) kp = 100;
	else kp = KP;
}



PID::~PID() {
	// TODO Auto-generated destructor stub
}

