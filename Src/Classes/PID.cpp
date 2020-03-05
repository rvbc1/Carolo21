/*
 * PID.cpp
 *
 *  Created on: 05.03.2020
 *      Author: Marta
 */

#include <PID.h>
void PID::setkp (uint16_t KP){
	if(KP > 100) kp = 100;
	kp = KP;
}
void PID::pid () {
	//TODO
	//implementacja regulatora pid
	output = 1 ;
}
uint16_t PID::getOutput(){
	pid();
	return output;
}
PID::PID() {
	ki = 0;
	kp = 0;
	kd = 0;
	// TODO Auto-generated constructor stub

}
PID::PID(uint16_t KP){
	kp = KP;
}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

