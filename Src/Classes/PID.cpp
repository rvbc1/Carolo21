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
	return pid();
}
PID::PID() {
	// TODO Auto-generated constructor stub

}

PID::~PID() {
	// TODO Auto-generated destructor stub
}

