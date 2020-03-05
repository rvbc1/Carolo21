/*
 * PID.h
 *
 *  Created on: 05.03.2020
 *      Author: Marta
 */

#ifndef CLASSES_PID_H_
#define CLASSES_PID_H_
#include <main.h>

class PID {
public:
	PID();
	PID(uint16_t KP);
	virtual ~PID();
	void setkp (uint16_t KP);
	uint16_t getOutput();


private:
	uint16_t kp, kd, ki , output;
	void pid() ;

};



#endif /* CLASSES_PID_H_ */
