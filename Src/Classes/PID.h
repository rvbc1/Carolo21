/*
 * PID.h
 *
 *  Created on: 05.03.2020
 *      Author: Karolina
 */

#ifndef CLASSES_PID_H_
#define CLASSES_PID_H_

#include "main.h"

class PID {
public:
	PID();
	virtual ~PID();
	void setKp(uint8_t KP);
	uint8_t integral, proportional, derivative;
private:
	uint8_t kp, ki, kd, dt;

};


#endif /* CLASSES_PID_H_ */
