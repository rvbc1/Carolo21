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
	void setKi(uint8_t KI);
	void setKd(uint8_t KD);
	void measure(uint8_t M);
	void SET(uint8_t S);
	void calculate();
	void setDt(uint8_t DT);
	uint8_t integral, proportional, derivative,output;

private:
	uint8_t kp, ki, kd, dt;
	uint8_t set;
	uint8_t measured;
	uint8_t last_error;
};


#endif /* CLASSES_PID_H_ */
