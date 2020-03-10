/*
 * PID.h
 *
 *  Created on: 05.03.2020
 *      Author: Karolina
 */

#ifndef CLASSES_PID_H_
#define CLASSES_PID_H_

#include "main.h"
#include <Tools.h>

class PID {
public:
	PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

	virtual ~PID();
	void setKp(float KP);
	void setKi(float KI);
	void setKd(float KD);
	void measure(uint8_t M);
	void set(float S);
	void calculate();
	void setDt(float DT);


private:
	uint8_t proportional_enable, integral_enable, derivative_enable;
	float integral, proportional, derivative,output;
	float kp, ki, kd, dt;
	float set_point;
	float measured;
	float last_error;

	int32_t now_time;
	int32_t before_time;
};


#endif /* CLASSES_PID_H_ */
