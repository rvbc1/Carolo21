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
#include <Mathematics.h>

class PID {

public:
	PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f);

	virtual ~PID();
	void setKp(float kp);
	void setKi(float ki);
	void setKd(float kd);

	void setSP(float sp);

	float getCV(float pv);
	float getCV(float pv, float sp);


	void enableP(uint8_t enable = true);
	void enableI(uint8_t enable = true);
	void enableD(uint8_t enable = true);


private:
	const float _dt = 0.002f;

	uint8_t proportional_enable, integral_enable, derivative_enable;
	float integral, proportional, derivative,output;
	float kp, ki, kd, dt;
	float set_point;
	float measured;
	float last_error;

	int32_t now_time;
	int32_t before_time;

	float calculate();
};


#endif /* CLASSES_PID_H_ */
