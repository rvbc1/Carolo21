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
private:
	uint8_t kp, ki, kd;

};


#endif /* CLASSES_PID_H_ */
