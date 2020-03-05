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
	virtual ~PID();
private:
	uint16_t kp, kd, ki ;

};

#endif /* CLASSES_PID_H_ */
