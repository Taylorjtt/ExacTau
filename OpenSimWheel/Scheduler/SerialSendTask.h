/*
 * SerialSendTask.h
 *
 *  Created on: Apr 6, 2018
 *      Author: John
 */

#ifndef SCHEDULER_SERIALSENDTASK_H_
#define SCHEDULER_SERIALSENDTASK_H_

#include "Task.h"
#include "../Hardware/OSWHardware.hh"

class SerialSendTask: public Task {
public:
	SerialSendTask(uint64_t interval, uint64_t lastTick,OSWSerial serial, OSWDigital digital):Task(interval,lastTick)
	{
		this->serial = serial;
		this->digital = digital;
	}
	SerialSendTask():Task(0,0)
	{

	}
	virtual void execute()
	{
		serial.send('A');
		serial.send('\n');
		digital.toggle(GPIO_Number_33);

	}
	virtual  ~SerialSendTask();
private:
	OSWSerial serial;
	OSWDigital digital;
};

#endif /* SCHEDULER_SERIALSENDTASK_H_ */
