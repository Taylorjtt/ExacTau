/*
 * Task.h
 *
 *  Created on: Apr 5, 2018
 *      Author: John
 */

#ifndef SCHEDULER_TASK_H_
#define SCHEDULER_TASK_H_
#include <stdint.h>
#include "../Hardware/TMS320F2806.hh"

class Task {
public:
	Task();
	Task(uint64_t interval, uint64_t lastTick)
	{
		this->interval = interval;
		this->lastTick = lastTick;
	}
	void setInterval(uint64_t interval)
	{
		this->interval = interval;
	}
	void setLastTick(uint64_t lastTick)
	{
		this->lastTick = lastTick;
	}

	uint64_t getInterval()
	{
		return interval;
	}
	uint64_t getLastTick()
	{
		return lastTick;
	}
	virtual void  execute() =0;
	virtual ~Task();
private:
	uint64_t interval;
	uint64_t lastTick;

};




#endif /* SCHEDULER_TASK_H_ */
