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
	Task(uint64_t interval, uint64_t lastTick, void (*funct)(void));
	void setInterval(uint64_t interval);
	void setLastTick(uint64_t lastTick);
	void setFunction(void (*funct)(void));
	void  execute();
	uint64_t getInterval();
	uint64_t getLastTick();
	virtual ~Task();
private:
	uint64_t interval;
	uint64_t lastTick;
	void (*funct)(void);

};




#endif /* SCHEDULER_TASK_H_ */
