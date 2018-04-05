/*
 * Task.cc
 *
 *  Created on: Apr 5, 2018
 *      Author: John
 */

#include "Task.h"

namespace std {
Task::Task(uint64_t interval, uint64_t lastTick,void (*funct)(void))
{
	this->interval = interval;
	this->lastTick = lastTick;
	this->funct = funct;
}
Task::Task()
{
	// TODO Auto-generated constructor stub

}

Task::~Task() {

	// TODO Auto-generated destructor stub
}
void Task::setInterval(uint64_t interval)
{
	this->interval = interval;
}
void Task::setLastTick(uint64_t lastTick)
{
	this->lastTick = lastTick;
}
void Task::setFunction(void (*funct)(void))
{
	this->funct = funct;
}
uint64_t Task::getInterval()
{
	return interval;
}
uint64_t Task::getLastTick()
{
	return lastTick;
}
void  Task:: execute()
{
	this->funct();
}

} /* namespace std */
