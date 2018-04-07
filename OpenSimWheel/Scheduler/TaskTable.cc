/*
 * TaskTable.cpp
 *
 *  Created on: Apr 5, 2018
 *      Author: John
 */

#include "TaskTable.h"


TaskTable::TaskTable()
{


}

void TaskTable::addTask(Task &t)
{
	tasks.push_back(&t);
}
void TaskTable::execute(TMS320F2806 processor)
{
	uint64_t ticks = processor.getTicks();

	for(std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); ++it)
	{
		if((*it)->getInterval() == 0)
		{
			(*it)->execute();
		}

		else if (ticks - (*it)->getLastTick() >= (*it)->getInterval())
		{
			(*it)->execute();
			(*it)->setLastTick(ticks);

		}
	}
}


TaskTable::~TaskTable()
{
	// TODO Auto-generated destructor stub
}

