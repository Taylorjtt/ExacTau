/*
 * TaskTable.h
 *
 *  Created on: Apr 5, 2018
 *      Author: John
 */

#ifndef SCHEDULER_TASKTABLE_H_
#define SCHEDULER_TASKTABLE_H_
#include <vector>
#include "Task.h"

#define FREQ_1KHZ 100
#define FREQ_100HZ 1000

class TaskTable {
public:
	TaskTable();
	void addTask(Task &t);
	void execute(TMS320F2806 processor);
	virtual ~TaskTable();
private:
	std::vector<Task*> tasks;
};


#endif /* SCHEDULER_TASKTABLE_H_ */
