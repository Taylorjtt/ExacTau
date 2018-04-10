/*
 * SerialSendTask.cpp
 *
 *  Created on: Apr 6, 2018
 *      Author: John
 */

#include "SerialSendTask.h"

uint32_t SerialSendTask::getPackedCurrentData(void)
{
	uint32_t packedData = 0;
	packedData |=  ((uint32_t)AdcResult.ADCRESULT0) << 12;
	packedData |= ((uint32_t)AdcResult.ADCRESULT1);
	return packedData;
}
SerialSendTask::~SerialSendTask()
{

}
