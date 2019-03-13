/*
 * QuadratureEncoder.cc
 *
 *  Created on: Jan 12, 2018
 *      Author: John
 */

#include "OSWHardware.hh"
#include "../math.h"
QuadratureEncoder::QuadratureEncoder()
{
}
QuadratureEncoder::QuadratureEncoder(TMS320F2806 processor,OSWDigital digital, uint16_t countsPerRevolution)
{
	this->offset = 0;

	processor.enableEQEP1Clock();
	this->qepHandle= QEP_init((void *)QEP1_BASE_ADDR, sizeof(QEP_Obj));
	this->countsPerRev = countsPerRevolution;
	this->ppr = 4 * countsPerRev;
	digital.setPullUp(GPIO_Number_21, GPIO_Pullup_Disable);
	digital.setPullUp(GPIO_Number_23, GPIO_Pullup_Disable);

	digital.setQualification(GPIO_Number_20, GPIO_Qual_Sync);
	digital.setQualification(GPIO_Number_21, GPIO_Qual_Sync);
	digital.setQualification(GPIO_Number_23, GPIO_Qual_Sync);

	digital.setMode(GPIO_Number_20, GPIO_20_Mode_EQEP1A);
	digital.setMode(GPIO_Number_21, GPIO_21_Mode_EQEP1B);
	digital.setMode(GPIO_Number_23, GPIO_23_Mode_EQEP1I);

	QEP_set_index_event_latch(this->qepHandle,QEPCTL_Iel_Rising_Edge);
	QEP_set_posn_count_reset_mode(this->qepHandle,QEPCTL_Pcrm_Index_Reset);
	QEP_set_QEP_source(this->qepHandle, QEP_Qsrc_Quad_Count_Mode);
	QEP_set_index_event_init(this->qepHandle,QEPCTL_Iei_Rising_Edge_Init);

	QEP_set_A_polarity(this->qepHandle,QEP_Qap_Inverted);


	QEP_set_emu_control(this->qepHandle, QEPCTL_Freesoft_Unaffected_Halt);
	QEP_set_watchdog_period(this->qepHandle,0xFFFF);

	QEP_set_posn_init_count(this->qepHandle,0);
	QEP_set_max_posn_count(this->qepHandle,countsPerRevolution * 4);

	QEP_enable_counter(this->qepHandle);
	QEP_set_unit_period(this->qepHandle, 1.6e6);
	QEP_set_capture_latch_mode(this->qepHandle, QEPCTL_Qclm_Latch_on_Unit_Timeout);
	QEP_set_capture_prescale(this->qepHandle, QCAPCTL_Ccps_Capture_Div_128);
	QEP_enable_capture(this->qepHandle);
	QEP_enable_interrupt(this->qepHandle, QEINT_Iel);



//	/*
//	 * We will use a velocity sampling rate of 50hz:
//	 * the count for the UTO is 2*processorSpeed/samplingRate
//	 *
//	 */
//	PIE_registerPieIntHandler(obj->pieHandle,PIE_GroupNumber_5,PIE_SubGroupNumber_1,encoderInterrupt);
//	PIE_enableInt(obj->pieHandle,PIE_GroupNumber_5,PIE_InterruptSource_EQEP1);		//enable the interrupt in the PIE
//	QEP_clear_all_interrupt_flags(obj->qepHandle);
//
//	CPU_enableInt(obj->cpuHandle, CPU_IntNumber_5);
//
//	EALLOW;
//	EQep1Regs.QCLR.bit.INT = 1;	//clear the global interrupt flag
//	PieCtrlRegs.PIEACK.bit.ACK5 = 1;
//	EDIS;

}
uint32_t QuadratureEncoder::getShiftedTicks()
{
	int shifted = ((uint32_t)QEP_read_posn_count(this->qepHandle) - this->offset);
	if(shifted < 0)
	{
		shifted = ppr + shifted;
	}
	return shifted;
}
uint32_t QuadratureEncoder::getRawTicks()
{
	return QEP_read_posn_count(this->qepHandle);
}
float QuadratureEncoder::getPositionInRadians()
{
	float divided = (float)getShiftedTicks()/ ((float)ppr);
	float final = 2*MATH_PI*divided;
	return final;
}

float QuadratureEncoder::getPositionInDegrees()
{

	float divided = (float)getShiftedTicks()/ ((float)ppr);
	float final = 360*divided;
	return final;
}
float QuadratureEncoder::getVelocityInRadiansPerSecond()
{
	return 0.0;
}
float QuadratureEncoder::getVelocityInDegreesPerSecond()
{
	return 0.0;
}
float QuadratureEncoder::getVelocityInRPM()
{
	return 0.0;
}
void QuadratureEncoder::zero()
{
	this->offset = QEP_read_posn_count(this->qepHandle);
}

