/*
 * TorqueController.c
 *
 *  Created on: May 5, 2018
 *      Author: John
 */



#include "TorqueController.h"

TorqueControllerHandle TorqueController_Constructor(void *pmemory, const size_t numBytes)
{
	TorqueControllerHandle handle;
	TorqueController_Obj *obj;
	if(numBytes < sizeof(TorqueController_Obj))
	{
		return ((TorqueControllerHandle)NULL);
	}

	handle = (TorqueControllerHandle)pmemory;
	obj = (TorqueController_Obj*) handle;

	obj->clark = (CLARKE) {0,0,0,0,0};
	obj->park = (PARK) {0,0,0,0,0,0,0};
	obj->ipark = (IPARK){0,0,0,0,0,0,0};
	obj->Vq = 0.0;
	obj->Vd = 0.0;
	obj->Valpha = 0.0;
	obj->Vbeta = 0.0;
	obj->vc = 0.0;


	/*
	 * set up PI Controllers
	 */
	obj->iq = malloc(sizeof(PID_Obj));
	obj->iq = PID_init((void *)obj->iq, sizeof(PID_Obj));
	PID_setGains(obj->iq,0,0,0);
	PID_setFbackValue(obj->iq,0.0);
	PID_setMinMax(obj->iq,-100.0,100.0);
	PID_setUi(obj->iq,0.0);
	PID_setUIMinMax(obj->iq,-10.0,10.0);
	PID_setDerFilterParams(obj->iq,0.0,0.0,0.0,0.0,0.0);

	obj->id = malloc(sizeof(PID_Obj));
	obj->id = PID_init((void *)obj->id, sizeof(PID_Obj));
	PID_setGains(obj->id,0,0,0);
	PID_setFbackValue(obj->id,0.0);
	PID_setMinMax(obj->id,-100.0,100.0);
	PID_setUi(obj->id,0.0);
	PID_setUIMinMax(obj->id,-10.0,10.0);
	PID_setDerFilterParams(obj->id,0.0,0.0,0.0,0.0,0.0);

	obj->tDes = 0.0;
	obj->thetaE = 0.0;
	return handle;


}
