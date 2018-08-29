#include "pid.h"
#include <stdlib.h>
#include "../Math/clarke.h"
#include "../Math/park.h"
#include "../Math/ipark.h"

typedef struct _TORQUE_CONTROLLER_Obj_
{
	PID_Handle iq;
	PID_Handle id;
	PARK park;
	IPARK ipark;
	CLARKE clark;
	float tDes;
	float thetaE;
	float Vq;
	float Vd;
	float Valpha;
	float Vbeta;
	float vc;
	float va;
	float vb;

} TorqueController_Obj;

typedef struct _TORQUE_CONTROLLER_Obj_ *TorqueControllerHandle;
extern TorqueControllerHandle TorqueController_Constructor(void *pmemory, const size_t numBytes);
static inline void TorqueController_doControl(TorqueControllerHandle handle, float ia, float ib, float thetaE)
{
	TorqueController_Obj *obj = (TorqueController_Obj*) handle;

	obj->park.Alpha = _IQ26(ia);
	obj->park.Beta = _IQ26(ib);
	_iq theta = _IQ26(obj->thetaE);
	_iq  SINE = _IQsin(theta);
	_iq  COS = _IQcos(theta);
	obj->park.Sine = SINE;
	obj->park.Cosine = COS;

	PARK_MACRO(obj->park);

	PID_run_series(obj->iq,obj->tDes,_IQ24toF(obj->park.Qs),0.0,&(obj->Vq));
	PID_run_series(obj->id,0.0,_IQ24toF(obj->park.Ds),0.0,&(obj->Vd));

	obj->ipark.Qs = _IQ26(obj->Vq);
	obj->ipark.Ds = _IQ26(obj->Vd);
	obj->ipark.Sine = SINE;
	obj->ipark.Cosine = COS;
	IPARK_MACRO(obj->ipark);

	obj->Valpha = _IQ24toF(obj->ipark.Alpha);
	obj->Vbeta = _IQ24toF(obj->ipark.Beta);
	int min = MIN(obj->Valpha, obj->Vbeta);
	int max = MAX(obj->Valpha, obj->Vbeta);
	obj->vc =  0.5 * (100 - (max-min));
	obj->va = obj->Valpha + obj->vc;
	obj->vb = obj->Vbeta + obj->vc;

}
static inline float TorqueController_getA(TorqueControllerHandle handle)
{
	TorqueController_Obj *obj = (TorqueController_Obj*) handle;
	return obj->va;
}
static inline float TorqueController_getB(TorqueControllerHandle handle)
{
	TorqueController_Obj *obj = (TorqueController_Obj*) handle;
	return obj->vb;
}
static inline float TorqueController_getC(TorqueControllerHandle handle)
{
	TorqueController_Obj *obj = (TorqueController_Obj*) handle;
	return obj->vc;
}
