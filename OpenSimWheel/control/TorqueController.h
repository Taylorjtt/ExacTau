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

} TorqueController_Obj;

typedef struct _TORQUE_CONTROLLER_Obj_ *TorqueControllerHandle;
extern TorqueControllerHandle TorqueController_Constructor(void *pmemory, const size_t numBytes);
static inline void TorqueController_doControl(TorqueControllerHandle handle, float ia, float ib)
{
	TorqueController_Obj *obj = (TorqueController_Obj*) handle;
	obj->clark.As = _IQ26(ia);
	obj->clark.Bs = _IQ26(ib);
	CLARKE_MACRO(obj->clark);
	obj->park.Alpha = obj->clark.Alpha;
	obj->park.Beta = obj->clark.Beta;

	obj->park.Sine = _IQsin(_IQ26(obj->thetaE));
	obj->park.Cosine = _IQcos(_IQ26(obj->thetaE));

	PARK_MACRO(obj->park);

	PID_run_series(obj->iq,obj->tDes,_IQtoF(obj->park.Qs),0.0,&(obj->Vq));
	PID_run_series(obj->id,0.0,_IQtoF(obj->park.Ds),0.0,&(obj->Vd));

	obj->ipark.Qs = obj->Vq;
	obj->ipark.Ds = obj->Vd;
	obj->ipark.Sine = _IQsin(_IQ26(obj->thetaE));
	obj->ipark.Cosine = _IQcos(_IQ26(obj->thetaE));
	IPARK_MACRO(obj->ipark);

	obj->Valpha = _IQtoF(obj->ipark.Alpha);
	obj->Vbeta = _IQtoF(obj->ipark.Beta);
	int min = MIN(obj->Valpha, obj->Vbeta);
	int max = MAX(obj->Valpha, obj->Vbeta);
	obj->vc = 0.5 * (100 - (max-min));




}
