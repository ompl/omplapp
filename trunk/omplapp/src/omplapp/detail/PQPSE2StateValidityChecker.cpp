#include "omplapp/detail/PQPSE2StateValidityChecker.h"
#include <cmath>

bool ompl::app::PQPSE2StateValidityChecker::isValid(const base::State *state) const
{
    if (!environment_)
	return true;
    
    const base::SE2StateManifold::StateType *s = state->as<base::SE2StateManifold::StateType>();
    const double ca = cos(s->getYaw());
    const double sa = sin(s->getYaw());
    
    PQP_REAL robTrans[3] = {s->getX(), s->getY(), 0.0};
    PQP_REAL robRot[3][3] = { { ca, -sa, 0.0}, {sa, ca, 0.0}, {0.0, 0.0, 1.0} };
    
    PQP_CollideResult cr;
    
    static PQP_REAL identityTranslation[3] = { 0.0, 0.0, 0.0 };
    static PQP_REAL identityRotation[3][3] = { { 1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
    
    PQP_Collide(&cr, robRot,  robTrans, robot_.get(),
		identityRotation, identityTranslation, environment_.get(), PQP_FIRST_CONTACT);
    
    return !cr.Colliding();
}
