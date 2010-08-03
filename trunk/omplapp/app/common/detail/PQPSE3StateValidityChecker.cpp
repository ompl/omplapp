#include "PQPSE3StateValidityChecker.h"

bool ompl::app::PQPSE3StateValidityChecker::isValid(const base::State *state) const
{
    if (!environment_)
	return true;
    
    const base::SE3StateManifold::StateType *s = state->as<base::SE3StateManifold::StateType>();
    PQP_REAL robTrans[3] = {s->getX(), s->getY(), s->getZ()};
    PQP_REAL robRot[3][3];
    quaternionToMatrix(state->as<base::SE3StateManifold::StateType>()->getRotation(), robRot);
    
    PQP_CollideResult cr;
    
    static PQP_REAL identityTranslation[3] = { 0.0, 0.0, 0.0 };
    static PQP_REAL identityRotation[3][3] = { { 1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0} };
    
    PQP_Collide(&cr, robRot,  robTrans, robot_.get(),
		identityRotation, identityTranslation, environment_.get(), PQP_FIRST_CONTACT);
    
    return cr.Colliding() != 0;
}
