#ifndef OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_SE3_RIGID_BODY_PLANNING_

#include "omplapp/RigidBodyPlanning.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {

	/** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
	    for rigid bodies in SE2. */	
	class SE3RigidBodyPlanning : public RigidBodyPlanning
	{
	public:

	    SE3RigidBodyPlanning(void) : RigidBodyPlanning(base::StateManifoldPtr(new base::SE3StateManifold()))
	    {
	    }

	    virtual ~SE3RigidBodyPlanning(void)
	    {
	    }

	protected:
	    	    
	    virtual void inferEnvironmentBounds(const aiScene *scene);
	    virtual void inferProblemDefinitionBounds(void);
	    virtual void getRobotCenterAndStartState(const aiScene *robot);

	    virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const;
	};	
	
    }
}

#endif
