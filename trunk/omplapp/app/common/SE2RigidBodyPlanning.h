#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>
#include <string>

namespace ompl
{
    namespace app
    {
	
	class SE2RigidBodyPlanning : public geometric::SimpleSetup
	{
	public:
	    SE2RigidBodyPlanning(void) : geometric::SimpleSetup(base::StateManifoldPtr(new base::SE2StateManifold()))
	    {
	    }
	    
	    int setMeshes(const std::string &robot, const std::string &env, bool useOpenGL=false);
	    
	    virtual void setup(void);
	};

	class SE2RigidBodyPlanningWithControls : public control::SimpleSetup
	{
	public:
	    SE2RigidBodyPlanningWithControls(void) : control::SimpleSetup(control::ControlManifoldPtr(new control::RealVectorControlManifold
												      (base::StateManifoldPtr(new base::SE2StateManifold()), 2)))
	    {
	    }
	    
	};	
	
    }
}
