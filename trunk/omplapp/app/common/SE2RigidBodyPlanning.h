#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/manifolds/SE2StateManifold.h>
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
	    
	    void setMeshes(const std::string &robot, const std::string &env);
	    
	    virtual void setup(void);
	};	
	
    }
}
