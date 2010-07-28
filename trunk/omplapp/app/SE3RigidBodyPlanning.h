#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <string>

namespace ompl
{
    namespace app
    {
	
	class SE3RigidBodyPlanning : public geometric::SimpleSetup
	{
	public:
	    SE3RigidBodyPlanning(void) : geometric::SimpleSetup(base::StateManifoldPtr(new base::SE3StateManifold()))
	    {
	    }
	    
	    void setMeshes(const std::string &robot, const std::string &env);
	    
	    virtual void setup(void);
	};	
	
    }
}
