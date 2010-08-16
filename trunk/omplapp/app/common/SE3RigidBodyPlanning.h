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

#ifdef __GCCXML__
	    /// \todo Figure out why we need to do to this to avoid infinite loops in py bindings
	    virtual bool solve(double t) { return geometric::SimpleSetup::solve(t); }
	    virtual void clear(void) { geometric::SimpleSetup::clear(); }
#endif

	    int setMeshes(const std::string &robot, const std::string &env, bool useOpenGL = false);
	    
	    virtual void setup(void);
	};	
	
    }
}
