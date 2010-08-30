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
	    SE3RigidBodyPlanning(void) : geometric::SimpleSetup(base::StateManifoldPtr(new base::SE3StateManifold())), factor_(1.0), add_(0.0)
	    {
	    }

	    int setMeshes(const std::string &robot, const std::string &env, bool useOpenGL = false);
	    
	    void setBoundsFactor(double factor)
	    {
		factor_ = factor;
	    }
	    
	    double getBoundsFactor(void)
	    {
		return factor_;
	    }	    

	    void setBoundsAddition(double add)
	    {
		add_ = add;
	    }
	    
	    double getBoundsAddition(void)
	    {
		return add_;
	    }
	    
	    virtual void setup(void);
	    
	protected:

	    double factor_;
	    double add_;
	    
	};	
	
    }
}
