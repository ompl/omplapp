#ifndef OMPLAPP_COMMON_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_RIGID_BODY_PLANNING_

#include <ompl/geometric/SimpleSetup.h>
#include <aiScene.h>
#include <string>

namespace ompl
{
    namespace app
    {
	
	class RigidBodyPlanning : public geometric::SimpleSetup
	{
	public:

	    RigidBodyPlanning(const base::StateManifoldPtr &manifold) : geometric::SimpleSetup(manifold), factor_(1.0), add_(0.0), start_(manifold)
	    {
		start_.random();
	    }

	    virtual ~RigidBodyPlanning()
	    {
	    }

	    const base::ScopedState<>& getEnvStartState(void) const
	    {
		return start_;
	    }
	    
	    void setEnvStartState(const base::ScopedState<> &state)
	    {
		start_ = state;
	    }
	    
	    virtual int setMeshes(const std::string &robot, const std::string &env, bool useOpenGL = false);
	    
	    void setBoundsFactor(double factor)
	    {
		factor_ = factor;
	    }
	    
	    double getBoundsFactor(void) const
	    {
		return factor_;
	    }	    

	    void setBoundsAddition(double add)
	    {
		add_ = add;
	    }
	    
	    double getBoundsAddition(void) const
	    {
		return add_;
	    }
	    
	    virtual void setup(void);
	    
	protected:
	    
	    
	    virtual void inferEnvironmentBounds(const aiScene *scene) = 0;
	    virtual void inferProblemDefinitionBounds(void) = 0;

	    virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const = 0;
	    
	    double              factor_;
	    double              add_;
	    
	    base::ScopedState<> start_;
	    
	};	
	
    }
}

#endif
