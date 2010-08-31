#include "common/RigidBodyPlanning.h"
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <string>

namespace ompl
{
    namespace app
    {
	
	class SE2RigidBodyPlanning : public RigidBodyPlanning
	{
	public:

	    SE2RigidBodyPlanning(void) : RigidBodyPlanning(base::StateManifoldPtr(new base::SE2StateManifold()))
	    {
	    }
	    
	protected:

	    virtual void inferEnvironmentBounds(const aiScene *scene);
	    virtual void inferProblemDefinitionBounds(void);

	    virtual base::StateValidityCheckerPtr allocStateValidityChecker(const aiScene *env, const aiScene *robot) const;
	    
	};
	
	
    }
}
