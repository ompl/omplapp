#include "PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE2StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	class PQPSE2StateValidityChecker : public PQPStateValidityChecker
	{
	public:
	    
	    PQPSE2StateValidityChecker(const base::SpaceInformationPtr &si,
				       const std::vector<const aiMesh*> &robot,
				       const std::vector<const aiMesh*> &obstacles):
		PQPStateValidityChecker(si, robot, obstacles)
	    {
	    }
	    
	    virtual bool isValid(const base::State *state) const;
	    
	};
	
    }
}
