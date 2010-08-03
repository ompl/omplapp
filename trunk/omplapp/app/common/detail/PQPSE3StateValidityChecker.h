#include "PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	class PQPSE3StateValidityChecker : public PQPStateValidityChecker
	{
	public:
	    
	    PQPSE3StateValidityChecker(const base::SpaceInformationPtr &si,
				       const std::vector<const aiMesh*> &robot,
				       const std::vector<const aiMesh*> &obstacles):
		PQPStateValidityChecker(si, robot, obstacles)
	    {
	    }
	    
	    virtual bool isValid(const base::State *state) const;
	    
	};
	
    }
}
