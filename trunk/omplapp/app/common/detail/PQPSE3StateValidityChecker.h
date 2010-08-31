#include "common/detail/PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	class PQPSE3StateValidityChecker : public PQPStateValidityChecker
	{
	public:
	    
	    PQPSE3StateValidityChecker(const base::SpaceInformationPtr &si) : PQPStateValidityChecker(si)
	    {
	    }
	    
	    virtual bool isValid(const base::State *state) const;
	    
	};
	
    }
}
