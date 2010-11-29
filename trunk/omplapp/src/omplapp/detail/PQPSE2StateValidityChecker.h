#include "omplapp/detail/PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE2StateManifold.h>

namespace ompl
{
    namespace app
    {
	
	/** \brief Specialization of PQPStateValidityChecker for SE2 (this class assumes states are allocated by ompl::base::SE2StateManifold) */
	class PQPSE2StateValidityChecker : public PQPStateValidityChecker
	{
	public:
	    
	    PQPSE2StateValidityChecker(const base::SpaceInformationPtr &si) : PQPStateValidityChecker(si)
	    {
	    }
	    
	    virtual bool isValid(const base::State *state) const;
	    
	};
	
    }
}
