#include "omplapp/detail/PQPStateValidityChecker.h"
#include <ompl/base/manifolds/SE3StateManifold.h>

namespace ompl
{
    namespace app
    {

	/** \brief Specialization of PQPStateValidityChecker for SE3 (this class assumes states are allocated by ompl::base::SE3StateManifold) */	
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
