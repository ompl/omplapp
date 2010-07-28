#include <ompl/base/SpaceInformation.h>
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <PQP.h>
#include <aiMesh.h>
#include <boost/shared_ptr.hpp>
#include <vector>

namespace ompl
{
    namespace app
    {
	
	class PQPStateValidityChecker : public base::StateValidityChecker
	{
	public:
	    
	    PQPStateValidityChecker(const base::SpaceInformationPtr &si, const aiMesh* robot,
				    const std::vector<const aiMesh*> &obstacles);
	    
	    virtual bool isValid(const base::State *state) const;
	    
	protected:
	    
	    /** \brief Shared pointe wrapper for PQP_Model */
	    typedef boost::shared_ptr<PQP_Model> PQPModelPtr;    
	    
	    /** \brief  Convert a quaternion to a 3x3 rotation matrix; based on code from 
		http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm */
	    void quaternionToMatrix(const base::SO3StateManifold::StateType &q, PQP_REAL m[3][3]) const;

	    /** \brief Convert a set of meshes to a PQP model */
	    PQPModelPtr getPQPModelFromMeshes(const std::vector<const aiMesh*> &meshes) const;
	    
	    PQPModelPtr    robot_;
	    PQPModelPtr    environment_;    
	    msg::Interface msg_;
	    
	};
	
    }
}
