#include "PQPStateValidityChecker.h"
#include <ompl/geometric/SimpleSetup.h>
#include <assimp.hpp>     
#include <aiScene.h>      
#include <aiPostProcess.h>

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
	    
	    void setMeshes(const std::string &robot, const std::string &scene)
	    {
		
	    }
	    
	};	
	
    }
}

int main()
{
    return 0;    
}
