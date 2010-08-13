#include "SE2RigidBodyPlanning.h"
#include "detail/PQPSE2StateValidityChecker.h"
#include "detail/assimpGL.h"
#include <assimp.hpp>     
#include <aiScene.h>      
#include <aiPostProcess.h>
#include <limits>

namespace ompl
{
    namespace app
    {
	
	static void inferBounds(const base::StateManifoldPtr &m, const aiScene* scene)
	{
	    // infer some bounding box for state manifold if we have meshes and a valid bounding box has not been set
	    if (scene->HasMeshes())
	    {
		base::RealVectorBounds bounds = m->as<base::SE2StateManifold>()->getBounds();
		
		// if bounds are not valid
		if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
		{
		    double minX = std::numeric_limits<double>::infinity();
		    double minY = minX;
		    double maxX = -minX;
		    double maxY = maxX;
		    for (unsigned int j = 0 ; j < scene->mNumMeshes ; ++j)
		    {	    
			const aiMesh *a = scene->mMeshes[j];
			for (unsigned int i = 0 ; i < a->mNumVertices ; ++i)
			{
			    if (minX > a->mVertices[i].x) minX = a->mVertices[i].x;
			    if (maxX < a->mVertices[i].x) maxX = a->mVertices[i].x;
			    if (minY > a->mVertices[i].y) minY = a->mVertices[i].y;
			    if (maxY < a->mVertices[i].y) maxY = a->mVertices[i].y;
			}
		    }
		    double dx = (maxX - minX) * 0.10;
		    double dy = (maxY - minY) * 0.10;
		    bounds.low[0] = minX - dx; bounds.low[1] = minY - dy;
		    bounds.high[0] = maxX + dx; bounds.high[1] = maxY + dy;
		    m->as<base::SE2StateManifold>()->setBounds(bounds);
		}
	    }
	}
	
	static void inferBounds(const base::StateManifoldPtr &m, const base::ProblemDefinitionPtr &pdef)
	{
	    // update the bounds based on start states, if needed
	    base::RealVectorBounds bounds = m->as<base::SE2StateManifold>()->getBounds();
	    
	    std::vector<const base::State*> states;
	    pdef->getInputStates(states);
	    
	    double minX = std::numeric_limits<double>::infinity();
	    double minY = minX;
	    double maxX = -minX;
	    double maxY = maxX;
	    for (unsigned int i = 0 ; i < states.size() ; ++i)
	    {
		double x = states[i]->as<base::SE2StateManifold::StateType>()->getX();
		double y = states[i]->as<base::SE2StateManifold::StateType>()->getY();
		if (minX > x) minX = x;
		if (maxX < x) maxX = x;
		if (minY > y) minY = y;
		if (maxY < y) maxY = y;
	    }
	    double dx = (maxX - minX) * 0.10;
	    double dy = (maxY - minY) * 0.10;
	    
	    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
	    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;
	    
	    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
	    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;
	    
	    m->as<base::SE2StateManifold>()->setBounds(bounds);
	}
    }
}

int ompl::app::SE2RigidBodyPlanning::setMeshes(const std::string &robot, const std::string &env, bool useOpenGL)
{

    // load environment 
    std::vector<const aiMesh*> envMesh;
    Assimp::Importer importerE;
    
    assert(!robot.empty());
    assert(!env.empty());
	const aiScene* envScene = importerE.ReadFile(env.c_str(),
						     aiProcess_Triangulate            |
						     aiProcess_JoinIdenticalVertices  |
						     aiProcess_SortByPType);
	if (envScene)
	{
	    if (envScene->HasMeshes())
	    {
		for (unsigned int i = 0 ; i < envScene->mNumMeshes ; ++i)
		    envMesh.push_back(envScene->mMeshes[i]);
		inferBounds(getStateManifold(), envScene);
	    }
	    else
		msg_.error("There is no mesh specified in the indicated environment resource: %s", env.c_str());
	}
	else
	    msg_.error("Unable to load environment scene: %s", env.c_str());

    // load robot 
    Assimp::Importer importerR;
    const aiScene* robotScene = importerR.ReadFile(robot.c_str(),
						   aiProcess_Triangulate            |
						   aiProcess_JoinIdenticalVertices  |
						   aiProcess_SortByPType);
    std::vector<const aiMesh*> robotMesh;
    if (robotScene)
    {
	if (robotScene->HasMeshes())
	{
	    for (unsigned int i = 0 ; i < robotScene->mNumMeshes ; ++i)
		robotMesh.push_back(robotScene->mMeshes[i]);
	}
	else
	    msg_.error("There is no mesh specified in the indicated robot resource: %s", robot.c_str());
    }
    else
	msg_.error("Unable to load robot scene: %s", robot.c_str());

    // create state validity checker
    if (!robotMesh.empty())
	setStateValidityChecker(base::StateValidityCheckerPtr(new PQPSE2StateValidityChecker(getSpaceInformation(), robotMesh, envMesh)));
	
    return useOpenGL ? assimpRender(robotScene, envScene) : 0;
}

void ompl::app::SE2RigidBodyPlanning::setup(void)
{
    inferBounds(getStateManifold(), getProblemDefinition());
    geometric::SimpleSetup::setup();    
}
