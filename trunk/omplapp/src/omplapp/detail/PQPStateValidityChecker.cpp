#include "omplapp/detail/PQPStateValidityChecker.h"
#include "omplapp/detail/assimpUtil.h"

void ompl::app::PQPStateValidityChecker::configure(const aiScene *robot, const aiScene *obstacles, const aiVector3D &center)
{
    environment_ = getPQPModelFromScene(obstacles);
    if (!environment_)
	msg_.inform("Empty environment loaded");
    else
	msg_.inform("Loaded environment model with %d triangles", environment_->num_tris);

    robot_ = getPQPModelFromScene(robot, center);
    if (!robot_)
	throw ompl::Exception("Invalid robot mesh");
    else
	msg_.inform("Loaded robot model with %d triangles", robot_->num_tris);
}

void ompl::app::PQPStateValidityChecker::quaternionToMatrix(const base::SO3StateManifold::StateType &q, PQP_REAL m[3][3]) const
{
    double sqw = q.w*q.w;
    double sqx = q.x*q.x;	
    double sqy = q.y*q.y;
    double sqz = q.z*q.z;
    
    m[0][0] =  sqx - sqy - sqz + sqw;
    m[1][1] = -sqx + sqy - sqz + sqw;	
    m[2][2] = -sqx - sqy + sqz + sqw;
    
    double tmp1 = q.x*q.y;	
    double tmp2 = q.z*q.w;
    m[1][0] = 2.0 * (tmp1 + tmp2);
    m[0][1] = 2.0 * (tmp1 - tmp2);
    tmp1 = q.x*q.z;
    tmp2 = q.y*q.w;
    m[2][0] = 2.0 * (tmp1 - tmp2);
    m[0][2] = 2.0 * (tmp1 + tmp2);
    tmp1 = q.y*q.z;
    tmp2 = q.x*q.w;
    m[2][1] = 2.0 * (tmp1 + tmp2);
    m[1][2] = 2.0 * (tmp1 - tmp2);
}


ompl::app::PQPStateValidityChecker::PQPModelPtr ompl::app::PQPStateValidityChecker::getPQPModelFromScene(const aiScene *scene, const aiVector3D &center) const
{ 
    std::vector<aiVector3D> triangles;
    if (scene)
	scene::extractTriangles(scene, triangles);
    for (unsigned int j = 0 ; j < triangles.size() ; ++j)
	triangles[j] -= center;
    return getPQPModelFromTris(triangles);
}

ompl::app::PQPStateValidityChecker::PQPModelPtr ompl::app::PQPStateValidityChecker::getPQPModelFromScene(const aiScene *scene) const
{	
    std::vector<aiVector3D> triangles;
    if (scene)
	scene::extractTriangles(scene, triangles);
    return getPQPModelFromTris(triangles);
}	    

ompl::app::PQPStateValidityChecker::PQPModelPtr ompl::app::PQPStateValidityChecker::getPQPModelFromTris(const std::vector<aiVector3D> &triangles) const
{
    PQPModelPtr model;
    
    if (triangles.empty())
	return model;
    
    // create the PQP model
    model.reset(new PQP_Model());	
    model->BeginModel();
    int id = 0;
    const int N = triangles.size() / 3;
    for (int j = 0 ; j < N ; ++j)
    {	    
	const aiVector3D &v0 = triangles[j * 3];
	const aiVector3D &v1 = triangles[j * 3 + 1];
	const aiVector3D &v2 = triangles[j * 3 + 2];
	PQP_REAL dV0[3] = {v0.x, v0.y, v0.z};
	PQP_REAL dV1[3] = {v1.x, v1.y, v1.z};
	PQP_REAL dV2[3] = {v2.x, v2.y, v2.z};
	model->AddTri(dV0, dV1, dV2, id++);
    }
    
    model->EndModel();
    
    return model;  
}

