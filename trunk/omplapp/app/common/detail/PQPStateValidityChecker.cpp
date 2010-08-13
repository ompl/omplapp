#include "PQPStateValidityChecker.h"

ompl::app::PQPStateValidityChecker::PQPStateValidityChecker(const base::SpaceInformationPtr &si, 
							    const std::vector<const aiMesh*> &robot,
							    const std::vector<const aiMesh*> &obstacles) : base::StateValidityChecker(si)
{
    environment_ = getPQPModelFromMeshes(obstacles);
    if (!environment_)
	msg_.inform("Empty environment loaded");
    else
	msg_.inform("Loaded environment model with %d triangles", environment_->num_tris);
    robot_ = getPQPModelFromMeshes(robot);
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


ompl::app::PQPStateValidityChecker::PQPModelPtr ompl::app::PQPStateValidityChecker::getPQPModelFromMeshes(const std::vector<const aiMesh*> &meshes) const
{	
    PQPModelPtr model;
    
    if (meshes.empty())
	return model;
    
    // make sure we can create a model
    for (unsigned int j = 0 ; j < meshes.size() ; ++j)
    {
	const aiMesh *a = meshes[j];
	if (!a->HasFaces())
	{
	    msg_.error("Mesh asset has no faces");
	    return model;
	}
	
	if (!a->HasPositions())
	{
	    msg_.error("Mesh asset has no positions");
	    return model;
	}
	
	unsigned int good = 0;
	for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
	    if (a->mFaces[i].mNumIndices != 3)
		msg_.warn("Asset is not a triangle mesh: face %d has %d vertices", i, a->mFaces[i].mNumIndices);
	    else
		good++;
	if (good == 0)
	{
	    msg_.error("Insufficient faces in mesh");
	    return model;
	}
    }
    
    // create the PQP model
    model.reset(new PQP_Model());	
    model->BeginModel();
    int id = 0;
    for (unsigned int j = 0 ; j < meshes.size() ; ++j)
    {	    
	const aiMesh *a = meshes[j];
	for (unsigned int i = 0 ; i < a->mNumFaces ; ++i)
	{
	    if (a->mFaces[i].mNumIndices != 3)
		continue;
	    const aiVector3D &v0 = a->mVertices[a->mFaces[i].mIndices[0]];
	    const aiVector3D &v1 = a->mVertices[a->mFaces[i].mIndices[1]];
	    const aiVector3D &v2 = a->mVertices[a->mFaces[i].mIndices[2]];
	    PQP_REAL dV0[3] = {v0.x, v0.y, v0.z};
	    PQP_REAL dV1[3] = {v1.x, v1.y, v1.z};
	    PQP_REAL dV2[3] = {v2.x, v2.y, v2.z};
	    model->AddTri(dV0, dV1, dV2, id++);
	}	    
    }
    
    model->EndModel();
    
    return model;	
}	    
