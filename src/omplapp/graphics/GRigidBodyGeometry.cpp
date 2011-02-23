/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include "omplapp/graphics/GRigidBodyGeometry.h"
#include "omplapp/graphics/detail/assimpGUtil.h"

int ompl::app::GRigidBodyGeometry::setRobotMesh(const std::string &robot)
{
    return RigidBodyGeometry::setRobotMesh(robot);
}

int ompl::app::GRigidBodyGeometry::addRobotMesh(const std::string &robot)
{
    int r = RigidBodyGeometry::addRobotMesh(robot);
    if (r)
    {
        std::vector<const aiScene*> scenes;
        for (unsigned int i = 0 ; i < importerRobot_.size() ; ++i)
            scenes.push_back(importerRobot_[i]->GetScene());
        return scene::assimpRender(scenes, getRobotCenter());
    }
    else
        return 0;
}

int ompl::app::GRigidBodyGeometry::setEnvironmentMesh(const std::string &env)
{
    return RigidBodyGeometry::setEnvironmentMesh(env);
}

int ompl::app::GRigidBodyGeometry::addEnvironmentMesh(const std::string &env)
{
    int r = RigidBodyGeometry::addEnvironmentMesh(env);
    if (r)
    {
        std::vector<const aiScene*> scenes;
        for (unsigned int i = 0 ; i < importerEnv_.size() ; ++i)
            scenes.push_back(importerEnv_[i]->GetScene());
        return scene::assimpRender(scenes);
    }
    else
        return 0;
}
