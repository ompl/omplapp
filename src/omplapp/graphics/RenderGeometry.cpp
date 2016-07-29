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

#include "omplapp/graphics/RenderGeometry.h"
#include "omplapp/graphics/detail/assimpGUtil.h"
#include "omplapp/graphics/detail/RenderPlannerData.h"

int ompl::app::RenderGeometry::renderEnvironment() const
{
    const GeometrySpecification &gs = rbg_.getGeometrySpecification();
    return scene::assimpRender(gs.obstacles, gs.obstaclesShift);
}

int ompl::app::RenderGeometry::renderRobot() const
{
    const GeometrySpecification &gs = rbg_.getGeometrySpecification();
    return scene::assimpRender(gs.robot, gs.robotShift);
}

int ompl::app::RenderGeometry::renderRobotPart(unsigned int index) const
{
    const GeometrySpecification &gs = rbg_.getGeometrySpecification();
    if (index >= gs.robot.size())
        return 0;
    return scene::assimpRender(gs.robot[index], gs.robotShift.size() > index ? gs.robotShift[index] : aiVector3D(0.0, 0.0, 0.0));
}

int ompl::app::RenderGeometry::renderPlannerData(const base::PlannerData &pd) const
{
    return RenderPlannerData(pd, aiVector3D(0.0, 0.0, 0.0), rbg_.getMotionModel(), se_, rbg_.getLoadedRobotCount());
}
