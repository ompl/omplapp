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

#ifndef OMPLAPP_GRAPHICS_RENDER_GEOMETRY_
#define OMPLAPP_GRAPHICS_RENDER_GEOMETRY_

#include "omplapp/geometry/RigidBodyGeometry.h"
#include <ompl/base/Planner.h>

#include <utility>

namespace ompl
{
    namespace app
    {

        class RenderGeometry
        {
        public:

            /** \brief Constructor expects a state space that can represent a rigid body */
            RenderGeometry(const RigidBodyGeometry &rbg, GeometricStateExtractor se) : rbg_(rbg), se_(std::move(se))
            {
            }

            virtual ~RenderGeometry() = default;

            int renderEnvironment() const;

            int renderRobot() const;

            int renderRobotPart(unsigned int index) const;

            int renderPlannerData(const base::PlannerData &pd) const;

        private:

            const RigidBodyGeometry &rbg_;
            GeometricStateExtractor  se_;

        };

    }
}

#endif
