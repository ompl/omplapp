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

#ifndef OMPLAPP_COMMON_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_COMMON_SE2_RIGID_BODY_PLANNING_

#include "omplapp/graphics/GRigidBodyGeometry.h"
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/geometric/SimpleSetup.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE2RigidBodyPlanning : public geometric::SimpleSetup,
                                     public GRigidBodyGeometry
        {
        public:

            SE2RigidBodyPlanning(void) : geometric::SimpleSetup(base::StateManifoldPtr(new base::SE2StateManifold())), GRigidBodyGeometry(Motion_2D)
            {
            }

            virtual ~SE2RigidBodyPlanning(void)
            {
            }

            const base::StateManifoldPtr& getSE2StateManifold(void)
            {
                return getStateManifold();
            }

            void inferEnvironmentBounds(void);
            void inferProblemDefinitionBounds(void);

            int renderPlannerData(void) const;

            virtual void setup(void);
        };


    }
}

#endif
