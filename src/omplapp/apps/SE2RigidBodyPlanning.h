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

#ifndef OMPLAPP_SE2_RIGID_BODY_PLANNING_
#define OMPLAPP_SE2_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/manifolds/SE2StateManifold.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE2RigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:

            SE2RigidBodyPlanning(void) : AppBase<GEOMETRIC>(base::StateManifoldPtr(new base::SE2StateManifold()), Motion_2D)
            {
                name_ = "Rigid body planning (2D)";
            }

            virtual ~SE2RigidBodyPlanning(void)
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            virtual const base::StateManifoldPtr& getGeometricComponentStateManifold(void) const
            {
                return getStateManifold();
            }

            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const
            {
                return state;
            }

            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }

        };

    }
}

#endif
