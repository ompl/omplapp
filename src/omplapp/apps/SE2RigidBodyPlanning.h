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
#include <ompl/base/spaces/SE2StateSpace.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE2RigidBodyPlanning : public AppBase<AppType::GEOMETRIC>
        {
        public:

            SE2RigidBodyPlanning() : AppBase<AppType::GEOMETRIC>(std::make_shared<base::SE2StateSpace>(), Motion_2D)
            {
                name_ = "Rigid body planning (2D)";
            }

            ~SE2RigidBodyPlanning() override = default;

            bool isSelfCollisionEnabled() const override
            {
                return false;
            }

            base::ScopedState<> getDefaultStartState() const override;

            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override
            {
                return state;
            }

            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace();
            }

            unsigned int getRobotCount() const override
            {
                return 1;
            }

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state;
            }

        };

    }
}

#endif
