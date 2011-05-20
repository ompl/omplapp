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

#ifndef OMPLAPP_SE3_CONTROL_PLANNING_
#define OMPLAPP_SE3_CONTROL_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE3ControlPlanning : public AppBase<CONTROL>
        {
        public:

            SE3ControlPlanning(void) : AppBase<CONTROL>(constructControlSpace(), Motion_3D)
            {
                getControlSpace()->setPropagationFunction(boost::bind(&SE3ControlPlanning::propagateForward, this, _1, _2, _3, _4));
                name_ = "Rigid body planning with controls (3D)";
                setDefaultBounds();
            }

            virtual ~SE3ControlPlanning(void)
            {
            }


            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;

            virtual const base::State* getGeometricComponentState(const base::State* state, unsigned int index) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(0);
            }

        private:
            void setDefaultBounds();

            void propagateForward(const base::State *from, const control::Control *ctrl, const double duration, base::State *result);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 4));
            }

            static base::StateSpacePtr constructStateSpace(void)
            {
                return base::StateSpacePtr(new base::SE3StateSpace()) + base::StateSpacePtr(new base::RealVectorStateSpace(1));
            }

        };

    }
}

#endif
