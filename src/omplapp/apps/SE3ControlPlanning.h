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
#include <ompl/base/manifolds/SE3StateManifold.h>
#include <ompl/control/manifolds/RealVectorControlManifold.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for rigid bodies in SE2. */
        class SE3ControlPlanning : public AppBase<CONTROL>
        {
        public:

            SE3ControlPlanning(void) : AppBase<CONTROL>(constructControlManifold(), Motion_3D)
            {
                getControlManifold()->setPropagationFunction(boost::bind(&SE3ControlPlanning::propagateForward, this, _1, _2, _3, _4));
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

            virtual const base::StateManifoldPtr& getGeometricComponentStateManifold(void) const
            {
                return getStateManifold()->as<base::CompoundStateManifold>()->getSubManifold(0);
            }

        private:
            void setDefaultBounds();

            void propagateForward(const base::State *from, const control::Control *ctrl, const double duration, base::State *result);

            static control::ControlManifoldPtr constructControlManifold(void)
            {
                return control::ControlManifoldPtr(new control::RealVectorControlManifold(constructStateManifold(), 4));
            }

            static base::StateManifoldPtr constructStateManifold(void)
            {
                return base::StateManifoldPtr(new base::SE3StateManifold()) + base::StateManifoldPtr(new base::RealVectorStateManifold(1));
            }

        };

    }
}

#endif
