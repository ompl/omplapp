/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2012, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ryan Luna */

#ifndef OMPLAPP_MULTI_RIGID_BODY_PLANNING_
#define OMPLAPP_MULTI_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl
{
    namespace app
    {

        /** \brief Wrapper for ompl::app::RigidBodyPlanning that plans
            for multiple rigid bodies in SE3. */
        class MultiRigidBodyPlanning : public AppBase<GEOMETRIC>
        {
        public:

            // N is the number of robots in SE(3)
            MultiRigidBodyPlanning(unsigned int n) : AppBase<GEOMETRIC>(base::StateSpacePtr(new base::CompoundStateSpace()), Motion_3D), n_(n)
            {
                name_ = "Rigid body planning (n*3D)";
                for (unsigned int i = 0; i < n_; ++i)
                    si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.0);
            }

            virtual ~MultiRigidBodyPlanning(void)
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return true;
            }

            virtual base::ScopedState<> getDefaultStartState(void) const;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
            {
                return state;
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                // Return the zeroth SE3 component.  UNSURE OF THE IMPACT OF THIS.  INVESTIGATE
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(0);
            }

            virtual unsigned int getRobotCount(void) const
            {
                return n_;
            }
            
            virtual void setup(void);

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
            {
                // Add sanity check that index is a subspace of the state
                const base::SE3StateSpace::StateType* st = state->as<base::CompoundStateSpace::StateType>()->
                                                                  as<base::SE3StateSpace::StateType>(index);
                return static_cast<const base::State*>(st);
            }
            
            // Number of robots
            unsigned int n_;

        };

    }
}

#endif
