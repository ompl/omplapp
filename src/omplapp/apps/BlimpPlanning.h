/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Mark Moll */

#ifndef OMPLAPP_BLIMP_PLANNING_
#define OMPLAPP_BLIMP_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a simple blimp model

            The dynamics of the blimp are described by the following equations:
            \f{eqnarray*}{
            \ddot x &=& u_f\cos\theta,\\
            \ddot y &=& u_f\sin\theta,\\
            \ddot z &=& u_z,\\
            \ddot\theta &=& u_\theta,\f}
            where \f$(x,y,z)\f$ is the position, \f$\theta\f$ the heading, and the
            controls \f$(u_f,u_z,u_\theta)\f$ control their rate of change.
        */
        class BlimpPlanning : public AppBase<CONTROL>
        {
        public:
            BlimpPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&BlimpPlanning::ode, this, _1, _2, _3)))
            {
                name_ = std::string("Blimp");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&BlimpPlanning::postPropagate, this, _1, _2, _3, _4)));
            }
            ~BlimpPlanning()
            {
            }

            bool isSelfCollisionEnabled(void) const
            {
                return false;
            }
            virtual unsigned int getRobotCount(void) const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState(void) const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 3));
            }
            static base::StateSpacePtr constructStateSpace(void);

            double timeStep_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
