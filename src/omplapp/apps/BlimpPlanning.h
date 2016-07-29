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
                : AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), odeSolver(new control::ODEBasicSolver<>(si_, std::bind(&BlimpPlanning::ode, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3)))
            {
                name_ = std::string("Blimp");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, std::bind(&BlimpPlanning::postPropagate, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4)));
            }
            ~BlimpPlanning()
            {
            }

            bool isSelfCollisionEnabled() const
            {
                return false;
            }
            virtual unsigned int getRobotCount() const
            {
                return 1;
            }
            virtual base::ScopedState<> getDefaultStartState() const;
            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const;
            virtual const base::StateSpacePtr& getGeometricComponentStateSpace() const
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

            static control::ControlSpacePtr constructControlSpace()
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 3));
            }
            static base::StateSpacePtr constructStateSpace();

            double timeStep_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
