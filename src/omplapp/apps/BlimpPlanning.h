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
        class BlimpPlanning : public AppBase<AppType::CONTROL>
        {
        public:
            BlimpPlanning()
                : AppBase<AppType::CONTROL>(constructControlSpace(), Motion_3D),
                  odeSolver(std::make_shared<control::ODEBasicSolver<>>(si_, [this](const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot)
                      {
                          ode(q, ctrl, qdot);
                      }))
            {
                name_ = std::string("Blimp");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver,
                    [this](const base::State* state, const control::Control* control, const double duration, base::State* result)
                    {
                        postPropagate(state, control, duration, result);
                    }));
            }
            ~BlimpPlanning() override = default;

            bool isSelfCollisionEnabled() const override
            {
                return false;
            }
            unsigned int getRobotCount() const override
            {
                return 1;
            }
            base::ScopedState<> getDefaultStartState() const override;
            base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const override;
            const base::StateSpacePtr& getGeometricComponentStateSpace() const override
            {
                return getStateSpace()->as<base::CompoundStateSpace>()->getSubspace(0);
            }

            virtual void setDefaultBounds();

        protected:

            const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const override
            {
                return state->as<base::CompoundState>()->components[0];
            }

            void postPropagate(const base::State* state, const control::Control* control, double duration, base::State* result);

            virtual void ode(const control::ODESolver::StateType& q, const control::Control *ctrl, control::ODESolver::StateType& qdot);

            static control::ControlSpacePtr constructControlSpace()
            {
                return std::make_shared<control::RealVectorControlSpace>(constructStateSpace(), 3);
            }
            static base::StateSpacePtr constructStateSpace();

            double timeStep_{1e-2};
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
