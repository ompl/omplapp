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

#ifndef OMPLAPP_QUADROTOR_PLANNING_
#define OMPLAPP_QUADROTOR_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ompl
{
    namespace app
    {
        /** \brief A class to facilitate planning for a simple quadrotor model

            The dynamics of the quadrotor are described by the following equations:
            \f{eqnarray*}{
            m\ddot \mathbf{p} &=& -u_0\mathbf{n}-\beta\dot\mathbf{p} -m\mathbf{g},\\
            \mathbf{\alpha} &=& (u_1,u_2,u_3)^T,\f}
            where \f$\mathbf{p}\f$ is the position, \f$\mathbf{n}\f$ is the Z-axis of
            the body frame in world coordinates, \f$\alpha\f$ is the angular
            acceleration, \f$m\f$ is the mass, and \f$\beta\f$ is a damping coefficient.
            The system is controlled through \f$u=(u_0,u_1,u_2,u_3)\f$.
        */
        class QuadrotorPlanning : public AppBase<CONTROL>
        {
        public:
            QuadrotorPlanning()
                : AppBase<CONTROL>(constructControlSpace(), Motion_3D), timeStep_(1e-2), massInv_(1.), beta_(1.), odeSolver(new control::ODEBasicSolver<>(si_, boost::bind(&QuadrotorPlanning::ode, this, _1, _2, _3)))
            {
                name_ = std::string("Quadrotor");
                setDefaultBounds();

                si_->setStatePropagator(control::ODESolver::getStatePropagator(odeSolver, boost::bind(&QuadrotorPlanning::postPropagate, this, _1, _2, _3, _4)));
            }
            ~QuadrotorPlanning()
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
            double getMass()
            {
                return 1./massInv_;
            }
            void setMass(double mass)
            {
                massInv_ = 1./mass;
            }
            double getDampingCoefficient()
            {
                return beta_;
            }
            void setDampingCoefficient(double beta)
            {
                beta_ = beta;
            }
            virtual void setDefaultBounds();

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int /*index*/) const
            {
                return state->as<base::CompoundState>()->components[0];
            }

            virtual void ode(const control::ODESolver::StateType& q, const control::Control* ctrl, control::ODESolver::StateType& qdot);

            virtual void postPropagate(const base::State* state, const control::Control* control, const double duration, base::State* result);

            static control::ControlSpacePtr constructControlSpace(void)
            {
                return control::ControlSpacePtr(new control::RealVectorControlSpace(constructStateSpace(), 4));
            }
            static base::StateSpacePtr constructStateSpace(void);

            double timeStep_;
            double massInv_;
            double beta_;
            control::ODESolverPtr odeSolver;
        };

    }
}

#endif
