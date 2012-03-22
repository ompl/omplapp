#ifndef OMPLAPP_SE3_MULTI_RIGID_BODY_PLANNING_
#define OMPLAPP_SE3_MULTI_RIGID_BODY_PLANNING_

#include "omplapp/apps/AppBase.h"
#include <ompl/base/spaces/SE3StateSpace.h>

namespace ompl
{

    namespace app
    {
        class SE3MultiRigidBodyPlanning : public app::AppBase<app::GEOMETRIC>
        {
            public:
                /// @brief Constructs an instance of multiple rigid bodies for 3D geometric planning.  n is the number of independent bodies in SE(3)
                SE3MultiRigidBodyPlanning(unsigned int n) : app::AppBase<app::GEOMETRIC>(base::StateSpacePtr(new base::CompoundStateSpace()), app::Motion_3D), n_(n)
            {
                assert (n > 0);
                name_ = "Rigid body planning (n*3D)";
                // Adding n SE(3) StateSpaces
                for (unsigned int i = 0; i < n_; ++i)
                    si_->getStateSpace()->as<base::CompoundStateSpace>()->addSubSpace(base::StateSpacePtr(new base::SE3StateSpace()), 1.0);
            }

                virtual ~SE3MultiRigidBodyPlanning(void) {}

                bool isSelfCollisionEnabled(void) const 
                {
                    // Make sure that self collision is enabled to avoid inter-rigid body collision
                    return true; 
                }

                /// @brief Constructs the default start state where all robots begin at their geometric center.
                /// If robots are all using the same mesh, this state is not likely to be valid.
                virtual base::ScopedState<> getDefaultStartState(void) const
                {
                    base::ScopedState<> st(getStateSpace());
                    base::CompoundStateSpace::StateType* c_st = st.get()->as<base::CompoundStateSpace::StateType>();
                    for (unsigned int i = 0; i < n_; ++i)
                    {
                        aiVector3D s = getRobotCenter(i);
                        base::SE3StateSpace::StateType* sub = c_st->as<base::SE3StateSpace::StateType>(i);
                        sub->setX(s.x);
                        sub->setY(s.y);
                        sub->setZ(s.z);
                        sub->rotation().setIdentity();
                    }
                    return st;
                }

                virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const
                {
                    // States are composed of only geometric components.  No work needed here.
                    return state;
                }

                /// @brief Returns the state space corresponding for the indexth rigid body
                virtual const base::StateSpacePtr& getGeometricComponentStateSpace(unsigned int index) const
                {
                    return getStateSpace()->as<base::CompoundStateSpace>()->getSubSpace(index);
                }

                virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const
                {
                    // Return the zeroth component.  All components are the same.
                    return getGeometricComponentStateSpace(0);
                }

                virtual unsigned int getRobotCount(void) const 
                {
                    return n_; 
                }

                virtual void inferEnvironmentBounds(void)
                {
                    // Infer bounds for all n SE(3) spaces
                    for (unsigned int i = 0; i < n_; ++i)
                        InferEnvironmentBounds(getGeometricComponentStateSpace(i), *static_cast<RigidBodyGeometry*>(this));
                }

                virtual void inferProblemDefinitionBounds(void)
                {
                    // Make sure that all n SE(3) spaces get the same bounds
                    for (unsigned int i = 0; i < n_; ++i)
                        InferProblemDefinitionBounds(app::AppTypeSelector<app::GEOMETRIC>::SimpleSetup::getProblemDefinition(), getGeometricStateExtractor(), factor_, add_,
                                n_, getGeometricComponentStateSpace(i), mtype_);
                }

            protected:
                /// @brief Returns the SE3 state corresponding to the indexth rigid body in the compound state
                virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const
                {
                    assert (index < n_);
                    const base::SE3StateSpace::StateType* st = state->as<base::CompoundStateSpace::StateType>()->
                        as<base::SE3StateSpace::StateType>(index);
                    return static_cast<const base::State*>(st);
                }

                /// @brief The number of independent rigid bodies to plan for
                unsigned int n_;
        };
    }
}
#endif
