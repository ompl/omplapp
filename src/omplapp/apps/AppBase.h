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

#ifndef OMPLAPP_APP_BASE_
#define OMPLAPP_APP_BASE_

#include "omplapp/geometry/RigidBodyGeometry.h"
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/SimpleSetup.h>
#include "omplapp/apps/detail/appUtil.h"

namespace ompl
{
    namespace app
    {

        enum AppType
            { GEOMETRIC, CONTROL };

        template<AppType T>
        struct AppTypeSelector
        {
            typedef geometric::SimpleSetup SimpleSetup;
            typedef base::StateSpacePtr    SpaceType;
        };

        template<>
        struct AppTypeSelector<CONTROL>
        {
            typedef control::SimpleSetup     SimpleSetup;
            typedef control::ControlSpacePtr SpaceType;
        };

        template<AppType T>
        class AppBase : public AppTypeSelector<T>::SimpleSetup,
                        public RigidBodyGeometry
        {
        public:
            AppBase(const typename AppTypeSelector<T>::SpaceType &space, MotionModel model) :
                AppTypeSelector<T>::SimpleSetup(space), RigidBodyGeometry(model)
            {
            }

            virtual ~AppBase(void)
            {
            }

            AppType getAppType()
            {
                return GEOMETRIC;
            }
            const std::string& getName()
            {
                return name_;
            }

            virtual bool isSelfCollisionEnabled(void) const = 0;

            virtual base::ScopedState<> getDefaultStartState(void) const = 0;

            virtual base::ScopedState<> getFullStateFromGeometricComponent(const base::ScopedState<> &state) const = 0;

            virtual base::ScopedState<> getGeometricComponentState(const base::ScopedState<> &state, unsigned int index) const
            {
                return base::ScopedState<>(getGeometricComponentStateSpace(), getGeometricComponentStateInternal(state.get(), index));
            }

            virtual const base::StateSpacePtr& getGeometricComponentStateSpace(void) const = 0;

            virtual unsigned int getRobotCount(void) const = 0;

            GeometricStateExtractor getGeometricStateExtractor(void) const
            {
                return boost::bind(&AppBase::getGeometricComponentStateInternal, this, _1, _2);
            }

            virtual void inferEnvironmentBounds(void)
            {
                InferEnvironmentBounds(getGeometricComponentStateSpace(), *static_cast<RigidBodyGeometry*>(this));
            }

            virtual void inferProblemDefinitionBounds(void)
            {
                InferProblemDefinitionBounds(AppTypeSelector<T>::SimpleSetup::getProblemDefinition(), getGeometricStateExtractor(), factor_, add_,
                                             getRobotCount(), getGeometricComponentStateSpace(), mtype_);
            }

            virtual void setup(void)
            {
                inferEnvironmentBounds();

                if (AppTypeSelector<T>::SimpleSetup::getProblemDefinition()->getStartStateCount() == 0)
                {
                    OMPL_INFORM("Adding default start state");
                    AppTypeSelector<T>::SimpleSetup::addStartState(getDefaultStartState());
                }

                inferProblemDefinitionBounds();

                const base::StateValidityCheckerPtr &svc = allocStateValidityChecker(AppTypeSelector<T>::SimpleSetup::si_,
                                                                                     getGeometricStateExtractor(), isSelfCollisionEnabled());
                if (AppTypeSelector<T>::SimpleSetup::si_->getStateValidityChecker() != svc)
                    AppTypeSelector<T>::SimpleSetup::si_->setStateValidityChecker(svc);

                AppTypeSelector<T>::SimpleSetup::getStateSpace()->setup();

                if (!AppTypeSelector<T>::SimpleSetup::getStateSpace()->hasDefaultProjection())
                    AppTypeSelector<T>::SimpleSetup::getStateSpace()->
                        registerDefaultProjection(allocGeometricStateProjector(AppTypeSelector<T>::SimpleSetup::getStateSpace(),
                                                                               mtype_, getGeometricComponentStateSpace(),
                                                                               getGeometricStateExtractor()));

                AppTypeSelector<T>::SimpleSetup::setup();
            }

            /** \brief Convenience function for the omplapp GUI. The objective can be one of:
                "length", "max min clearance", or "mechanical work" */
            void setOptimizationObjectiveAndThreshold(const std::string &objective, double threshold)
            {
                AppTypeSelector<T>::SimpleSetup::setOptimizationObjective(
                    getOptimizationObjective(this->si_, objective, threshold));
            }

            control::DecompositionPtr allocDecomposition()
            {
                return ompl::app::allocDecomposition(AppTypeSelector<T>::SimpleSetup::getStateSpace(),
                    mtype_, getGeometricComponentStateSpace());
            }

        protected:

            virtual const base::State* getGeometricComponentStateInternal(const base::State* state, unsigned int index) const = 0;

            std::string name_;

        };

        template<>
        inline AppType AppBase<CONTROL>::getAppType(void)
        {
            return CONTROL;
        }

    }
}

#endif
