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

#include <ompl/base/goals/GoalRegion.h>
#include <ompl/config.h>
#include <ompl/extensions/opende/OpenDESimpleSetup.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

/// @cond IGNORE

// Define the goal we want to reach
class CarGoal : public ob::GoalRegion
{
public:
    CarGoal(const ob::SpaceInformationPtr &si, double x, double y) : ob::GoalRegion(si), x_(x), y_(y)
    {
        threshold_ = 0.25;
    }

    virtual double distanceGoal(const ob::State *st) const
    {
        const double *pos = st->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        const double *vel = st->as<oc::OpenDEStateSpace::StateType>()->getBodyLinearVelocity(0);
        double dx = x_ - pos[0];
        double dy = y_ - pos[1];
        double dot = dx * vel[0] + dy * vel[1];
        if (dot > 0)
            dot = 0;
        else
            dot = sqrt(fabs(dot));

        return sqrt(dx * dx + dy * dy) + dot;
    }

private:
    double x_, y_;
};

// Define how we project a state
class CarStateProjectionEvaluator : public ob::ProjectionEvaluator
{
public:
    CarStateProjectionEvaluator(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
    {
    }

    virtual unsigned int getDimension(void) const
    {
        return 2;
    }

    virtual void defaultCellSizes(void)
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 1.0;
        cellSizes_[1] = 1.0;
    }

    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        const double *pos = state->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        projection[0] = pos[0];
        projection[1] = pos[1];
    }
};

// Define our own space, to include a distance function we want and register a default projection
class CarStateSpace : public oc::OpenDEStateSpace
{
public:
    CarStateSpace(const oc::OpenDEEnvironmentPtr &env) : oc::OpenDEStateSpace(env)
    {
    }

    virtual double distance(const ob::State *s1, const ob::State *s2) const
    {
        const double *p1 = s1->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        const double *p2 = s2->as<oc::OpenDEStateSpace::StateType>()->getBodyPosition(0);
        double dx = fabs(p1[0] - p2[0]);
        double dy = fabs(p1[1] - p2[1]);
        return sqrt(dx * dx + dy * dy);
    }

    virtual void registerProjections(void)
    {
        registerDefaultProjection(ob::ProjectionEvaluatorPtr(new CarStateProjectionEvaluator(this)));
    }
};

class CarControlSampler : public oc::RealVectorControlUniformSampler
{
public:
    CarControlSampler(const oc::ControlSpace *cm) : oc::RealVectorControlUniformSampler(cm)
    {
    }

    virtual void sampleNext(oc::Control *control, const oc::Control *previous)
    {
        space_->copyControl(control, previous);
        const ob::RealVectorBounds &b = space_->as<oc::OpenDEControlSpace>()->getBounds();
        if (rng_.uniform01() > 0.3)
        {
            double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[0];
            static const double DT0 = 0.05;
            v += (rng_.uniformBool() ? 1 : -1) * DT0;
            if (v > b.high[0])
                v = b.high[0] - DT0;
            if (v < b.low[0])
                v = b.low[0] + DT0;
        }
        if (rng_.uniform01() > 0.3)
        {
            double &v = control->as<oc::OpenDEControlSpace::ControlType>()->values[1];
            static const double DT1 = 0.05;
            v += (rng_.uniformBool() ? 1 : -1) * DT1;
            if (v > b.high[1])
                v = b.high[1] - DT1;
            if (v < b.low[1])
                v = b.low[1] + DT1;
        }
    }

    virtual void sampleNext(oc::Control *control, const oc::Control *previous, const ob::State * /*state*/)
    {
        sampleNext(control, previous);
    }
};

class CarControlSpace : public oc::OpenDEControlSpace
{
public:
    CarControlSpace(const ob::StateSpacePtr &m) : oc::OpenDEControlSpace(m)
    {
    }

    virtual oc::ControlSamplerPtr allocControlSampler(void) const
    {
        return oc::ControlSamplerPtr(new CarControlSampler(this));
    }
};

/// @endcond
