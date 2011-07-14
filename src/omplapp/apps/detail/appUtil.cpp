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

#include "omplapp/apps/detail/appUtil.h"
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <limits>

void ompl::app::InferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                             unsigned int robotCount, const base::StateSpacePtr &space, MotionModel mtype)
{
    // update the bounds based on start states, if needed
    base::RealVectorBounds bounds = mtype == Motion_2D ? space->as<base::SE2StateSpace>()->getBounds() : space->as<base::SE3StateSpace>()->getBounds();

    std::vector<const base::State*> states;
    pdef->getInputStates(states);

    double minX = std::numeric_limits<double>::infinity();
    double minY = minX;
    double minZ = minX;
    double maxX = -minX;
    double maxY = maxX;
    double maxZ = maxX;
    for (unsigned int i = 0 ; i < states.size() ; ++i)
    {
        for (unsigned int r = 0 ; r < robotCount ; ++r)
        {
            const base::State *s = se(states[i], r);
            double x = mtype == Motion_2D ? s->as<base::SE2StateSpace::StateType>()->getX() : s->as<base::SE3StateSpace::StateType>()->getX();
            double y = mtype == Motion_2D ? s->as<base::SE2StateSpace::StateType>()->getY() : s->as<base::SE3StateSpace::StateType>()->getY();
            double z = mtype == Motion_2D ? 0.0 : s->as<base::SE3StateSpace::StateType>()->getZ();
            if (minX > x) minX = x;
            if (maxX < x) maxX = x;
            if (minY > y) minY = y;
            if (maxY < y) maxY = y;
            if (minZ > z) minZ = z;
            if (maxZ < z) maxZ = z;
        }
    }
    double dx = (maxX - minX) * (factor - 1.0) + add;
    double dy = (maxY - minY) * (factor - 1.0) + add;
    double dz = (maxZ - minZ) * (factor - 1.0) + add;

    if (bounds.low[0] > minX - dx) bounds.low[0] = minX - dx;
    if (bounds.low[1] > minY - dy) bounds.low[1] = minY - dy;

    if (bounds.high[0] < maxX + dx) bounds.high[0] = maxX + dx;
    if (bounds.high[1] < maxY + dy) bounds.high[1] = maxY + dy;

    if (mtype == Motion_3D)
    {
        if (bounds.high[2] < maxZ + dz) bounds.high[2] = maxZ + dz;
        if (bounds.low[2] > minZ - dz) bounds.low[2] = minZ - dz;

        space->as<base::SE3StateSpace>()->setBounds(bounds);
    }
    else
        space->as<base::SE2StateSpace>()->setBounds(bounds);
}

void ompl::app::InferEnvironmentBounds(const base::StateSpacePtr &space, const RigidBodyGeometry &rbg)
{
    MotionModel mtype = rbg.getMotionModel();

    base::RealVectorBounds bounds = mtype == Motion_2D ? space->as<base::SE2StateSpace>()->getBounds() : space->as<base::SE3StateSpace>()->getBounds();

    // if bounds are not valid
    if (bounds.getVolume() < std::numeric_limits<double>::epsilon())
    {
        if (mtype == Motion_2D)
            space->as<base::SE2StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
        else
            space->as<base::SE3StateSpace>()->setBounds(rbg.inferEnvironmentBounds());
    }
}

namespace ompl
{
    namespace app
    {
        namespace detail
        {
            class GeometricStateProjector2D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector2D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE2StateSpace>()), se_(se)
                {
                }

                virtual unsigned int getDimension(void) const
                {
                    return 2;
                }

                virtual void project(const base::State *state, base::EuclideanProjection &projection) const
                {
                    const base::State *gs = se_(state, 0);
                    projection(0) = gs->as<base::SE2StateSpace::StateType>()->getX();
                    projection(1) = gs->as<base::SE2StateSpace::StateType>()->getY();
                }

                virtual void defaultCellSizes(void)
                {
                    const std::vector<double> &b = gm_->getBounds().getDifference();
                    cellSizes_.resize(2);
                    cellSizes_[0] = b[0] / 20.0;
                    cellSizes_[1] = b[1] / 20.0;
                }

            protected:

                const base::SE2StateSpace *gm_;
                GeometricStateExtractor    se_;

            };

            class GeometricStateProjector3D : public base::ProjectionEvaluator
            {
            public:

                GeometricStateProjector3D(const base::StateSpacePtr &space, const base::StateSpacePtr &gspace, const GeometricStateExtractor &se) : base::ProjectionEvaluator(space), gm_(gspace->as<base::SE3StateSpace>()), se_(se)
                {
                }

                virtual unsigned int getDimension(void) const
                {
                    return 3;
                }

                virtual void project(const base::State *state, base::EuclideanProjection &projection) const
                {
                    const base::State *gs = se_(state, 0);
                    projection(0) = gs->as<base::SE3StateSpace::StateType>()->getX();
                    projection(1) = gs->as<base::SE3StateSpace::StateType>()->getY();
                    projection(2) = gs->as<base::SE3StateSpace::StateType>()->getZ();
                }

                virtual void defaultCellSizes(void)
                {
                    const std::vector<double> &b = gm_->getBounds().getDifference();
                    cellSizes_.resize(3);
                    cellSizes_[0] = b[0] / 20.0;
                    cellSizes_[1] = b[1] / 20.0;
                    cellSizes_[2] = b[2] / 20.0;
                }

            protected:

                const base::SE3StateSpace *gm_;
                GeometricStateExtractor    se_;
            };
        }
    }
}

ompl::base::ProjectionEvaluatorPtr ompl::app::allocGeometricStateProjector(const base::StateSpacePtr &space, MotionModel mtype,
                                                                           const base::StateSpacePtr &gspace, const GeometricStateExtractor &se)
{
    if (mtype == Motion_2D)
        return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector2D(space, gspace, se));
    return base::ProjectionEvaluatorPtr(new detail::GeometricStateProjector3D(space, gspace, se));
}
