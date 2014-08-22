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

#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/planners/syclop/Decomposition.h>
#include "omplapp/geometry/RigidBodyGeometry.h"

namespace ompl
{

    namespace app
    {

        void InferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                          unsigned int robotCount, const base::StateSpacePtr &space, MotionModel mtype);
        void InferEnvironmentBounds(const base::StateSpacePtr &space, const RigidBodyGeometry &rbg);

        base::ProjectionEvaluatorPtr allocGeometricStateProjector(const base::StateSpacePtr &space, MotionModel mtype,
                                                                  const base::StateSpacePtr &gspace, const GeometricStateExtractor &se);

        control::DecompositionPtr allocDecomposition(const base::StateSpacePtr &space, MotionModel mtype,
            const base::StateSpacePtr &gspace);

    }

}
