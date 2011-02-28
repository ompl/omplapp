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
#include "omplapp/geometry/RigidBodyGeometry.h"

namespace ompl
{
    
    namespace app
    {
        
        void InferProblemDefinitionBounds(const base::ProblemDefinitionPtr &pdef, const GeometricStateExtractor &se, double factor, double add,
                                          unsigned int robotCount, const base::StateManifoldPtr &manifold, MotionModel mtype);
        void InferEnvironmentBounds(const base::StateManifoldPtr &manifold, const RigidBodyGeometry &rbg);
    }
    
}
