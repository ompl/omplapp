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

#ifndef OMPLAPP_GEOMETRY_GEOMETRY_SPECIFICATION_
#define OMPLAPP_GEOMETRY_GEOMETRY_SPECIFICATION_

#include "omplapp/config.h"
#if OMPL_HAS_ASSIMP3
#include <assimp/scene.h>
#else
#include <assimp/aiScene.h>
#endif
#include <vector>
#include <boost/function.hpp>
#include <ompl/base/State.h>

namespace ompl
{
    namespace app
    {

        /// Specify whether bodies are moving in 2D or bodies moving in 3D
        enum MotionModel { Motion_2D, Motion_3D };

        typedef boost::function<const base::State*(const base::State*, unsigned int)> GeometricStateExtractor;

        class GeometrySpecification
        {
        public:

            GeometrySpecification(void)
            {
            }

            std::vector<const aiScene *> robot;
            std::vector<aiVector3D>      robotShift;

            std::vector<const aiScene *> obstacles;
            std::vector<aiVector3D>      obstaclesShift;
        };

    }
}
#endif
