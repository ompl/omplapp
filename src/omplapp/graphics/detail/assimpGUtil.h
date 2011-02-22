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

#ifndef OMPLAPP_GEOMETRY_DETAIL_ASSIMP_G_UTIL_
#define OMPLAPP_GEOMETRY_DETAIL_ASSIMP_G_UTIL_

#include "omplapp/geometry/detail/assimpUtil.h"

namespace ompl
{
    namespace app
    {

        /// @cond IGNORE
        /** \brief Utilitiy functions for processing imported scenes */
        namespace scene
        {

            int assimpRender(const std::vector<const aiScene*> &scenes);
            int assimpRender(const std::vector<const aiScene*> &scenes, const aiVector3D &robotCenter);

        }
        /// @endcond
    }
}
#endif

