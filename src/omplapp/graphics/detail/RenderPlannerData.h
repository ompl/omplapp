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

#ifndef OMPLAPP_RENDER_PLANNER_DATA_
#define OMPLAPP_RENDER_PLANNER_DATA_

#include "omplapp/geometry/GeometrySpecification.h"
#include "omplapp/geometry/detail/assimpUtil.h"
#include <ompl/base/Planner.h>

namespace ompl
{
    namespace app
    {

        /** \brief Render the planner states in \e pd, after shifting them by \e translate, using the motion model \e m.
            The SE2 (or SE3) states can be extracted from \e pd using \e gse. There are \e robotCount points to extract from each state.
            Return a gl list. */
        int RenderPlannerData(const base::PlannerData &pd, const aiVector3D &translate,
                              MotionModel m, const GeometricStateExtractor &gse, unsigned int count);
    }
}

#endif
