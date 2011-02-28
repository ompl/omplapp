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

#ifndef OMPLAPP_G_SE3_CONTROL_PLANNING_
#define OMPLAPP_G_SE3_CONTROL_PLANNING_

#include "omplapp/apps/SE3ControlPlanning.h"
#include "omplapp/graphics/RenderGeometry.h"

namespace ompl
{
    namespace app
    {

        class GSE3ControlPlanning : public SE3ControlPlanning,
                                    public RenderGeometry
        {
        public:

            GSE3ControlPlanning(void) : SE3ControlPlanning(),
                                        RenderGeometry(*dynamic_cast<const RigidBodyGeometry*>(this), getGeometricStateExtractor())
            {
            }
            
            virtual ~GSE3ControlPlanning(void)
            {
            }
        };
        
    }
}

#endif
