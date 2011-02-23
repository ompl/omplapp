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

#include "omplapp/graphics/detail/RenderPlannerData.h"
#include <ompl/base/manifolds/SE2StateManifold.h>
#include <ompl/base/manifolds/SE3StateManifold.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace ompl
{
    namespace app
    {

        static void renderState(const base::SE2StateManifold::StateType &state)
        {
            glVertex3d(state.getX(), state.getY(), 0.0);
        }

        static void renderState(const base::SE3StateManifold::StateType &state)
        {
            glVertex3d(state.getX(), state.getY(), state.getZ());
        }

        int RenderPlannerData(const base::PlannerData &pd, const aiVector3D &robotCenter, MotionModel m, const GeometricStateExtractor &gse, unsigned int count)
        {
            int result = glGenLists(1);
            glNewList(result, GL_COMPILE);

            aiMatrix4x4 t;
            aiMatrix4x4::Translation(-robotCenter, t);
            aiTransposeMatrix4(&t);
            glPushMatrix();
            glMultMatrixf((float*)&t);

            glDisable(GL_LIGHTING);
            glDisable(GL_COLOR_MATERIAL);

            glColor4f(1.0f, 0.0f, 0.0f, 0.6f);
            glPointSize(2.0f);

            glBegin(GL_POINTS);

            if (m == Motion_2D)
                for (std::size_t i = 0 ; i < pd.states.size() ; ++i)
                    for (unsigned int r = 0 ; r < count ; ++r)
                        renderState(*static_cast<const base::SE2StateManifold::StateType*>(gse(pd.states[i], r)));
            else
                for (std::size_t i = 0 ; i < pd.states.size() ; ++i)
                    for (unsigned int r = 0 ; r < count ; ++r)
                        renderState(*static_cast<const base::SE3StateManifold::StateType*>(gse(pd.states[i], r)));

            glEnd();

            glPopMatrix();

            glEndList();
            return result;
        }

    }
}
