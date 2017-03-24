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
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

namespace ompl
{
    namespace app
    {

        static void renderState(const base::SE2StateSpace::StateType &state)
        {
            glVertex3d(state.getX(), state.getY(), 0.0);
        }

        static void renderState(const base::SE3StateSpace::StateType &state)
        {
            glVertex3d(state.getX(), state.getY(), state.getZ());
        }

        static void renderEdge(const base::SE2StateSpace::StateType &state1,
            const base::SE2StateSpace::StateType &state2)
        {
            renderState(state1);
            renderState(state2);
        }

        static void renderEdge(const base::SE3StateSpace::StateType &state1,
            const base::SE3StateSpace::StateType &state2)
        {
            renderState(state1);
            renderState(state2);
        }

        static void setStateColor(int tag)
        {
            static const int NC = 7;
            static float colors[NC][4] =
                {
                    {1.0f, 0.0f, 0.0f, 0.6f},
                    {0.0f, 1.0f, 0.0f, 0.6f},
                    {0.0f, 0.0f, 1.0f, 0.6f},
                    {1.0f, 1.0f, 0.0f, 0.6f},
                    {0.0f, 1.0f, 1.0f, 0.6f},
                    {1.0f, 0.0f, 1.0f, 0.6f},
                    {1.0f, 1.0f, 1.0f, 0.6f},
                };

            int c = abs(tag) % NC;
            glColor4f(colors[c][0], colors[c][1], colors[c][2], colors[c][3]);
        }

        int RenderPlannerData(const base::PlannerData &pd, const aiVector3D &translate, MotionModel m, const GeometricStateExtractor &gse, unsigned int count)
        {
            static int result = -1;

            if (result != -1)
                glDeleteLists(result, 2);
            result = glGenLists(2);

            aiMatrix4x4 t;
            aiMatrix4x4::Translation(-translate, t);
            aiTransposeMatrix4(&t);

            // render vertices
            glNewList(result, GL_COMPILE);
            glPushMatrix();
            glMultMatrixf((float*)&t);
            glDisable(GL_LIGHTING);
            glDisable(GL_COLOR_MATERIAL);
            glPointSize(2.0f);
            glBegin(GL_POINTS);
            if (m == Motion_2D)
                for (std::size_t i = 0; i < pd.numVertices(); ++i)
                {
                    const base::PlannerDataVertex& vtx = pd.getVertex(i);
                    setStateColor(vtx.getTag());
                    for (unsigned int r = 0 ; r < count ; ++r)
                    {
                        const base::State *st = gse(vtx.getState(), r);
                        const auto* se2st = static_cast<const base::SE2StateSpace::StateType*>(st);
                        renderState (*se2st);
                    }
                }
            else
                for (std::size_t i = 0; i < pd.numVertices(); ++i)
                {
                    const base::PlannerDataVertex& vtx = pd.getVertex(i);
                    setStateColor(vtx.getTag());
                    for (unsigned int r = 0 ; r < count ; ++r)
                    {
                        const base::State *st = gse(vtx.getState(), r);
                        const auto* se3st = static_cast<const base::SE3StateSpace::StateType*>(st);
                        renderState (*se3st);
                    }
                }
            glEnd();
            glPopMatrix();
            glEndList();

            // render edges
            glNewList(result+1, GL_COMPILE);
            glPushMatrix();
            glMultMatrixf((float*)&t);
            glDisable(GL_LIGHTING);
            glDisable(GL_COLOR_MATERIAL);
            glPointSize(2.0f);
            glBegin(GL_LINES);
            std::vector<unsigned int> edgeList;
            unsigned int numEdges;
            if (m == Motion_2D)
                for (std::size_t i = 0; i < pd.numVertices(); ++i)
                {
                    const base::PlannerDataVertex& vtx = pd.getVertex(i);
                    const auto* vi =
                        static_cast<const base::SE2StateSpace::StateType*>(gse(vtx.getState(), 0));
                    setStateColor(vtx.getTag());
                    numEdges = pd.getEdges(i, edgeList);
                    for (unsigned int j = 0; j < numEdges; ++j)
                    {
                        const auto* vj =
                            static_cast<const base::SE2StateSpace::StateType*>(gse(pd.getVertex(edgeList[j]).getState(), 0));
                        renderEdge(*vi, *vj);
                    }
                }
            else
                for (std::size_t i = 0; i < pd.numVertices(); ++i)
                {
                    const base::PlannerDataVertex& vtx = pd.getVertex(i);
                    const auto* vi =
                        static_cast<const base::SE3StateSpace::StateType*>(gse(vtx.getState(), 0));
                    setStateColor(vtx.getTag());
                    numEdges = pd.getEdges(i, edgeList);
                    for (unsigned int j = 0; j < numEdges; ++j)
                    {
                        const auto* vj =
                            static_cast<const base::SE3StateSpace::StateType*>(gse(pd.getVertex(edgeList[j]).getState(), 0));
                        renderEdge(*vi, *vj);
                    }
                }
            glEnd();
            glPopMatrix();
            glEndList();

            return result;
        }

    }
}
