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

#include <drawstuff/drawstuff.h>
#include <ode/ode.h>
#include <map>
#include <vector>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

/// @cond IGNORE
class DisplayOpenDESpaces
{
public:
    DisplayOpenDESpaces(void)
    {
        m_activeColor.r = m_activeColor.g = m_activeColor.b = 0.5;
    }

    void drawGeom(dGeomID g, const dReal *pos, const dReal *R, int show_aabb);

    void displaySpace(dSpaceID space);
    void displaySpaces(void);

    void addSpace(dSpaceID space, float r = 0.75, float g = 0.75, float b = 0.75);
    void clear(void);

    void setGeomColor(dGeomID geom, float r, float g, float b);

protected:
    struct Color
    {
        float r, g, b;
    };

    std::vector<dSpaceID> m_spaces;
    std::vector<Color> m_colors;
    std::map<dGeomID, Color> m_gcolors;

    Color m_activeColor;
};
/// @endcond
