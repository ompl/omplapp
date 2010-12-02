#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <vector>

#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif

class DisplayODESpaces
{
public:
    
    void drawGeom (dGeomID g, const dReal *pos, const dReal *R, int show_aabb);
    
    void displaySpace(dSpaceID space);
    void displaySpaces(void);

    void addSpace(dSpaceID space, float r = 0.75, float g = 0.75, float b = 0.75);
    void clear(void);
    
protected:
    
    struct Color
    {
	float r, g, b;
    };
    
    std::vector<dSpaceID> m_spaces;
    std::vector<Color>    m_colors;
    
};
