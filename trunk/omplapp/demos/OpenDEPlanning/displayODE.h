#include <ode/ode.h>
#include <vector>

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
