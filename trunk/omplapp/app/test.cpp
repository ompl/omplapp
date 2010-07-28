#include "SE3RigidBodyPlanning.h"

using namespace ompl;

int main()
{
    app::SE3RigidBodyPlanning setup;
    setup.setMeshes("sphere.stl", "");
    setup.solve();
    
    return 0;    
}
