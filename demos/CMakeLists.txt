if (OMPL_BUILD_DEMOS)
    set(OMPLAPP_DEMO_LIBRARIES ${OMPLAPP_LIBRARIES} ompl ompl_app_base)

    macro(add_omplapp_demo demo_name)
      add_executable(${ARGV})
      target_link_libraries(${demo_name} ${OMPLAPP_DEMO_LIBRARIES})
    endmacro(add_omplapp_demo)

    add_omplapp_demo(demo_SE3RigidBodyPlanningBenchmark SE3RigidBodyPlanning/SE3RigidBodyPlanningBenchmark.cpp)
    add_omplapp_demo(demo_SE3RigidBodyPlanning SE3RigidBodyPlanning/SE3RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_SE3RigidBodyPlanningWithOptimization SE3RigidBodyPlanning/SE3RigidBodyPlanningWithOptimization.cpp)
    add_omplapp_demo(demo_SE2RigidBodyPlanning SE2RigidBodyPlanning/SE2RigidBodyPlanning.cpp)
    add_omplapp_demo(demo_KinematicCarPlanning SE2RigidBodyPlanning/KinematicCarPlanning.cpp)
    add_omplapp_demo(demo_DynamicCarPlanning SE2RigidBodyPlanning/DynamicCarPlanning.cpp)
    add_omplapp_demo(demo_BlimpPlanning SE3RigidBodyPlanning/BlimpPlanning.cpp)
    add_omplapp_demo(demo_QuadrotorPlanning SE3RigidBodyPlanning/QuadrotorPlanning.cpp)
    add_omplapp_demo(demo_SE2MultiRigidBodyPlanning SE2RigidBodyPlanning/SE2MultiRigidBodyPlanning.cpp)
    add_omplapp_demo(demo_SE3MultiRigidBodyPlanning SE3RigidBodyPlanning/SE3MultiRigidBodyPlanning.cpp)
    add_omplapp_demo(demo_CollisionCheckers SE3RigidBodyPlanning/CollisionCheckers.cpp)
    add_omplapp_demo(demo_AnytimePathShortening AnytimePathShorteningDemo.cpp)

    # create executable for planning with OPENDE
    if (ODE_FOUND AND DRAWSTUFF_FOUND AND GLUT_INCLUDE_DIR AND GLUT_LIBRARIES)
        if(APPLE)
            set_package_properties(GLUT PROPERTIES
                URL "https://www.opengl.org/resources/libraries/glut/"
                PURPOSE "Used for one ODE demo program on macOS.")
            find_package(GLUT)
        endif(APPLE)
        aux_source_directory(OpenDEPlanning OpenDEPlanning_SRC)
        add_executable(demo_OpenDEPlanning ${OpenDEPlanning_SRC})
        target_include_directories(demo_OpenDEPlanning PRIVATE ${ODE_INCLUDE_DIRS} ${DRAWSTUFF_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
        target_link_libraries(demo_OpenDEPlanning ${DRAWSTUFF_LIBRARY} ${ODE_LIBRARIES} ${OMPLAPP_DEMO_LIBRARIES} ${GLUT_LIBRARIES})
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB OMPLAPP_DEMO_PY_FILES "*/*.py")
install_python(PROGRAMS ${OMPLAPP_DEMO_PY_FILES}
  DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
  COMPONENT omplapp)
file(GLOB OMPLAPP_DEMO_CXX_FILES "*/*.cpp")
install(FILES ${OMPLAPP_DEMO_CXX_FILES}
    DESTINATION "${OMPL_DEMO_INSTALL_DIR}"
    COMPONENT omplapp)
