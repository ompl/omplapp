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
    if (OPENDE_FOUND AND DRAWSTUFF_FOUND)
        if(APPLE)
            find_package(GLUT)
        endif(APPLE)
        add_definitions(${OPENDE_DEFINITIONS})
        include_directories(${OPENDE_INCLUDE} ${DRAWSTUFF_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

        aux_source_directory(OpenDEPlanning OpenDEPlanning_SRC)
        add_executable(demo_OpenDEPlanning ${OpenDEPlanning_SRC})
        target_link_libraries(demo_OpenDEPlanning ${DRAWSTUFF_LIBRARY} ${OPENDE_LIBRARY} ${OMPLAPP_DEMO_LIBRARIES} ${GLUT_LIBRARIES})
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB_RECURSE OMPLAPP_DEMO_FILES "*/*")
install(FILES ${OMPLAPP_DEMO_FILES}
    DESTINATION ${OMPL_DEMO_INSTALL_DIR}
    COMPONENT omplapp)
