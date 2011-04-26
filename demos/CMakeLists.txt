if (OMPL_BUILD_DEMOS)
    set(OMPLAPP_DEMO_LIBRARIES ${OMPLAPP_LIBRARIES} ompl ompl_app)

    # create executable for SE3 rigid body planning application
    add_executable(SE3RigidBodyPlanning SE3RigidBodyPlanning/SE3RigidBodyPlanning.cpp)
    target_link_libraries(SE3RigidBodyPlanning ${OMPLAPP_DEMO_LIBRARIES})

    # create executable for SE3 planning with controls application
    add_executable(SE3ControlPlanning SE3RigidBodyPlanning/SE3ControlPlanning.cpp)
    target_link_libraries(SE3ControlPlanning ${OMPLAPP_DEMO_LIBRARIES})

    # create executable for SE3 rigid body benchmarking application
    add_executable(SE3RigidBodyPlanningBenchmark SE3RigidBodyPlanning/SE3RigidBodyPlanningBenchmark.cpp)
    target_link_libraries(SE3RigidBodyPlanningBenchmark ${OMPLAPP_DEMO_LIBRARIES})

    # create executable for SE2 rigid body planning application
    add_executable(SE2RigidBodyPlanning SE2RigidBodyPlanning/SE2RigidBodyPlanning.cpp)
    target_link_libraries(SE2RigidBodyPlanning ${OMPLAPP_DEMO_LIBRARIES})

    # create executable for kinematic car planning application
    add_executable(KinematicCarPlanning SE2RigidBodyPlanning/KinematicCarPlanning.cpp)
    target_link_libraries(KinematicCarPlanning ${OMPLAPP_DEMO_LIBRARIES})

    # create executable for planning with ODE
    if (ODE_FOUND AND DRAWSTUFF_FOUND)
        if(APPLE)
            find_package(GLUT)
        endif(APPLE)
        add_definitions(${ODE_DEFINITIONS})
        include_directories(${ODE_INCLUDE} ${DRAWSTUFF_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})

        aux_source_directory(OpenDEPlanning OpenDEPlanning_SRC)
        add_executable(OpenDEPlanning ${OpenDEPlanning_SRC})
        target_link_libraries(OpenDEPlanning ${DRAWSTUFF_LIBRARY} ${ODE_LIBRARY} ${OMPLAPP_DEMO_LIBRARIES} ${GLUT_LIBRARIES})
    endif()
endif(OMPL_BUILD_DEMOS)

file(GLOB_RECURSE OMPLAPP_DEMO_CXX_FILES "*.cpp")
file(GLOB_RECURSE OMPLAPP_DEMO_PY_FILES "*.py")
install(FILES ${OMPLAPP_DEMO_CXX_FILES} ${OMPLAPP_DEMO_PY_FILES} DESTINATION ${OMPL_DEMO_INSTALL_DIR})
