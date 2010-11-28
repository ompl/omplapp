if (OMPL_BUILD_DEMOS)
    set(OMPL_APP_LIBRARIES ${OMPL_APP_LIBRARIES} ompl ompl_app)

    # create executable for SE3 rigid body planning application
    set(SE3RigidBodyPlanning_SRC SE3RigidBodyPlanning/SE3RigidBodyPlanning.cpp)
    add_executable(SE3RigidBodyPlanning ${SE3RigidBodyPlanning_SRC}) 
    target_link_libraries(SE3RigidBodyPlanning ${OMPL_APP_LIBRARIES})

    # create executable for SE3 rigid body benchmarking application
    set(SE3RigidBodyPlanningBenchmark_SRC SE3RigidBodyPlanning/SE3RigidBodyPlanningBenchmark.cpp)
    add_executable(SE3RigidBodyPlanningBenchmark ${SE3RigidBodyPlanningBenchmark_SRC})
    target_link_libraries(SE3RigidBodyPlanningBenchmark ${OMPL_APP_LIBRARIES})

    # create executable for SE2 rigid body planning application
    aux_source_directory(SE2RigidBodyPlanning SE2RigidBodyPlanning_SRC)
    add_executable(SE2RigidBodyPlanning ${SE2RigidBodyPlanning_SRC}) 
    target_link_libraries(SE2RigidBodyPlanning ${OMPL_APP_LIBRARIES})
endif(OMPL_BUILD_DEMOS)

file(GLOB_RECURSE OMPLAPP_DEMO_CXX_FILES "*.cpp")
file(GLOB_RECURSE OMPLAPP_DEMO_PY_FILES "*.py")
install(FILES ${OMPLAPP_DEMO_CXX_FILES} ${OMPLAPP_DEMO_PY_FILES} DESTINATION ${OMPL_DEMO_INSTALL_DIR})
