add_executable(test test.cpp PQPStateValidityChecker.cpp SE3RigidBodyPlanning.cpp) 
target_link_libraries(test ${APP_LINK_LIBS})
