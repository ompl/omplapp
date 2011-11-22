add_executable(benchmark benchmark.cpp)
target_link_libraries(benchmark ${OMPLAPP_LIBRARIES} ompl ompl_app)
