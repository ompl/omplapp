add_executable(benchmark BenchmarkBase.cpp benchmark.cpp)
target_link_libraries(benchmark ${OMPLAPP_LIBRARIES} ompl ompl_app)
