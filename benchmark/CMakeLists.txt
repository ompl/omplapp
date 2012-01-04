add_executable(benchmark CFGBenchmark.cpp BenchmarkOptions.cpp benchmark.cpp)
target_link_libraries(benchmark ${OMPLAPP_LIBRARIES} ompl ompl_app)
