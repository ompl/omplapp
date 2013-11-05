add_executable(ompl_benchmark CFGBenchmark.cpp BenchmarkOptions.cpp GeometricBenchmark.cpp benchmark.cpp)
target_link_libraries(ompl_benchmark ${OMPLAPP_LIBRARIES} ompl ompl_app)
install(TARGETS ompl_benchmark
    RUNTIME DESTINATION bin
    COMPONENT omplapp)
