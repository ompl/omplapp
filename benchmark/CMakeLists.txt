add_executable(ompl_benchmark
    CFGBenchmark.cpp BenchmarkOptions.cpp BenchmarkTypes.cpp benchmark.cpp)
target_link_libraries(ompl_benchmark ${OMPLAPP_LIBRARIES} ompl ompl_app_base)
install(TARGETS ompl_benchmark
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT omplapp
    RENAME ompl_benchmark)
