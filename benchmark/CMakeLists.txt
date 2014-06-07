add_executable(ompl_benchmark${OMPL_INSTALL_SUFFIX}
    CFGBenchmark.cpp BenchmarkOptions.cpp GeometricBenchmark.cpp benchmark.cpp)
target_link_libraries(ompl_benchmark${OMPL_INSTALL_SUFFIX} ${OMPLAPP_LIBRARIES} ompl ompl_app)
install(TARGETS ompl_benchmark${OMPL_INSTALL_SUFFIX}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT omplapp
    RENAME ompl_benchmark)
