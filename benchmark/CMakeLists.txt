add_executable(ompl_benchmark${OMPL_INSTALL_SUFFIX}
    CFGBenchmark.cpp BenchmarkOptions.cpp BenchmarkTypes.cpp benchmark.cpp)
target_link_libraries(ompl_benchmark${OMPL_INSTALL_SUFFIX} ${OMPLAPP_LIBRARIES} ompl ompl_app_base)
install(TARGETS ompl_benchmark${OMPL_INSTALL_SUFFIX}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    COMPONENT omplapp
    RENAME ompl_benchmark)
