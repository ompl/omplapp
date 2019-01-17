add_subdirectory(omplapp)

# dirty hack to fix up OMPLAPP_RESOURCE_DIR in installed config.h
install(CODE "set(OMPLAPP_RESOURCE_DIR \"${CMAKE_INSTALL_FULL_DATAROOTDIR}/ompl/resources\")
set(OMPL_HAS_PQP ${OMPL_HAS_PQP})
configure_file(\"${CMAKE_CURRENT_SOURCE_DIR}/omplapp/config.h.in\" \"${CMAKE_CURRENT_BINARY_DIR}/omplapp/config.h\")")

install(DIRECTORY omplapp/ "${CMAKE_CURRENT_BINARY_DIR}/omplapp/"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/omplapp"
    COMPONENT omplapp
    FILES_MATCHING PATTERN "*.h")
