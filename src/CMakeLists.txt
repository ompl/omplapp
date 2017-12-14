add_subdirectory(omplapp)

# dirty hack to fix up OMPLAPP_RESOURCE_DIR in installed config.h
install(CODE "set(OMPLAPP_RESOURCE_DIR \"${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ompl/resources\")
set(OMPL_HAS_PQP ${OMPL_HAS_PQP})
configure_file(\"${CMAKE_CURRENT_SOURCE_DIR}/omplapp/config.h.in\" \"${CMAKE_CURRENT_SOURCE_DIR}/omplapp/config.h\")")

install(DIRECTORY omplapp/
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/omplapp"
    COMPONENT omplapp
    FILES_MATCHING PATTERN "*.h")
