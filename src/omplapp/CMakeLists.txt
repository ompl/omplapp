# create a library with the common parts of applications
file(GLOB_RECURSE OMPLAPP_SRC *.cpp)

add_library(ompl_app SHARED ${OMPLAPP_SRC})
if(TARGET assimp)
    add_dependencies(ompl_app assimp)
endif()
if(TARGET pqp)
    add_dependencies(ompl_app pqp)
endif()
target_link_libraries(ompl_app ompl ${OMPLAPP_LIBRARIES})

if(WIN32)
  set_target_properties(ompl_app PROPERTIES VERSION "${OMPLAPP_VERSION}")
else(WIN32)
  set_target_properties(ompl_app PROPERTIES VERSION "${OMPLAPP_VERSION}" SOVERSION "0")
endif(WIN32)

install(TARGETS ompl_app DESTINATION lib)
get_target_property(OMPLAPP_LIBRARY_LOC ompl_app LOCATION)
add_custom_command(TARGET ompl_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPLAPP_LIBRARY_LOC}"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../ompl/py-bindings/ompl/app/libompl_app${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/config.h")
