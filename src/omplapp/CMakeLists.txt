# create a library with the common parts of applications
file(GLOB OMPLAPP_BASE_SRC geometry/*.cpp geometry/detail/*.cpp apps/*.cpp apps/detail/*.cpp)
file(GLOB OMPLAPP_GUI_SRC apps/graphical/*.cpp graphics/*.cpp graphics/detail/*.cpp)

if(MSVC)
    add_library(ompl_app_base STATIC ${OMPLAPP_BASE_SRC})
    add_library(ompl_app STATIC ${OMPLAPP_GUI_SRC})
else(MSVC)
    add_library(ompl_app_base SHARED ${OMPLAPP_BASE_SRC})
    add_library(ompl_app SHARED ${OMPLAPP_GUI_SRC})
endif(MSVC)

if(TARGET assimp)
    add_dependencies(ompl_app_base assimp)
    add_dependencies(ompl_app assimp)
endif()
if(TARGET fcl)
    add_dependencies(ompl_app_base fcl)
endif()
target_link_libraries(ompl_app_base ompl ${OMPLAPP_LIBRARIES})
target_link_libraries(ompl_app ompl_app_base ompl ${OMPLAPP_LIBRARIES})

if(WIN32)
  if(MSVC10 OR MINGW)
    set(WIN32_STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib glu32.lib opengl32.lib \"${ASSIMP_LIBRARY}\" \"${PQP_LIBRARY}\"")
  else(MSVC10 OR MINGW)
    set(WIN32_STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib glu32.lib opengl32.lib &quot;${ASSIMP_LIBRARY}&quot; &quot;${PQP_LIBRARY}&quot;")
  endif(MSVC10 OR MINGW)
  set_target_properties(ompl_app PROPERTIES VERSION "${OMPLAPP_VERSION}" STATIC_LIBRARY_FLAGS "${WIN32_STATIC_LIBRARY_FLAGS}")
  set_target_properties(ompl_app_base PROPERTIES VERSION "${OMPLAPP_VERSION}" STATIC_LIBRARY_FLAGS "${WIN32_STATIC_LIBRARY_FLAGS}")
  if(MINGW)
    set_target_properties(ompl_app PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
    set_target_properties(ompl_app_base PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
  endif(MINGW)
else(WIN32)
  set_target_properties(ompl_app PROPERTIES VERSION "${OMPLAPP_VERSION}" SOVERSION "${OMPLAPP_ABI_VERSION}")
  set_target_properties(ompl_app_base PROPERTIES VERSION "${OMPLAPP_VERSION}" SOVERSION "${OMPLAPP_ABI_VERSION}")
endif(WIN32)

install(TARGETS ompl_app ompl_app_base
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
    COMPONENT omplapp)
if(NOT MSVC)
    add_custom_command(TARGET ompl_app POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:ompl_app>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../ompl/py-bindings/ompl/app/libompl_app${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
    add_custom_command(TARGET ompl_app_base POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:ompl_app_base>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../ompl/py-bindings/ompl/app/libompl_app_base${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
endif(NOT MSVC)

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/config.h")
