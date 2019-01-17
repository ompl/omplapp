# create a library with the common parts of applications
file(GLOB ompl_app_base_SRC geometry/*.cpp geometry/detail/*.cpp apps/*.cpp apps/detail/*.cpp)
file(GLOB ompl_app_SRC apps/graphical/*.cpp graphics/*.cpp graphics/detail/*.cpp)

file(GLOB ompl_app_base_HDRS geometry/*.h geometry/detail/*.h apps/*.h apps/detail/*.h)
file(GLOB ompl_app_HDRS apps/graphical/*.h graphics/*.h graphics/detail/*.h)
source_group("OMPL.app Source" FILES "${ompl_app_base_SRC};${ompl_app_SRC}")
source_group("OMPL.app Headers" FILES "${ompl_app_base_HDRS};${ompl_app_HDRS}")


# the ompl_app_base library can be built without OpenGL, which means that the
# ompl_benchmark tool can be built on machines that do not have OpenGL installed.
if(OPENGL_FOUND)
    set(OMPLAPP_TARGETS ompl_app_base ompl_app)
else()
    set(OMPLAPP_TARGETS ompl_app_base)
endif()

foreach(_target ${OMPLAPP_TARGETS})
    if(MSVC)
        add_library(${_target} STATIC ${${_target}_SRC})
    else(MSVC)
        add_library(${_target} SHARED ${${_target}_SRC})
    endif(MSVC)

    target_link_libraries(${_target} ompl ${OMPLAPP_LIBRARIES})
    if(_target STREQUAL ompl_app)
        target_compile_definitions(ompl_app PRIVATE -DGL_SILENCE_DEPRECATION)
        target_link_libraries(ompl_app ompl_app_base)
    endif()

    if(WIN32)
        set_target_properties(${_target} PROPERTIES VERSION "${PROJECT_VERSION}" STATIC_LIBRARY_FLAGS "psapi.lib ws2_32.lib glu32.lib opengl32.lib")
        if(MINGW)
            set_target_properties(${_target} PROPERTIES LINK_FLAGS "-Wl,--export-all-symbols")
        endif(MINGW)
    else(WIN32)
        set_target_properties(${_target} PROPERTIES VERSION "${PROJECT_VERSION}" SOVERSION "${OMPL_ABI_VERSION}")
    endif(WIN32)

    install(TARGETS ${_target}
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
        COMPONENT omplapp)
    if(NOT MSVC)
        add_custom_command(TARGET ${_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "$<TARGET_FILE:${_target}>"
        "${CMAKE_CURRENT_SOURCE_DIR}/../../ompl/py-bindings/ompl/app/lib${_target}${CMAKE_SHARED_LIBRARY_SUFFIX}"
        WORKING_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
    endif(NOT MSVC)
endforeach()

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/config.h.in"
    "${CMAKE_CURRENT_BINARY_DIR}/config.h")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/config.h"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/omplapp"
    COMPONENT omplapp)
