# create a library with the common parts of applications
file(GLOB_RECURSE COMMON_SRC common/*.cpp)

add_library(${OMPL_APP_LIBRARY} SHARED ${COMMON_SRC})
target_link_libraries(${OMPL_APP_LIBRARY} ompl ${OMPL_APP_LIBRARIES})
get_target_property(OMPL_APP_LIBRARY_LOC ${OMPL_APP_LIBRARY} LOCATION)
add_custom_command(TARGET ${OMPL_APP_LIBRARY} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_APP_LIBRARY_LOC}" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../ompl/py-bindings/ompl/lib${OMPL_APP_LIBRARY}${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
set(OMPL_APP_LIBRARIES ${OMPL_APP_LIBRARIES} ompl ${OMPL_APP_LIBRARY})

# create executable for SE3 rigid body planning application
aux_source_directory(SE3RigidBodyPlanning SE3RigidBodyPlanning_SRC)
add_executable(SE3RigidBodyPlanning ${SE3RigidBodyPlanning_SRC}) 
target_link_libraries(SE3RigidBodyPlanning ${OMPL_APP_LIBRARIES})

# create executable for SE2 rigid body planning application
aux_source_directory(SE2RigidBodyPlanning SE2RigidBodyPlanning_SRC)
add_executable(SE2RigidBodyPlanning ${SE2RigidBodyPlanning_SRC}) 
target_link_libraries(SE2RigidBodyPlanning ${OMPL_APP_LIBRARIES})

# copy file (if needed) for generation of python bindings
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${CMAKE_CURRENT_SOURCE_DIR}/headers_app.txt"
	"${CMAKE_SOURCE_DIR}/ompl/py-bindings/")

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/common/config.h.in" 
	"${CMAKE_CURRENT_SOURCE_DIR}/common/config.h")
