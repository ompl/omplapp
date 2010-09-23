# create a library with the common parts of applications
file(GLOB_RECURSE COMMON_SRC common/*.cpp)

add_library(ompl_app SHARED ${COMMON_SRC})
target_link_libraries(ompl_app ompl ${OMPL_APP_LIBRARIES})
get_target_property(OMPL_APP_LIBRARY_LOC ompl_app LOCATION)
add_custom_command(TARGET ompl_app POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${OMPL_APP_LIBRARY_LOC}" 
    "${CMAKE_CURRENT_SOURCE_DIR}/../ompl/py-bindings/ompl/app/libompl_app${CMAKE_SHARED_LIBRARY_SUFFIX}"
    WORKING_DIRECTORY ${LIBRARY_OUTPUT_PATH})
set(OMPL_APP_LIBRARIES ${OMPL_APP_LIBRARIES} ompl ompl_app)

# create executable for SE3 rigid body planning application
aux_source_directory(SE3RigidBodyPlanning SE3RigidBodyPlanning_SRC)
add_executable(SE3RigidBodyPlanning ${SE3RigidBodyPlanning_SRC}) 
target_link_libraries(SE3RigidBodyPlanning ${OMPL_APP_LIBRARIES})

# create executable for SE2 rigid body planning application
aux_source_directory(SE2RigidBodyPlanning SE2RigidBodyPlanning_SRC)
add_executable(SE2RigidBodyPlanning ${SE2RigidBodyPlanning_SRC}) 
target_link_libraries(SE2RigidBodyPlanning ${OMPL_APP_LIBRARIES})

configure_file("${CMAKE_CURRENT_SOURCE_DIR}/common/config.h.in" 
	"${CMAKE_CURRENT_SOURCE_DIR}/common/config.h")
