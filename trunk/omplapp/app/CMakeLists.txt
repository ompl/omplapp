# create a library with the common parts of applications
file(GLOB_RECURSE COMMON_SRC common/*.cpp)

add_shared_and_static_library(${OMPL_APP_LIB} ${COMMON_SRC})
target_link_libraries(${OMPL_APP_LIB} ${OMPL_APP_LIBRARIES})
set(OMPL_APP_LIBRARIES ${OMPL_APP_LIBRARIES} ${OMPL_APP_LIB})

# create executable for SE3 rigid body planning application
aux_source_directory(SE3RigidBodyPlanning SE3RigidBodyPlanning_SRC)
add_executable(SE3RigidBodyPlanning ${SE3RigidBodyPlanning_SRC}) 
target_link_libraries(SE3RigidBodyPlanning ${OMPL_APP_LIBRARIES})

# create executable for SE2 rigid body planning application
aux_source_directory(SE2RigidBodyPlanning SE2RigidBodyPlanning_SRC)
add_executable(SE2RigidBodyPlanning ${SE2RigidBodyPlanning_SRC}) 
target_link_libraries(SE2RigidBodyPlanning ${OMPL_APP_LIBRARIES})
