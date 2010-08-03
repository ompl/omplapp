
# create a library with the common parts of applications
SET(COMMON_LIB appcommon)
file(GLOB_RECURSE COMMON_SRC common/*.cpp)

add_library(${COMMON_LIB} ${COMMON_SRC}) 
target_link_libraries(${COMMON_LIB} ${OMPL_APP_LIBRARIES})
set(OMPL_APP_LIBRARIES ${OMPL_APP_LIBRARIES} ${COMMON_LIB})

# create executable for SE3 rigid body planning application
aux_source_directory(SE3RigidBodyPlanning SE3RigidBodyPlanning_SRC)
add_executable(SE3RigidBodyPlanning ${SE3RigidBodyPlanning_SRC}) 
target_link_libraries(SE3RigidBodyPlanning ${OMPL_APP_LIBRARIES})
