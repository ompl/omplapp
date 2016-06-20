include(PythonBindingsUtils)

# copy app module __init__.py
execute_process(COMMAND "${CMAKE_COMMAND}" -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/app/__init__.py"
    "${CMAKE_SOURCE_DIR}/ompl/py-bindings/ompl/app/__init__.py")

if(OMPL_BUILD_PYBINDINGS)
    if(PY_OMPL_GENERATE)
        create_module_generation_targets(app)
        add_dependencies(update_app_bindings update_geometric_bindings update_control_bindings)
    endif(PY_OMPL_GENERATE)

    if(PY_OMPL_COMPILE AND EXISTS "${CMAKE_CURRENT_BINARY_DIR}/bindings")
        create_module_target(app
            "${CMAKE_CURRENT_SOURCE_DIR}/../ompl/py-bindings/ompl" ompl_app)
    endif()
endif(OMPL_BUILD_PYBINDINGS)

add_custom_target(clean_app_bindings
    "${CMAKE_COMMAND}" -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/bindings"
    COMMAND "${CMAKE_COMMAND}" -E remove -f pyplusplus_app.{cache,log}
    WORKING_DIRECTORY "${PROJECT_BINARY_DIR}")
add_dependencies(clean_bindings clean_app_bindings)
