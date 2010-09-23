include(PythonBindingsUtils)

# copy app module __init__.py
execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${CMAKE_CURRENT_SOURCE_DIR}/app/__init__.py"
	"${CMAKE_SOURCE_DIR}/ompl/py-bindings/ompl/app/__init__.py")

if(PY_OMPL_GENERATE)	
	create_module_generation_targets(app ${CMAKE_CURRENT_SOURCE_DIR})
	add_dependencies(update_app_bindings update_geometric_bindings)
endif(PY_OMPL_GENERATE)

if(PY_OMPL_COMPILE)
	create_module_target(app "${CMAKE_CURRENT_SOURCE_DIR}" 
		"${CMAKE_CURRENT_SOURCE_DIR}/../ompl/py-bindings/ompl" ompl_app)
endif(PY_OMPL_COMPILE)
