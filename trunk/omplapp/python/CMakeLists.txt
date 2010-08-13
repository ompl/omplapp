execute_process(COMMAND ${CMAKE_COMMAND} -E copy_if_different
	"${CMAKE_CURRENT_SOURCE_DIR}/headers_app.txt"
	"${CMAKE_SOURCE_DIR}/ompl/py-bindings/")
