if(DOXYGEN)
    # add "doc" target if Doxygen is detected
    add_custom_target(doc
		COMMAND ${CMAKE_COMMAND} -E remove_directory html
		COMMAND ${DOXYGEN} "Doxyfile"
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
endif(DOXYGEN)
