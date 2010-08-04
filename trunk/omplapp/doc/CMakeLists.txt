if(DOXYGEN)
    # add "doc" target if Doxygen is detected
    add_custom_target(doc
		COMMAND ${CMAKE_COMMAND} -E remove_directory html
		COMMAND 
		COMMAND ${DOXYGEN} "Doxyfile"
		WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
	
	# if ruby is installed, we can also regenerate the CSS file from 
	# individual CSS files using the Blueprint framework
	if(RUBY_EXECUTABLE)
		add_custom_target(update_css
			COMMAND ${RUBY_EXECUTABLE} ./blueprint/lib/compress.rb -p ompl
			WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
		# automatically update CSS when regenerating HTML
		add_dependencies(doc update_css)
	endif()
	
endif(DOXYGEN)
