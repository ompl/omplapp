if(OMPL_BUILD_PYBINDINGS)
    install_python(PROGRAMS ompl_app.py
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT omplapp
        RENAME ompl_app)
endif()