configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md.in"
    "${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md")

install(FILES man/ompl_benchmark.1 man/ompl_app.1 man/ompl_webapp.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1 COMPONENT omplapp)
