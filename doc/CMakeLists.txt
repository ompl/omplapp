install(FILES man/ompl_benchmark.1 man/ompl_app.1 man/ompl_webapp.1
    DESTINATION ${CMAKE_INSTALL_MANDIR}/man1 COMPONENT omplapp)

if(DOXYGEN_FOUND AND CMAKE_VERSION VERSION_GREATER_EQUAL 3.9)
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/markdown/download.md.in"
        "${CMAKE_BINARY_DIR}/doc/omplapp_download.md")

    set(DOXYGEN_EXCLUDE
        "${CMAKE_SOURCE_DIR}/ompl/doc/markdown/installation.md"
        "${CMAKE_BINARY_DIR}/doc/ompl_download.md"
        "${CMAKE_SOURCE_DIR}/ompl/doc/markdown/license.md"
        "${CMAKE_SOURCE_DIR}/ompl/doc/markdown/releaseNotes.md")
    set(DOXYGEN_EXAMPLE_PATH "ompl/demos;demos;ompl/doc/markdown/code")
    set(DOXYGEN_HTML_OUTPUT "${CMAKE_BINARY_DIR}/omplapp_doc")
    set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_HTML_OUTPUT}/ompl.tag")
    set(DOXYGEN_ENABLED_SECTIONS "OMPLAPP")

    doxygen_add_docs(omplapp_doc
        "${CMAKE_SOURCE_DIR}/doc/markdown"
        "${CMAKE_SOURCE_DIR}/demos"
        "${CMAKE_SOURCE_DIR}/src/omplapp"
        "${CMAKE_SOURCE_DIR}/ompl/doc/markdown"
        "${CMAKE_SOURCE_DIR}/ompl/demos"
        "${CMAKE_SOURCE_DIR}/ompl/src/ompl"
        "${CMAKE_BINARY_DIR}/doc"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_custom_command(TARGET omplapp_doc POST_BUILD
        COMMAND cp -r ompl.css images ieee-ram-2012-ompl.pdf "${CMAKE_CURRENT_SOURCE_DIR}/images" "${CMAKE_BINARY_DIR}/install-ompl-ubuntu.sh" "${DOXYGEN_HTML_OUTPUT}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/ompl/doc")

    # use latexmk to create the OMPL primer PDF
    find_program(LATEXMK latexmk)
    if (LATEXMK)
        add_custom_command(TARGET omplapp_doc POST_BUILD
            COMMAND "BIBINPUTS=${CMAKE_CURRENT_SOURCE_DIR}/latex/bib" latexmk -gg -pdf "-outdir=${DOXYGEN_HTML_OUTPUT}" OMPL_Primer.tex
            COMMAND latexmk -c -pdf "-outdir=${DOXYGEN_HTML_OUTPUT}" OMPL_Primer.tex
            WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/latex")
    endif()

    add_custom_target(website
        COMMAND rsync omplapp_doc/ "$ENV{HOME}/src/ompl.github.io"
        COMMAND rsync ompl_doc/ "$ENV{HOME}/src/ompl.github.io/core"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}")
    add_dependencies(website omplapp_doc ompl_doc)
endif()
