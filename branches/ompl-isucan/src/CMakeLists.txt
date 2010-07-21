add_subdirectory(ompl)
install(DIRECTORY ompl DESTINATION include/ FILES_MATCHING PATTERN "*.h*" REGEX "\\.svn" EXCLUDE REGEX "/src$" EXCLUDE)
