add_subdirectory(omplapp)
install(DIRECTORY omplapp/ DESTINATION "include/omplapp${OMPL_INSTALL_SUFFIX}"
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".DS_Store" EXCLUDE
    PATTERN ".svn" EXCLUDE)
