add_subdirectory(omplapp)
install(DIRECTORY omplapp DESTINATION include/ 
	FILES_MATCHING PATTERN "*.h"
	PATTERN ".DS_Store" EXCLUDE
	PATTERN ".svn" EXCLUDE)
