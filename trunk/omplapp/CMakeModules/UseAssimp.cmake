set(ASSIMP_ROOT_DIR "${CMAKE_SOURCE_DIR}/external/assimp")
set(ASSIMP_INCLUDE_DIR "${ASSIMP_ROOT_DIR}/include")
# the next three variables are defined in assimp's top-level CMakeLists.txt, but
# we're skipping that one, since we only want the core assimp library and none 
# of the other stuff.
set(LIB_INSTALL_DIR "lib")
set(INCLUDE_INSTALL_DIR "include")
set(BIN_INSTALL_DIR "bin")

add_subdirectory("${ASSIMP_ROOT_DIR}/code")
set(ASSIMP_LIBRARY "assimp")
