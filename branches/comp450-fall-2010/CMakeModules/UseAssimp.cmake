set(ASSIMP_ROOT_DIR "${CMAKE_SOURCE_DIR}/external/assimp")
set(ASSIMP_INCLUDE_DIR "${ASSIMP_ROOT_DIR}/include")
set(ASSIMP_LIBRARY "assimp")
set(ASSIMP_PUBLIC_HEADERS
	${ASSIMP_INCLUDE_DIR}/aiAnim.h
	${ASSIMP_INCLUDE_DIR}/aiAssert.h
	${ASSIMP_INCLUDE_DIR}/aiCamera.h
	${ASSIMP_INCLUDE_DIR}/aiColor4D.h
	${ASSIMP_INCLUDE_DIR}/aiColor4D.inl
	${ASSIMP_INCLUDE_DIR}/aiConfig.h
	${ASSIMP_INCLUDE_DIR}/aiDefines.h
	${ASSIMP_INCLUDE_DIR}/aiFileIO.h
	${ASSIMP_INCLUDE_DIR}/aiLight.h
	${ASSIMP_INCLUDE_DIR}/aiMaterial.h
	${ASSIMP_INCLUDE_DIR}/aiMaterial.inl
	${ASSIMP_INCLUDE_DIR}/aiMatrix3x3.h
	${ASSIMP_INCLUDE_DIR}/aiMatrix3x3.inl
	${ASSIMP_INCLUDE_DIR}/aiMatrix4x4.h
	${ASSIMP_INCLUDE_DIR}/aiMatrix4x4.inl
	${ASSIMP_INCLUDE_DIR}/aiMesh.h
	${ASSIMP_INCLUDE_DIR}/aiPostProcess.h
	${ASSIMP_INCLUDE_DIR}/aiQuaternion.h
	${ASSIMP_INCLUDE_DIR}/aiScene.h
	${ASSIMP_INCLUDE_DIR}/aiTexture.h
	${ASSIMP_INCLUDE_DIR}/aiTypes.h
	${ASSIMP_INCLUDE_DIR}/aiVector2D.h
	${ASSIMP_INCLUDE_DIR}/aiVector3D.h
	${ASSIMP_INCLUDE_DIR}/aiVector3D.inl
	${ASSIMP_INCLUDE_DIR}/aiVersion.h
	${ASSIMP_INCLUDE_DIR}/assimp.h
	${ASSIMP_INCLUDE_DIR}/assimp.hpp
	${ASSIMP_INCLUDE_DIR}/DefaultLogger.h
	${ASSIMP_INCLUDE_DIR}/IOStream.h
	${ASSIMP_INCLUDE_DIR}/IOSystem.h
	${ASSIMP_INCLUDE_DIR}/Logger.h
	${ASSIMP_INCLUDE_DIR}/LogStream.h
	${ASSIMP_INCLUDE_DIR}/NullLogger.h
)
set(ASSIMP_COMPILER_HEADERS
	${ASSIMP_INCLUDE_DIR}/Compiler/pushpack1.h
	${ASSIMP_INCLUDE_DIR}/Compiler/poppack1.h
	pstdint.h
)

file(GLOB ASSIMP_CODE "${ASSIMP_ROOT_DIR}/code/*.cpp")
set(ASSIMP_CONTRIB 
	${ASSIMP_ROOT_DIR}/contrib/irrXML/irrXML.cpp
	${ASSIMP_ROOT_DIR}/contrib/zlib/adler32.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/compress.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/crc32.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/deflate.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/inffast.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/inflate.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/inftrees.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/trees.c
	${ASSIMP_ROOT_DIR}/contrib/zlib/zutil.c
	${ASSIMP_ROOT_DIR}/contrib/ConvertUTF/ConvertUTF.c
	${ASSIMP_ROOT_DIR}/contrib/unzip/ioapi.c
	${ASSIMP_ROOT_DIR}/contrib/unzip/unzip.c)

add_library(assimp SHARED ${ASSIMP_CODE} ${ASSIMP_CONTRIB})

add_definitions( -DASSIMP_BUILD_DLL_EXPORT )

if(WIN32)
	if(MSVC80 OR MSVC90)
		add_definitions(-D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS)
	endif(MSVC80 OR MSVC90)
endif(WIN32)
install(FILES ${ASSIMP_PUBLIC_HEADERS} DESTINATION include/assimp/ )
install(FILES ${ASSIMP_COMPILER_HEADERS} DESTINATION include/assimp/Compiler/)
install(TARGETS assimp DESTINATION lib/)