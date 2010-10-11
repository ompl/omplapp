set(PQP_DIR "${CMAKE_SOURCE_DIR}/external/pqp-1.3")
set(PQP_TGZ "${PQP_DIR}.tar.gz")
# check if PQP directory already exists
# if not, download and extract PQP
if(NOT EXISTS ${PQP_DIR})
	if(NOT EXISTS ${PQP_TGZ})
 		file(DOWNLOAD http://gamma.cs.unc.edu/software/downloads/SSV/pqp-1.3.tar.gz 
			${PQP_TGZ} STATUS PQP_DL_STATUS)
		# check download exit status
		list(GET PQP_DL_STATUS 0 PQP_DL_STATUS_VAL)
		list(GET PQP_DL_STATUS 1 PQP_DL_STATUS_MSG)
		if(NOT PQP_DL_STATUS_VAL EQUAL 0)
			message(FATAL_ERROR "PQP download failed with this error: ${PQP_DL_STATUS_MSG}")
		endif()
		# check MD5 checksum
		execute_process(COMMAND ${CMAKE_COMMAND} -E md5sum ${PQP_TGZ}
			OUTPUT_VARIABLE PQP_MD5)
		if(NOT PQP_MD5 MATCHES "^f710e24a62db763d61d08667439d46fd.*pqp-1.3.tar.gz.*")
			message(WARNING "The PQP distribution has changed. Compilation may fail.")
		endif()
	endif()
	# extract PQP tar ball
	execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${PQP_TGZ}
		WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/external")
	# show PQP license
	message(STATUS "	
-------------------------------------------------------------------------------
Copyright 1999 The University of North Carolina at Chapel Hill.
All Rights Reserved.

Permission to use, copy, modify and distribute this software and its
documentation for educational, research and non-profit purposes, without
fee, and without a written agreement is hereby granted, provided that the
above copyright notice and the following three paragraphs appear in all
copies.

IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL BE
LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE
USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY
OF NORTH CAROLINA HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
DAMAGES.

THE UNIVERSITY OF NORTH CAROLINA SPECIFICALLY DISCLAIM ANY
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE
PROVIDED HEREUNDER IS ON AN \"AS IS\" BASIS, AND THE UNIVERSITY OF
NORTH CAROLINA HAS NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT,
UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

The authors may be contacted via:

US Mail:             E. Larsen
                     Department of Computer Science
                     Sitterson Hall, CB #3175
                     University of N. Carolina
                     Chapel Hill, NC 27599-3175

Phone:               (919)962-1749

EMail:               geom@cs.unc.edu
-------------------------------------------------------------------------------

** THE PQP LIBRARY FOR COLLISION CHECKING HAS BEEN DOWNLOADED. BY USING THIS 
** LIBRARY YOU AGREE TO THE LICENSE ABOVE.
")
endif()


macro(ompl_backup filename)
	if(NOT EXISTS "${filename}.orig")
		if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 2.6)
			file(RENAME "${filename}" "${filename}.orig")
		else()
			file(READ "${filename}" FILECONTENTS)
			file(WRITE "${filename}.orig" "${FILECONTENTS}")
		endif()
	endif(NOT EXISTS "${filename}.orig")
endmacro(ompl_backup)

if(MSVC_IDE)
	message(STATUS "Patching PQP")
	ompl_backup("${PQP_INCLUDE}/PQP_compile.h")
	file(READ "${PQP_INCLUDE}/PQP_compile.h.orig" PQPCOMPILE_H_ORIG)
	string(REPLACE "inline float sqrt(float x) { return (float)sqrt((double)x); }\ninline float cos(float x) { return (float)cos((double)x); }\ninline float sin(float x) { return (float)sin((double)x); }\ninline float fabs(float x) { return (float)fabs((double)x); }\n" "" PQPCOMPILE_H "${PQPCOMPILE_H_ORIG}")
	file(WRITE "${PQP_INCLUDE}/PQP_compile.h" "${PQPCOMPILE_H}")
	message(STATUS "Patched ${PQP_INCLUDE}/PQP_compile.h")

	ompl_backup("${PQP_INCLUDE}/PQP.h")
	file(READ "${PQP_INCLUDE}/PQP.h.orig" PQP_H_ORIG)
	string(REPLACE "\nPQP_" "\n__declspec(dllexport) PQP_" PQP_H "${PQP_H_ORIG}")
	file(WRITE "${PQP_INCLUDE}/PQP.h" "${PQP_H}")
	message(STATUS "Patched ${PQP_INCLUDE}/PQP.h")

	ompl_backup("${PQP_INCLUDE}/PQP_internal.h")
	file(READ "${PQP_INCLUDE}/PQP_Internal.h.orig" PQPINTERNAL_H_ORIG)
	string(REPLACE "class PQP_Model" "class __declspec(dllexport) PQP_Model" PQPINTERNAL_H_1 "${PQPINTERNAL_H_ORIG}")
	string(REPLACE "struct CollisionPair" "struct __declspec(dllexport) CollisionPair" PQPINTERNAL_H_2 "${PQPINTERNAL_H_1}")
	string(REPLACE "struct PQP_CollideResult" "struct __declspec(dllexport) PQP_CollideResult" PQPINTERNAL_H "${PQPINTERNAL_H_2}")
	file(WRITE "${PQP_INCLUDE}/PQP_Internal.h" "${PQPINTERNAL_H}")
	message(STATUS "Patched ${PQP_INCLUDE}/PQP_Internal.h")
endif(MSVC_IDE)


set(PQP_INCLUDE_DIR "${PQP_DIR}/PQP_v1.3/src")
aux_source_directory(${PQP_INCLUDE_DIR} SRC_PQP)
add_library(PQP SHARED ${SRC_PQP})
set(PQP_LIBRARY "PQP")
install(TARGETS PQP DESTINATION lib/)
