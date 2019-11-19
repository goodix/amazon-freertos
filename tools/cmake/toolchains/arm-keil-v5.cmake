include("${CMAKE_CURRENT_LIST_DIR}/find_compiler.cmake")

set(CMAKE_SYSTEM_NAME Generic)

# Find Keil for ARM.
afr_find_compiler(AFR_COMPILER_CC armcc.exe)
afr_find_compiler(AFR_COMPILER_CXX armcc.exe)
afr_find_compiler(AFR_COMPILER_ASM armasm.exe)
afr_find_compiler(AFR_COMPILER_LINK armlink.exe)

# Specify the cross compiler.
set(CMAKE_C_COMPILER ${AFR_COMPILER_CC} CACHE FILEPATH "C compiler")
set(CMAKE_CXX_COMPILER ${AFR_COMPILER_CXX} CACHE FILEPATH "C++ compiler")
set(CMAKE_ASM_COMPILER ${AFR_COMPILER_ASM} CACHE FILEPATH "ASM compiler")
set(CMAKE_C_LINKER ${AFR_COMPILER_LINK} CACHE FILEPATH "C linker")

# Must specify '-' before q/qc in armcc compiler
set(CMAKE_C_ARCHIVE_CREATE "<CMAKE_AR> -qc <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_C_ARCHIVE_APPEND "<CMAKE_AR> -q  <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> -qc <TARGET> <LINK_FLAGS> <OBJECTS>")
set(CMAKE_CXX_ARCHIVE_APPEND "<CMAKE_AR> -q  <TARGET> <LINK_FLAGS> <OBJECTS>")

# SET(CMAKE_C_LINK_EXECUTABLE ${AFR_COMPILER_LINK})
set(CMAKE_C_LINK_EXECUTABLE "${CMAKE_C_LINKER} <FLAGS> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> <OBJECTS>  -o <TARGET> <LINK_LIBRARIES>")

# Not Use Response , to avoid error "objects1.rsp no such file or directory" under windows
set(CMAKE_C_USE_RESPONSE_FILE_FOR_OBJECTS 0)
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_OBJECTS 0)

set(CMAKE_C_USE_RESPONSE_FILE_LINK_FLAG "@")
set(CMAKE_CXX_USE_RESPONSE_FILE_LINK_FLAG "@")

# debug mode
set(CMAKE_BUILD_TYPE "Debug")

# Disable compiler checks.
set(CMAKE_C_COMPILER_FORCED TRUE)
set(CMAKE_CXX_COMPILER_FORCED TRUE)

# Add target system root to cmake find path.
get_filename_component(AFR_COMPILER_DIR "${AFR_COMPILER_CC}" DIRECTORY)
get_filename_component(CMAKE_FIND_ROOT_PATH "${AFR_COMPILER_DIR}" DIRECTORY)

# Don't look for executable in target system prefix.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)

# Look for includes and libraries only in the target system prefix.
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
