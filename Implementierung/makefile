cmake_minimum_required (VERSION 3.0)
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

set (PROJECT_NAME papo-model)
project (${PROJECT_NAME})

set (VERSION_MAJOR 0)
set (VERSION_MINOR 1)

set(DEV_DIR "${CMAKE_SOURCE_DIR}/dev")
set(DOC_DIR "${CMAKE_SOURCE_DIR}/doc")
set(DEPS_DIR "${CMAKE_SOURCE_DIR}/deps")
set(SRC_DIR "${CMAKE_SOURCE_DIR}/src")

set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake ${DEV_DIR}/CMakeModules)

# Required packages for CMAKE
include(CTest)
include(FeatureSummary)
include(CheckCSourceCompiles)
include(CheckCSourceRuns)
include(FindPackageHandleStandardArgs)

# Dependencies for this software package

find_package(PkgConfig REQUIRED)
find_package(MPI REQUIRED)


option(CODE_COVERAGE "CODE_COVERAGE" OFF)

IF (CODE_COVERAGE)
IF ( CMAKE_BUILD_TYPE STREQUAL "Debug")
	IF ( CMAKE_COMPILER_IS_GNUCC )
	    include(CodeCoverage)
	    setup_target_for_coverage(coverage ctest coverage)
	    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
	    set(GCOV_LIBRARIES "-lgcov")
        else()
		MESSAGE (FATAL_ERROR "Error, you need to compile with gcc")
        endif()
else()
	MESSAGE (FATAL_ERROR "Error, you need to compile with Debug mode")
endif()
endif()


# compile flags
set (CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -std=gnu11 -pipe -Wall -Wno-unused-parameter -Wextra -Wfloat-equal -fdata-sections -ffunction-sections -Wl,--gc-sections -Wdouble-promotion -pipe -flto -pedantic-errors -pedantic")
set ( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wdouble-promotion -flto -pedantic-errors -pedantic -pipe -Wall -Wextra -Wfloat-equal -fdata-sections -ffunction-sections -Wl,--gc-sections")

# for specific targets
set (CMAKE_C_FLAGS_DEBUG   "-O0 -g3 -fvar-tracking -DDEBUG")
set (CMAKE_CXX_FLAGS_DEBUG "-O0 -g3")
set (CMAKE_C_FLAGS_RELEASE "-O3")
set (CMAKE_CXX_FLAGS_RELEASE  "-O3")


# If you need additional libraries / includes:
# find_path(LIBZ_INCLUDE_DIRS NAMES zlib.h PATHS ENV ADDITIONAL_INC_PATH)
# find_library(LIBZ_LIBRARIES NAMES z PATHS ENV ADDITIONAL_LIB_PATH)

# run during configure:
#execute_process(COMMAND ${DEPS_DIR}/build-dependencies.sh) # WORKING_DIRECTORY ${DEPS_DIR}

# Add ./src/ to the include directories
include_directories(${SRC_DIR} ${MPI_C_INCLUDE_PATH})

## Documentation
find_package(Doxygen)
if(DOXYGEN_FOUND)
    add_custom_target(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM
    )
    configure_file(${DOC_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
endif(DOXYGEN_FOUND)
##### END Documentation

SUBDIRS (test)
SUBDIRS (src)

##
feature_summary(WHAT ALL)
