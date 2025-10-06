# Download stb
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG ae721c50eaf761660b4f90cc590453cdb0c2acd0
)

FetchContent_MakeAvailable(stb)

# Download zlib
FetchContent_Declare(
    zlib
    GIT_REPOSITORY https://github.com/madler/zlib.git
    GIT_TAG v1.3.1
)

set(ZLIB_BUILD_EXAMPLES OFF)
set(ZLIB_BUILD_TESTS OFF)
set(ZLIB_ENABLE_INSTALL OFF)

FetchContent_MakeAvailable(zlib)

# Download nanovdb
FetchContent_Declare(
    nanovdb
    GIT_REPOSITORY https://github.com/AcademySoftwareFoundation/openvdb.git
    GIT_TAG feature/nanovdb
)

FetchContent_Populate(nanovdb)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE HEADER_FILES ${CMAKE_CURRENT_SOURCE_DIR}/../include/bulbit/*.h)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" PREFIX "src" FILES ${SOURCE_FILES})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}/../include" PREFIX "include" FILES ${HEADER_FILES})

add_library(bulbit STATIC ${HEADER_FILES} ${SOURCE_FILES})

target_precompile_headers(bulbit
    PUBLIC
    ../include/bulbit/common.h
)

target_include_directories(bulbit
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
    ${stb_SOURCE_DIR}
    ${nanovdb_SOURCE_DIR}/nanovdb/nanovdb
)

target_link_libraries(bulbit PUBLIC zlib)

set_target_properties(bulbit PROPERTIES
    CMAKE_COMPILE_WARNING_AS_ERROR ON
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

if(MSVC)
    target_compile_options(bulbit PRIVATE /W4 /WX /wd4458 /wd4459)
else()
    target_compile_options(bulbit PRIVATE -Wall -Wextra -Wpedantic -Werror -Wno-missing-field-initializers)
endif()