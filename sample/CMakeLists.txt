# Include tinyobj
FetchContent_Declare(
    tinyobj
    GIT_REPOSITORY "https://github.com/tinyobjloader/tinyobjloader.git"
    GIT_TAG release
)
FetchContent_Populate(tinyobj)

# Include tinygltf
set(TINYGLTF_HEADER_ONLY ON CACHE INTERNAL "" FORCE)
set(TINYGLTF_INSTALL OFF CACHE INTERNAL "" FORCE)
FetchContent_Declare(
    tinygltf
    GIT_REPOSITORY "https://github.com/syoyo/tinygltf.git"
    GIT_TAG v2.9.6
)
FetchContent_Populate(tinygltf)

# Include pugixml
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY "https://github.com/zeux/pugixml.git"
    GIT_TAG v1.15
)
FetchContent_MakeAvailable(pugixml)

file(GLOB_RECURSE SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)

source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${SOURCE_FILES})

add_executable(bbrender ${HEADER_FILES} ${SOURCE_FILES})
target_include_directories(bbrender PRIVATE ${tinyobj_SOURCE_DIR} ${tinygltf_SOURCE_DIR})
target_link_libraries(bbrender PRIVATE bulbit pugixml)
set_target_properties(bbrender PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
    CXX_EXTENSIONS NO
)

# add_custom_command(
# TARGET bbrender POST_BUILD
# COMMAND ${CMAKE_COMMAND} -E copy_directory_if_different
# ${CMAKE_CURRENT_SOURCE_DIR}/../res/
# ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/$<CONFIG>/res/
# )
