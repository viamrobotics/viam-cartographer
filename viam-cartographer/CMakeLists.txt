cmake_minimum_required(VERSION 3.2)

project(viam-cartographer)
set (CMAKE_CXX_STANDARD 17)

message("PROJECT SOURCE DIR: " ${PROJECT_SOURCE_DIR})
message("API SOURCE DIR: " ${API_SOURCE_DIR})


# dependencies
add_subdirectory(cartographer)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPCPP REQUIRED grpc++)

find_package(absl REQUIRED)
add_definitions(-DABSL_LEGACY_THREAD_ANNOTATIONS)

find_package(PCL REQUIRED)


# build sources list
file(GLOB_RECURSE ALL_VIAM_HDRS "src/*.h")
file(GLOB_RECURSE ALL_VIAM_SRCS "src/*.cc")
file(GLOB_RECURSE ALL_VIAM_C_SRCS "src/*.c")
file(GLOB_RECURSE ALL_VIAM_TEST_SRCS "src/*_test.cc")
list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_VIAM_TEST_SRCS})

# remove dotfiles/-folders that could potentially pollute the build.
file(GLOB_RECURSE ALL_DOTFILES ".*/*")
if (ALL_DOTFILES)
  list(REMOVE_ITEM ALL_VIAM_HDRS ${ALL_DOTFILES})
  list(REMOVE_ITEM ALL_VIAM_SRCS ${ALL_DOTFILES})
endif()

# combine sources
list(APPEND ALL_LIBRARY_HDRS ${ALL_VIAM_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_VIAM_SRCS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_VIAM_C_SRCS})



# the viam-cartographer library
add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_HDRS} ${ALL_LIBRARY_SRCS} ${GRPC_SRCS})
target_link_libraries(${PROJECT_NAME} PUBLIC
  cartographer
  ${GRPCPP_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(unit_tests ${ALL_VIAM_TEST_SRCS})
target_link_libraries(unit_tests PUBLIC viam-cartographer)
