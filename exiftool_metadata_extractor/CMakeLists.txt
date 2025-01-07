# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(exiftool_metadata_extractor)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  movie_publisher
  pluginlib
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_subdirectory(src/cpp_exiftool)

add_library(${PROJECT_NAME} src/ExiftoolMetadataExtractor.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} PRIVATE exiftool)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_FILES include/*.h include/*.hpp src/*.cpp src/*.hpp src/*.h test/*.cpp)
  list(FILTER ROSLINT_FILES EXCLUDE REGEX ".*/cpp_exiftool/.*")

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  roslint_add_test()

  catkin_add_gtest(test_${PROJECT_NAME} test/test_extractor.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
  #catkin_lint: ignore ambiguous_include_path
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE TEST_DATA_DIR="${CATKIN_DEVEL_PREFIX}/share/test_data")

  add_custom_command(TARGET test_${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/../test_data" "${CATKIN_DEVEL_PREFIX}/share/test_data")
endif()
