# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.1)
project(movie_publisher)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  compass_conversions
  compass_msgs
  cras_cpp_common
  gps_common
  image_geometry
  image_transport
  image_transport_codecs
  nodelet
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES movie_reader
  CATKIN_DEPENDS compass_msgs cras_cpp_common gps_common roscpp sensor_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBAVUTIL libavutil REQUIRED)
pkg_check_modules(LIBAVCODEC libavcodec REQUIRED)
pkg_check_modules(LIBAVFILTER libavfilter REQUIRED)
pkg_check_modules(LIBAVFORMAT libavformat REQUIRED)
pkg_check_modules(LIBSWSCALE libswscale REQUIRED)
add_library(ffmpeg INTERFACE IMPORTED)
target_link_libraries(ffmpeg INTERFACE ${LIBAVUTIL_LIBRARIES} ${LIBAVCODEC_LIBRARIES} ${LIBAVFILTER_LIBRARIES} ${LIBAVFORMAT_LIBRARIES} ${LIBSWSCALE_LIBRARIES})
target_include_directories(ffmpeg INTERFACE ${LIBAVUTIL_INCLUDE_DIRS} ${LIBAVCODEC_INCLUDE_DIRS} ${LIBAVFILTER_INCLUDE_DIRS} ${LIBAVFORMAT_INCLUDE_DIRS} ${LIBSWSCALE_INCLUDE_DIRS})

find_package(Filesystem REQUIRED)  # From cras_cpp_common

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(movie_reader
  src/metadata/ExifBaseMetadataExtractor.cpp
  src/metadata_manager.cpp
  src/movie_reader.cpp
  src/movie_reader_private.cpp
)
target_link_libraries(movie_reader PUBLIC ${catkin_LIBRARIES} PRIVATE ffmpeg std::filesystem)

add_library(movie_reader_metadata_plugins
  src/metadata/FileMetadataExtractor.cpp
  src/metadata/FilenameMetadataExtractor.cpp
  src/metadata/LibavStreamMetadataExtractor.cpp
)
target_link_libraries(movie_reader_metadata_plugins PUBLIC ${catkin_LIBRARIES} movie_reader PRIVATE ffmpeg std::filesystem)

add_library(${PROJECT_NAME}_nodelet nodes/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PUBLIC movie_reader ${catkin_LIBRARIES})
cras_node_from_nodelet(${PROJECT_NAME}_nodelet ${PROJECT_NAME}::MoviePublisherNodelet OUTPUT_NAME ${PROJECT_NAME} ANONYMOUS)

install(TARGETS ${PROJECT_NAME}_nodelet movie_reader movie_reader_metadata_plugins
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS
  scripts/add_movie_to_bag
  scripts/movie_to_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/${PROJECT_NAME}_node
  scripts/fix_bag_timestamps
  scripts/merge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/${PROJECT_NAME}.launch  # This is wrong, but let's keep it for backwards compatibility
  metadata_plugins.xml
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files

  file(GLOB_RECURSE ROSLINT_INCLUDE include/*.h include/*.hpp)
  file(GLOB_RECURSE ROSLINT_SRC src/*.cpp src/*.hpp src/*.h)
  file(GLOB_RECURSE ROSLINT_TEST test/*.cpp)

  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_INCLUDE} ${ROSLINT_SRC} ${ROSLINT_TEST})

  roslint_add_test()

  # catkin_add_gtest(${PROJECT_NAME}-test test/test_camera_info_manager_lib.cpp)
  # target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

endif()
