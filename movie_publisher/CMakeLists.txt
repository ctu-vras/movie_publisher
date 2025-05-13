# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.1)
project(movie_publisher)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  compass_conversions
  compass_msgs
  cras_cpp_common
  cras_topic_tools
  gps_common
  image_geometry
  image_transport
  image_transport_codecs
  nodelet
  rosbag
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_msgs
  tf2_ros
  vision_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES movie_reader movie_reader_types
  CATKIN_DEPENDS compass_msgs cras_cpp_common gps_common roscpp sensor_msgs vision_msgs
)

find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBAVUTIL libavutil REQUIRED IMPORTED_TARGET)
pkg_check_modules(LIBAVCODEC libavcodec REQUIRED IMPORTED_TARGET)
pkg_check_modules(LIBAVFILTER libavfilter REQUIRED IMPORTED_TARGET)
pkg_check_modules(LIBAVFORMAT libavformat REQUIRED IMPORTED_TARGET)
pkg_check_modules(LIBSWSCALE libswscale REQUIRED IMPORTED_TARGET)
add_library(ffmpeg INTERFACE)
target_link_libraries(ffmpeg INTERFACE PkgConfig::LIBAVUTIL PkgConfig::LIBAVCODEC PkgConfig::LIBAVFILTER PkgConfig::LIBAVFORMAT PkgConfig::LIBSWSCALE)

# From cras_cpp_common, provides std::filesystem target
find_package(Filesystem REQUIRED COMPONENTS Final Experimental Boost)

pkg_check_modules(MUPARSER muparser REQUIRED IMPORTED_TARGET)

find_package(magic_enum CONFIG REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(movie_reader_types
  src/metadata_cache.cpp
  src/metadata_extractor.cpp
  src/movie_info.cpp
  src/movie_open_config.cpp
  src/movie_playback_state.cpp
  src/parsing_utils.cpp
  src/types.cpp
)
target_link_libraries(movie_reader_types PUBLIC ${catkin_LIBRARIES} PRIVATE ffmpeg magic_enum::magic_enum PkgConfig::MUPARSER)

add_library(movie_reader
  src/metadata/ExifBaseMetadataExtractor.cpp
  src/metadata_manager.cpp
  src/movie.cpp
  src/movie_metadata_processor.cpp
  src/movie_private.cpp
  src/movie_reader.cpp
  src/movie_reader_ros.cpp
)
target_link_libraries(movie_reader PUBLIC ${catkin_LIBRARIES} movie_reader_types PRIVATE ffmpeg magic_enum::magic_enum std::filesystem)

add_library(movie_reader_metadata_plugins
  src/metadata/FileMetadataExtractor.cpp
  src/metadata/FilenameMetadataExtractor.cpp
  src/metadata/LibavStreamMetadataExtractor.cpp
)
target_link_libraries(movie_reader_metadata_plugins PUBLIC ${catkin_LIBRARIES} movie_reader PRIVATE ffmpeg std::filesystem)

add_library(movie_reader_metadata_composers
  src/composers/CameraInfoComposer.cpp
  src/composers/CameraIntrinsicsComposer.cpp
  src/composers/CameraNameComposer.cpp
  src/composers/ImuComposer.cpp
  src/composers/OpticalFrameTFComposer.cpp
  src/composers/RollPitchComposer.cpp
  src/composers/ZeroRollPitchTFComposer.cpp
)
target_link_libraries(movie_reader_metadata_composers PUBLIC ${catkin_LIBRARIES} movie_reader)

add_library(movie_to_bag src/movie_to_bag.cpp)
target_link_libraries(movie_to_bag PUBLIC movie_reader ${catkin_LIBRARIES} PRIVATE std::filesystem)

add_executable(movie_to_bag_node nodes/movie_to_bag.cpp)
target_link_libraries(movie_to_bag_node movie_to_bag ${catkin_LIBRARIES})
set_target_properties(movie_to_bag_node PROPERTIES OUTPUT_NAME movie_to_bag PREFIX "")

add_library(${PROJECT_NAME}_nodelet nodelets/${PROJECT_NAME}_nodelet.cpp)
target_link_libraries(${PROJECT_NAME}_nodelet PUBLIC movie_reader ${catkin_LIBRARIES})
cras_node_from_nodelet(${PROJECT_NAME}_nodelet ${PROJECT_NAME}::MoviePublisherNodelet OUTPUT_NAME ${PROJECT_NAME} ANONYMOUS)

install(TARGETS movie_to_bag_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_nodelet movie_reader movie_reader_metadata_composers movie_reader_metadata_plugins movie_reader_types movie_to_bag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(PROGRAMS
  nodes/add_movie_to_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/fix_bag_timestamps
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
  launch/${PROJECT_NAME}.launch  # This is wrong, but let's keep it for backwards compatibility
  metadata_composers.xml
  metadata_plugins.xml
  nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)
  find_package(roslaunch REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES include/*.h src/*.cpp nodes/*.cpp nodelets/*.cpp test/*.cpp)
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  roslint_add_test()

  roslaunch_add_file_check(launch/${PROJECT_NAME}.launch IGNORE_UNSET_ARGS)

  catkin_add_gmock(test_movie_reader test/test_movie_reader.cpp)
  target_link_libraries(test_movie_reader movie_reader movie_reader_metadata_plugins)
  target_compile_definitions(test_movie_reader PRIVATE TEST_DATA_DIR="${CATKIN_DEVEL_PREFIX}/share/test_data")
  add_custom_command(TARGET test_movie_reader POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_SOURCE_DIR}/../test_data" "${CATKIN_DEVEL_PREFIX}/share/test_data")

  add_rostest_gmock(test_${PROJECT_NAME}_nodelet test/test_${PROJECT_NAME}_nodelet.test test/test_${PROJECT_NAME}_nodelet.cpp)
  target_link_libraries(test_${PROJECT_NAME}_nodelet ${catkin_LIBRARIES} ${PROJECT_NAME}_nodelet)
  add_dependencies(test_${PROJECT_NAME}_nodelet test_movie_reader)  # Wait for copying files
  target_compile_definitions(test_${PROJECT_NAME}_nodelet PRIVATE TEST_DATA_DIR="${CATKIN_DEVEL_PREFIX}/share/test_data")
  roslaunch_add_file_check(test/test_${PROJECT_NAME}_nodelet.test USE_TEST_DEPENDENCIES)
  if(CMAKE_VERSION VERSION_LESS "3.13.0")
    set_property(TARGET test_${PROJECT_NAME}_nodelet APPEND_STRING PROPERTY LINK_FLAGS " -Wl,--no-as-needed")
  else()
    #catkin_lint: ignore_once cmake_old
    target_link_options(test_${PROJECT_NAME}_nodelet PUBLIC "LINKER:--no-as-needed")
  endif()
endif()
