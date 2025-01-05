# SPDX-License-Identifier: LicenseRef-ExiftoolCpp
# SPDX-FileCopyrightText: Czech Technical University in Prague

add_library(exiftool src/ExifTool.cpp src/ExifToolPipe.cpp src/TagInfo.cpp)
target_include_directories(exiftool SYSTEM PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/inc)
target_compile_options(exiftool PRIVATE -w)  # silence warnings