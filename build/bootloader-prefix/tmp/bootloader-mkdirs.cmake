# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Espressif/frameworks/esp-idf-v4.4.4/components/bootloader/subproject"
  "D:/Project_ESP32/Robot/build/bootloader"
  "D:/Project_ESP32/Robot/build/bootloader-prefix"
  "D:/Project_ESP32/Robot/build/bootloader-prefix/tmp"
  "D:/Project_ESP32/Robot/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Project_ESP32/Robot/build/bootloader-prefix/src"
  "D:/Project_ESP32/Robot/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Project_ESP32/Robot/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
