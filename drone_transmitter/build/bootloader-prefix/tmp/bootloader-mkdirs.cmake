# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/harry/esp/esp-idf/components/bootloader/subproject"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/tmp"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/src/bootloader-stamp"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/src"
  "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/harry/esp/projects/drone_transmitterV1/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
