# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

#set(IDF_PATH "~/Tools/ESP/esp-idf")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bt_spp_acceptor_demo)
