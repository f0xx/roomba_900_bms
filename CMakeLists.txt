cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
# This repo has no commit yet, and IDF 6's git-describe step errors in that case.
set(PROJECT_VER "1")
project(roomba_bms_unlock)
