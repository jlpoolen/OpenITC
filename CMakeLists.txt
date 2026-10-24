cmake_minimum_required(VERSION 3.20)
project(OpenITC LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(RTLSDR REQUIRED IMPORTED_TARGET librtlsdr)
pkg_check_modules(SQLITE3 REQUIRED IMPORTED_TARGET sqlite3)

# ---- core: SDR capture + native PHY + decode engine (no UI) ----
add_library(openitc_core STATIC
  src/rtl_source.cpp
  src/dsp.cpp
  src/itc.cpp
  src/parse.cpp
  src/repository.cpp
  src/engine.cpp
  src/store.cpp
)
target_include_directories(openitc_core PUBLIC src)
target_link_libraries(openitc_core PUBLIC PkgConfig::RTLSDR PkgConfig::SQLITE3)
target_compile_options(openitc_core PRIVATE -Wall -Wextra)

# ---- console front-end ----
add_executable(openitc
  src/main.cpp
  src/native.cpp
)
target_link_libraries(openitc PRIVATE openitc_core)
target_compile_options(openitc PRIVATE -Wall -Wextra)

