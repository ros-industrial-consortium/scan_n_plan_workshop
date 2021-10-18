cmake_minimum_required(VERSION 3.5.0)

include(ExternalProject)

if (NOT UPDATE_DISCONNECTED)
  set(UPDATE_DISCONNECTED ON)
endif()

if (NOT INSTALL_FCL)
  find_package(fcl 0.6 QUIET)
endif()

if (NOT fcl_FOUND OR INSTALL_FCL)
  if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(FCL_CXX_FLAGS "-w -std=c++14")
    set(FCL_C_FLAGS "-w")

    ExternalProject_Add(fcl
      GIT_REPOSITORY        https://github.com/flexible-collision-library/fcl.git
      GIT_TAG               0.6.1
      SOURCE_DIR            ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-src
      BINARY_DIR            ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-build
      UPDATE_DISCONNECTED   ${UPDATE_DISCONNECTED}
      CMAKE_CACHE_ARGS
              -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
              -DCMAKE_BUILD_TYPE:STRING=Release
              -DCMAKE_CXX_FLAGS:STRING=${FCL_CXX_FLAGS}
              -DCMAKE_C_FLAGS:STRING=${FCL_C_FLAGS}
              -DFCL_BUILD_TESTS:BOOL=OFF
              -DBUILD_TESTING:BOOL=OFF
    )
  elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    ExternalProject_Add(fcl
      GIT_REPOSITORY        https://github.com/flexible-collision-library/fcl.git
      GIT_TAG               0.6.1
      SOURCE_DIR            ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-src
      BINARY_DIR            ${CMAKE_BINARY_DIR}/../tesseract_ext/fcl-build
      UPDATE_DISCONNECTED   ${UPDATE_DISCONNECTED}
      CMAKE_CACHE_ARGS
              -DCMAKE_INSTALL_PREFIX:STRING=${CMAKE_INSTALL_PREFIX}
              -DCMAKE_BUILD_TYPE:STRING=Release
              -DFCL_BUILD_TESTS:BOOL=OFF
              -DBUILD_TESTING:BOOL=OFF
    )
  endif()

  install(FILES package.xml DESTINATION share/fcl)
else()
  install(FILES package.xml DESTINATION share/dummy RENAME fcl.xml)
endif()
