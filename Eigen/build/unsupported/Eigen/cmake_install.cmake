# Install script for directory: /home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE FILE FILES
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/AdolcForward"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/AlignedVector3"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/ArpackSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/AutoDiff"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/BVH"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/EulerAngles"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/FFT"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/IterativeSolvers"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/KroneckerProduct"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/LevenbergMarquardt"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/MatrixFunctions"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/MoreVectorization"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/MPRealSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/NonLinearOptimization"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/NumericalDiff"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/OpenGLSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/Polynomials"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/Skyline"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/SparseExtra"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/SpecialFunctions"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/Splines"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/unsupported/Eigen" TYPE DIRECTORY FILES "/home/rnm/project_GroupD/rnm_ss19d/Eigen/unsupported/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rnm/project_GroupD/rnm_ss19d/Eigen/build/unsupported/Eigen/CXX11/cmake_install.cmake")

endif()

