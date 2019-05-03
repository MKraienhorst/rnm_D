# Install script for directory: /home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE FILE FILES
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/QR"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/MetisSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SVD"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/UmfPackSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SuperLUSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/CholmodSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Dense"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Sparse"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Eigen"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SparseQR"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/IterativeLinearSolvers"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Eigenvalues"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Householder"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SPQRSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/OrderingMethods"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/StdList"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/PaStiXSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Jacobi"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SparseCholesky"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Cholesky"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/QtAlignedMalloc"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/StdVector"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Core"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/LU"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SparseCore"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/SparseLU"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/StdDeque"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/PardisoSupport"
    "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/Geometry"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Devel")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/eigen3/Eigen" TYPE DIRECTORY FILES "/home/rnm/project_GroupD/rnm_ss19d/Eigen/Eigen/src" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

