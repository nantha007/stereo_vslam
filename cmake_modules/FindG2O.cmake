include(FindPackageHandleStandardArgs)

# Build search paths for ROS/colcon installs (e.g. /opt/ros/kilted)
set(G2O_SEARCH_INCLUDE_PATHS
  ${G2O_ROOT}/include
  $ENV{G2O_ROOT}/include
  $ENV{G2O_ROOT}
  /usr/local/include
  /usr/include
  /opt/local/include
  /sw/local/include
  /sw/include
)
set(G2O_SEARCH_LIB_PATHS
  ${G2O_ROOT}/lib
  ${G2O_ROOT}/lib/Release
  ${G2O_ROOT}/lib/Debug
  $ENV{G2O_ROOT}/lib
  $ENV{G2O_ROOT}/lib/Release
  $ENV{G2O_ROOT}/lib/Debug
  /usr/local/lib
  /usr/local/lib64
  /usr/lib
  /usr/lib64
  /opt/local/lib
  /sw/local/lib
  /sw/lib
)
if(DEFINED ENV{AMENT_PREFIX_PATH})
  string(REPLACE ":" ";" _ament_prefix_paths $ENV{AMENT_PREFIX_PATH})
  foreach(_prefix ${_ament_prefix_paths})
    list(APPEND G2O_SEARCH_INCLUDE_PATHS ${_prefix}/include)
    list(APPEND G2O_SEARCH_LIB_PATHS
      ${_prefix}/lib
      ${_prefix}/lib/${CMAKE_LIBRARY_ARCHITECTURE})
  endforeach()
endif()
file(GLOB _ros_install_dirs "/opt/ros/*")
foreach(_rosdir ${_ros_install_dirs})
  list(APPEND G2O_SEARCH_INCLUDE_PATHS ${_rosdir}/include)
  list(APPEND G2O_SEARCH_LIB_PATHS
    ${_rosdir}/lib
    ${_rosdir}/lib/${CMAKE_LIBRARY_ARCHITECTURE})
endforeach()

# Find the header files
FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_SEARCH_INCLUDE_PATHS}
  NO_DEFAULT_PATH
  )
FIND_PATH(G2O_INCLUDE_DIR g2o/core/base_vertex.h
  ${G2O_SEARCH_INCLUDE_PATHS}
  )

# Macro to unify finding both the debug and release versions of the
# libraries; this is adapted from the OpenSceneGraph FIND_LIBRARY
# macro.

MACRO(FIND_G2O_LIBRARY MYLIBRARY MYLIBRARYNAME)

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS ${G2O_SEARCH_LIB_PATHS}
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY("${MYLIBRARY}_DEBUG"
    NAMES "g2o_${MYLIBRARYNAME}_d"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    ${G2O_SEARCH_LIB_PATHS}
    )
  
  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS ${G2O_SEARCH_LIB_PATHS}
    NO_DEFAULT_PATH
    )

  FIND_LIBRARY(${MYLIBRARY}
    NAMES "g2o_${MYLIBRARYNAME}"
    PATHS
    ~/Library/Frameworks
    /Library/Frameworks
    ${G2O_SEARCH_LIB_PATHS}
    )
  
  IF(NOT ${MYLIBRARY}_DEBUG)
    IF(MYLIBRARY)
      SET(${MYLIBRARY}_DEBUG ${MYLIBRARY})
    ENDIF(MYLIBRARY)
  ENDIF( NOT ${MYLIBRARY}_DEBUG)
  
ENDMACRO(FIND_G2O_LIBRARY LIBRARY LIBRARYNAME)

# Find the core elements
FIND_G2O_LIBRARY(G2O_STUFF_LIBRARY stuff)
FIND_G2O_LIBRARY(G2O_CORE_LIBRARY core)

# Find the CLI library
FIND_G2O_LIBRARY(G2O_CLI_LIBRARY cli)

# Find the pluggable solvers
FIND_G2O_LIBRARY(G2O_SOLVER_CHOLMOD solver_cholmod)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE solver_csparse)
FIND_G2O_LIBRARY(G2O_SOLVER_CSPARSE_EXTENSION csparse_extension)
FIND_G2O_LIBRARY(G2O_SOLVER_DENSE solver_dense)
FIND_G2O_LIBRARY(G2O_SOLVER_PCG solver_pcg)
FIND_G2O_LIBRARY(G2O_SOLVER_SLAM2D_LINEAR solver_slam2d_linear)
FIND_G2O_LIBRARY(G2O_SOLVER_STRUCTURE_ONLY solver_structure_only)
FIND_G2O_LIBRARY(G2O_SOLVER_EIGEN solver_eigen)

# Find the predefined types
FIND_G2O_LIBRARY(G2O_TYPES_DATA types_data)
FIND_G2O_LIBRARY(G2O_TYPES_ICP types_icp)
FIND_G2O_LIBRARY(G2O_TYPES_SBA types_sba)
FIND_G2O_LIBRARY(G2O_TYPES_SCLAM2D types_sclam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SIM3 types_sim3)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM2D types_slam2d)
FIND_G2O_LIBRARY(G2O_TYPES_SLAM3D types_slam3d)

# G2O solvers declared found if we found at least one solver
SET(G2O_SOLVERS_FOUND "NO")
IF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)
  SET(G2O_SOLVERS_FOUND "YES")
ENDIF(G2O_SOLVER_CHOLMOD OR G2O_SOLVER_CSPARSE OR G2O_SOLVER_DENSE OR G2O_SOLVER_PCG OR G2O_SOLVER_SLAM2D_LINEAR OR G2O_SOLVER_STRUCTURE_ONLY OR G2O_SOLVER_EIGEN)

set(G2O_INCLUDE_DIRS ${G2O_INCLUDE_DIR})

find_package_handle_standard_args(G2O DEFAULT_MSG
  G2O_INCLUDE_DIR
  G2O_STUFF_LIBRARY
  G2O_CORE_LIBRARY
  G2O_SOLVERS_FOUND
)

mark_as_advanced(
  G2O_INCLUDE_DIR
  G2O_STUFF_LIBRARY
  G2O_CORE_LIBRARY
)
