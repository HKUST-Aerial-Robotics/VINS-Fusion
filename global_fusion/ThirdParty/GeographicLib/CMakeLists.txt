project (GeographicLib)

# Version information
set (PROJECT_VERSION_MAJOR 1)
set (PROJECT_VERSION_MINOR 49)
set (PROJECT_VERSION_PATCH 0)
set (PROJECT_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}")
if (PROJECT_VERSION_PATCH GREATER 0)
  set (PROJECT_VERSION "${PROJECT_VERSION}.${PROJECT_VERSION_PATCH}")
endif ()

# The library version tracks the numbering given by libtool in the
# autoconf set up.
set (LIBVERSION_API 17)
set (LIBVERSION_BUILD 17.1.2)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string (TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)

cmake_minimum_required (VERSION 2.8.4) # This version was released 2011-02-16

# (7) Set the default "real" precision.  This should probably be left
# at 2 (double).
set (GEOGRAPHICLIB_PRECISION 2 CACHE STRING
  "Precision: 1 = float, 2 = double, 3 = extended, 4 = quadruple, 5 = variable")
set_property (CACHE GEOGRAPHICLIB_PRECISION PROPERTY STRINGS 1 2 3 4 5)


set (LIBNAME Geographic)

include_directories(
  ./include/
)

add_library(libGeographiccc src/LocalCartesian.cpp
                          src/Geocentric.cpp
                          src/Math.cpp)