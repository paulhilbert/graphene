###############################################################################
# Find Space Navigator libraries
#
# This sets the following variables:
# SPACENAV_FOUND - True if SpaceNav was found.
# SPACENAV_INCLUDE_DIRS - Directory containing the SpaceNav include files.
# SPACENAV_LIBRARY_DIRS - Directory containing the SpaceNav library.
# SPACENAV_LIBRARIES - SpaceNav library files.

find_path(SPACENAV_INCLUDE_DIR spnav.h HINTS "/usr/include" "/usr/local/include")

find_library(SPACENAV_LIBRARY_DIR spnav HINTS "/usr/lib" "/usr/local/lib")

if(EXISTS ${SPACENAV_LIBRARY_DIR})
get_filename_component(SPACENAV_LIBRARY ${SPACENAV_LIBRARY_DIR} NAME)
endif()

set(SPACENAV_INCLUDE_DIRS ${SPACENAV_INCLUDE_DIR})
set(SPACENAV_LIBRARY_DIRS ${SPACENAV_LIBRARY_DIR})
set(SPACENAV_LIBRARIES ${SPACENAV_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(SpaceNav DEFAULT_MSG SPACENAV_INCLUDE_DIR SPACENAV_LIBRARY_DIR SPACENAV_LIBRARY)

mark_as_advanced(SPACENAV_INCLUDE_DIR)
mark_as_advanced(SPACENAV_LIBRARY_DIR)
mark_as_advanced(SPACENAV_LIBRARY)
