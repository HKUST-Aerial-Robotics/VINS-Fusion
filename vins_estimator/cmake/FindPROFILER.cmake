# - Find libprofiler
# - This module determines the libprofiler library of the system
# the vairable can be set
#   LIBPROFILER_STATIC_LINK set true for static link library 为TRUE是要求静态连接
# The following variables are set if the library found:
# LIBPROFILER_FOUND - If false do nnt try to use libprofiler.
# LIBPROFILER_INCLUDE_DIRS - where to find the headfile of library.include文件夹位置
# LIBPROFILER_LIBRARY_DIRS - where to find the libprofiler library.profiler库所在位置
# LIBPROFILER_LIBRARIES, the library file name needed to use libprofiler.profiler库及所有依赖库列表
# LIBPROFILER_LIBRARY - the library needed to use libprofiler. profiler库全路径
# imported target
#   gperftools::profiler

if(LIBPROFILER_FOUND)
    return()
endif()
include (depcommon)
# linux系统下调用pkg-config查找profiler
if (NOT WIN32)
    include(FindPkgConfig)
    unset(_verexp)
    if(LIBPROFILER_FIND_VERSION)
        if(LIBPROFILER_FIND_VERSION_EXACT)
            set(_verexp "=${LIBPROFILER_FIND_VERSION}")
        else()
            set(_verexp ">=${LIBPROFILER_FIND_VERSION}")
        endif()
    endif()
    pkg_check_modules (LIBPROFILER libprofiler${_verexp})
endif()

if (NOT LIBPROFILER_FOUND)
    # windows系统下通过查找头文件 gperftools/profiler.h和find_library 查找profiler来实现
    # find the headfile of library
    set (PROFILER_HEADS gperftools/profiler.h)
    find_path (LIBPROFILER_INCLUDE_DIRS ${PROFILER_HEADS})

    set (PROFILER_NAMES ${PROFILER_NAMES} profiler)
    find_library (LIBPROFILER_LIBRARY NAMES ${PROFILER_NAMES})

    # just find one of dependency, guess other one.
    if (NOT LIBPROFILER_LIBRARY AND LIBPROFILER_INCLUDE_DIRS)
        message ("We just find the headfile, try to guess the library location.")
        set (LIBPROFILER_LIBRARY_DIRS "${LIBPROFILER_INCLUDE_DIRS}/../lib")
        find_library (LIBPROFILER_LIBRARY NAMES ${PROFILER_NAMES} PATHS ${LIBPROFILER_LIBRARY_DIRS})
    elseif (NOT LIBPROFILER_INCLUDE_DIRS AND LIBPROFILER_LIBRARY)
        message ("We just find the lib file, try to guess the include location.")
        get_filename_component(LIBPROFILER_LIBRARY_DIRS ${LIBPROFILER_LIBRARY} DIRECTORY)
        find_path (LIBPROFILER_INCLUDE_DIRS ${PROFILER_HEADS} "${LIBPROFILER_LIBRARY_DIRS}../included")
    endif()

    # find the library.
    if (LIBPROFILER_INCLUDE_DIRS AND LIBPROFILER_LIBRARY)
        if (NOT LIBPROFILER_LIBRARY_DIRS)
            get_filename_component(LIBPROFILER_LIBRARY_DIRS ${LIBPROFILER_LIBRARY} DIRECTORY)
        endif ()
        list(APPEND profiler pthread)
    endif()
else ()
    list(GET MGNCS_LIBRARIES 0 _name)
    find_library (LIBPROFILER_LIBRARY NAMES ${LIBPROFILER_LIBRARIES} PATHS ${LIBPROFILER_LIBRARY_DIRS})
endif()
# handle the QUIETLY and REQUIRED arguments and set LIBPROFILER_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(LIBPROFILER DEFAULT_MSG    LIBPROFILER_LIBRARIES LIBPROFILER_INCLUDE_DIRS)

if(LIBPROFILER_FOUND)
    set(_static_libname libtcmalloc_and_profiler.a)
    find_library (LIBPROFILER_STATIC_LIBRARY NAMES ${_static_libname} PATHS ${LIBPROFILER_LIBRARY_DIRS})
    if(NOT LIBPROFILER_FIND_QUIETLY)
        message(STATUS "  -I: ${LIBPROFILER_INCLUDE_DIRS}")
        message(STATUS "  -L: ${LIBPROFILER_LIBRARY_DIRS}")
        message(STATUS "  -l: ${LIBPROFILER_LIBRARIES}")
    endif()
    find_library (LIBPROFILER_LIBRARY NAMES ${LIBPROFILER_LIBRARIES} PATHS ${LIBPROFILER_LIBRARY_DIRS})

    # create imported target
    if (NOT TARGET gperftools::profiler)
        add_library(gperftools::profiler INTERFACE IMPORTED)
        if(LIBPROFILER_STATIC_LINK)
            # for linking static profiler,must use libtcmalloc_and_profiler.a,see also README gperftools
            set(_link_libs ${LIBPROFILER_STATIC_LDFLAGS})
            if(NOT LIBPROFILER_STATIC_LIBRARY)
                message(FATAL_ERROR "NOT FOUND static library for profiler:${_static_libname} ")
            endif()
            # 替换 profiler 为 :libtcmalloc_and_profiler.a
            string(REPLACE profiler :${_static_libname} _link_libs "${_link_libs}")
        else()
            set(_link_libs ${LIBPROFILER_LDFLAGS})
        endif()
        set_target_properties(gperftools::profiler PROPERTIES
                INTERFACE_COMPILE_OPTIONS "${LIBPROFILER_CFLAGS_OTHER}"
                INTERFACE_INCLUDE_DIRECTORIES "${LIBPROFILER_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES   "${_link_libs}"
                )
        if(NOT LIBPROFILER_FIND_QUIETLY)
            message(STATUS "IMPORTED TARGET: gperftools::profiler,link libraies ${_link_libs}")
        endif()
    endif ()

endif(LIBPROFILER_FOUND)
