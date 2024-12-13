# Find the GUROBI include directory
find_path(GUROBI_INCLUDE_DIRS
  NAMES gurobi_c++.h
  PATHS "C:/gurobi1200/win64/include"
)

# Find the GUROBI libraries
find_library(GUROBI_LIBRARY
  NAMES gurobi gurobi100 gurobi110 gurobi120
  PATHS "C:/gurobi1200/win64/lib"
)

if(MSVC)
    set(MSVC_YEAR "2017")
    
    if(MT)
        set(M_FLAG "mt")
    else()
        set(M_FLAG "md")
    endif()
    
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++${M_FLAG}${MSVC_YEAR}
        PATHS "C:/gurobi1200/win64/lib"
    )
    find_library(GUROBI_CXX_DEBUG_LIBRARY
        NAMES gurobi_c++${M_FLAG}d${MSVC_YEAR}
        PATHS "C:/gurobi1200/win64/lib"
    )
else()
    find_library(GUROBI_CXX_LIBRARY
        NAMES gurobi_c++
        PATHS "C:/gurobi1200/win64/lib"
    )
    set(GUROBI_CXX_DEBUG_LIBRARY ${GUROBI_CXX_LIBRARY})
endif()

# Check if the include directory and libraries were found
if(NOT GUROBI_INCLUDE_DIRS)
  message(FATAL_ERROR "GUROBI include directory not found")
endif()

if(NOT GUROBI_LIBRARY)
  message(FATAL_ERROR "GUROBI library not found")
endif()

if(NOT GUROBI_CXX_LIBRARY)
  message(FATAL_ERROR "GUROBI C++ library not found")
endif()

if(NOT GUROBI_CXX_DEBUG_LIBRARY)
  message(FATAL_ERROR "GUROBI C++ debug library not found")
endif()

# Set the include directories and libraries
set(GUROBI_INCLUDE_DIRS ${GUROBI_INCLUDE_DIRS})
set(GUROBI_LIBRARIES ${GUROBI_LIBRARY} ${GUROBI_CXX_LIBRARY} ${GUROBI_CXX_DEBUG_LIBRARY})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG GUROBI_LIBRARY)