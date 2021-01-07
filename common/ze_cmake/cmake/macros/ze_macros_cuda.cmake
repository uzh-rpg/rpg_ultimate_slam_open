# finding cuda and adding compile flags
macro(find_cuda)
  cmake_parse_arguments(add_args "REQUIRED;NOT_AUTOMATIC" "" "" ${ARGN})

  if(add_args_REQUIRED)
    find_package(CUDA REQUIRED)
  else()
    find_package(CUDA)
  endif()

  message(STATUS "CUDA_FOUND: ${CUDA_FOUND}")
  message(STATUS "CUDA_VERSION_STRING: ${CUDA_VERSION_STRING}")
  message(STATUS "CUDA_VERSION_MAJOR: ${CUDA_VERSION_MAJOR}")
  
  if ((NOT ${CUDA_FOUND}) OR (${CUDA_VERSION_MAJOR} LESS 7))
    message(WARNING "CUDA not found or too old CUDA version (CUDA_VERSION_MAJOR < 7). Skipping this package.")
    return()
  endif()
  add_definitions(-DWITH_CUDA)

  #list (APPEND CUDA_NVCC_FLAGS -relaxed-constexpr -use_fast_math -std=c++11)
  list (APPEND CUDA_NVCC_FLAGS -use_fast_math -std=c++11)
  #list (APPEND CUDA_NVCC_FLAGS --compiler-options;-fno-strict-aliasing;)
  list (APPEND CUDA_NVCC_FLAGS --compiler-options;-fPIC;)
  list (APPEND CUDA_NVCC_FLAGS --compiler-options;-Wall;)
  #list (APPEND CUDA_NVCC_FLAGS --compiler-options;-Werror;)
  if(CMAKE_BUILD_TYPE MATCHES Debug)
    list (APPEND CUDA_NVCC_FLAGS --device-debug)
    list (APPEND CUDA_NVCC_FLAGS --compiler-options;-g;)
    list (APPEND CUDA_NVCC_FLAGS --compiler-options;-rdynamic;)
    list (APPEND CUDA_NVCC_FLAGS --compiler-options;-lineinfo;)
    list (APPEND CUDA_NVCC_FLAGS --ptxas-options=-v;)
  endif()

  #list (APPEND CUDA_NVCC_FLAGS --device-c)
  #list (APPEND CUDA_NVCC_FLAGS -rdc=true)

  set(CUDA_SEPARABLE_COMPILATION OFF)
  # set to OFF cuda files are added to multiple targets
  set(CUDA_ATTACH_VS_BUILD_RULE_TO_CUDA_FILE ON)
  set(BUILD_SHARED_LIBS ON)

  # nvcc and ccache are not very good friends, hence we set the host compiler
  # for cuda manually if ccache is enabled.
  ##! @todo (MWE) find/check the path of the compiler as well as the OS
  get_filename_component(COMPILER_EXE ${CMAKE_C_COMPILER} REALPATH)
  message(STATUS "CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")
  message(STATUS "COMPILER_EXE: ${COMPILER_EXE}")
  string(REGEX MATCH "ccache" SYSTEM_USE_CCACHE "${COMPILER_EXE}")
  if(SYSTEM_USE_CCACHE)
    if(EXISTS "${CUDA_TOOLKIT_ROOT_DIR}/bin/gcc")
      set(CUDA_HOST_COMPILER ${CUDA_TOOLKIT_ROOT_DIR}/bin/gcc)
    elseif(EXISTS "/usr/bin/gcc")
      set(CUDA_HOST_COMPILER /usr/bin/gcc)
    endif()
    MESSAGE(STATUS "CUDA_HOST_COMPILER: ${CUDA_HOST_COMPILER}")
  endif()

  include_directories(
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_SDK_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../cuda_toolkit/${CUDA_VERSION_STRING}/include
    )

  # Checking cuda version
  if(CUDA_VERSION_STRING STREQUAL "7.0")
    # CUDA 7.0
    add_definitions(-DCUDA_VERSION_70)
  elseif(CUDA_VERSION_STRING STREQUAL "7.5")
    # CUDA 7.5
    add_definitions(-DCUDA_VERSION_75)
  elseif(CUDA_VERSION_STRING STREQUAL "8.0")
    # CUDA 8.0
    add_definitions(-DCUDA_VERSION_80)
  else()
    message(FATAL_ERROR "unknown CUDA version. some things might not be tested.")
  endif()


  # Selection of compute capability via environment variable
  if("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "1.1")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_11)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "1.2")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_12)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "1.3")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_13)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "2.0")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_20)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_20,code=sm_20)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "2.1")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_21)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "3.0")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_30)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_30,code=sm_30)
  elseif("$ENV{NV_COMPUTE_CAPABILITY}" MATCHES "3.5")
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_35)
    list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_35,code=sm_35)
  else()
    list(APPEND CUDA_NVCC_FLAGS -arch=sm_30)
  endif()


  # define can be used to define exceptions and throw it if a CUDA error is caught by the error-check
  ##! @todo (MWE) make this configurable
  add_definitions(-DTHROW_ON_CUDA_ERROR)
  
endmacro()

##------------------------------------------------------------------------------
# macro for adding cuda executables in the style of catkin_simple
macro(cs_cuda_add_executable _target)
  if(${_target} STREQUAL ${PROJECT_NAME}_package)
    message(WARNING "Could not create executable with name '${_target}' as '${PROJECT_NAME}_package' is reserved for the top level target name for this project.")
  endif()
  cmake_parse_arguments(cs_cuda_add_executable_args "NO_AUTO_LINK;NO_AUTO_DEP" "" "" ${ARGN})
  cuda_add_executable(${_target} ${cs_cuda_add_executable_args_UNPARSED_ARGUMENTS})
  if(NOT cs_cuda_add_executable_args_NO_AUTO_LINK)
    target_link_libraries(${_target} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})
  endif()
  if(NOT cs_cuda_add_executable_args_NO_AUTO_DEP)
    if(NOT "${${PROJECT_NAME}_CATKIN_BUILD_DEPENDS_EXPORTED_TARGETS}" STREQUAL "")
      add_dependencies(${_target} ${${PROJECT_NAME}_CATKIN_BUILD_DEPENDS_EXPORTED_TARGETS})
    endif()
  endif()
  cs_add_targets_to_package(${_target})
endmacro()

##------------------------------------------------------------------------------
# macro for adding cuda library in the style of catkin_simple
macro(cs_cuda_add_library _target)
  if(${_target} STREQUAL ${PROJECT_NAME}_package)
    message(WARNING "Could not create library with name '${_target}' as '${PROJECT_NAME}_package' is reserved for the top level target name for this project.")
  endif()
  cmake_parse_arguments(cs_cuda_add_library "NO_AUTO_LINK;NO_AUTO_DEP;NO_AUTO_EXPORT" "" "" ${ARGN})
  cuda_add_library(${_target} ${cs_cuda_add_library_UNPARSED_ARGUMENTS})
  if(NOT cs_cuda_add_library_NO_AUTO_LINK)
    target_link_libraries(${_target} ${CUDA_LIBRARIES} ${catkin_LIBRARIES})
  endif()
  if(NOT cs_cuda_add_library_NO_AUTO_DEP)
    if(NOT "${${PROJECT_NAME}_CATKIN_BUILD_DEPENDS_EXPORTED_TARGETS}" STREQUAL "")
      add_dependencies(${_target} ${${PROJECT_NAME}_CATKIN_BUILD_DEPENDS_EXPORTED_TARGETS})
    endif()
  endif()
  if(NOT cs_cuda_add_library_NO_AUTO_EXPORT)
    list(APPEND ${PROJECT_NAME}_LIBRARIES ${_target})
  endif()
  cs_add_targets_to_package(${_target})
endmacro()