# Check if the Git workspace is clean
if(GIT_CLEAN_CHECK)
  execute_process(
    COMMAND git status --porcelain
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_STATUS
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  
  # If the output variable is not empty, there are uncommitted changes
  if(NOT "${GIT_STATUS}" STREQUAL "")
  message(FATAL_ERROR "Git workspace is not clean. Please commit or stash your changes before proceeding.")
  endif()
else()
  message(WARNING "Git workspace check is disabled.")
endif()

execute_process(
  COMMAND git log -1 --format=%h
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE GIT_HASH
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
if(DEFINED ENV{CI_COMMIT_TAG})
  # If CI_COMMIT_TAG exists, use its value for VERSION_STR
  set(VERSION_STR $ENV{CI_COMMIT_TAG})
else()
  # If CI_COMMIT_TAG does not exist, find git commit hash and use it for VERSION_STR
  if(GIT_CLEAN_CHECK)
    set(VERSION_STR ${PROJECT_VERSION}-${GIT_HASH})
  else()
    set(VERSION_STR ${PROJECT_VERSION}-${GIT_HASH}-dirty)
  endif()
endif()

message(STATUS "Git hash: ${VERSION_STR}")