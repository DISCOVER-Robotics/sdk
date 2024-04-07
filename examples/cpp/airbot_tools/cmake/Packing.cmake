# these are cache variables, so they could be overwritten with -D,
set(CPACK_PACKAGE_NAME
    ${PROJECT_NAME}
    CACHE STRING "The resulting package name")

set(CPACK_PACKAGE_VENDOR "DISCOVER Robotics Limited")

set(CPACK_VERBATIM_VARIABLES YES)

set(CPACK_PACKAGE_INSTALL_DIRECTORY ${CPACK_PACKAGE_NAME})
set(CPACK_OUTPUT_FILE_PREFIX "${CMAKE_SOURCE_DIR}")

# https://unix.stackexchange.com/a/11552/254512
# set(CPACK_PACKAGING_INSTALL_PREFIX "/opt/some") # /${CMAKE_PROJECT_VERSION}")

# set(CPACK_PACKAGE_VERSION_MAJOR ${PROJECT_VERSION_MAJOR})
# set(CPACK_PACKAGE_VERSION_MINOR ${PROJECT_VERSION_MINOR})
# set(CPACK_PACKAGE_VERSION_PATCH ${PROJECT_VERSION_MINOR})
set(CPACK_DEBIAN_PACKAGE_VERSION ${VERSION_STR})

set(CPACK_PACKAGE_CONTACT "george@discover-robotics.com")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "Beiwen Tian")

# set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_RESOURCE_FILE_README "${CMAKE_CURRENT_SOURCE_DIR}/README.md")

# package name for deb. If set, then instead of some-application-0.9.2-Linux.deb
# you'll get some-application_0.9.2_amd64.deb (note the underscores too)
set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

# that is if you want every group to have its own package, although the same
# will happen if this is not set (so it defaults to ONE_PER_GROUP) and
# CPACK_DEB_COMPONENT_INSTALL is set to YES
set(CPACK_COMPONENTS_GROUPING ALL_COMPONENTS_IN_ONE) # ONE_PER_GROUP)
# without this you won't be able to pack only specified component
set(CPACK_DEB_COMPONENT_INSTALL YES)
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "airbot_play (>= 2.6.1)")

include(CPack)
