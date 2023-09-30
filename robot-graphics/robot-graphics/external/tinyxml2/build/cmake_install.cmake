# Install script for directory: C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/tinyxml2")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "tinyxml2_development" OR NOT CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/Debug/tinyxml2.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/Release/tinyxml2.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/MinSizeRel/tinyxml2.lib")
  elseif(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/RelWithDebInfo/tinyxml2.lib")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "tinyxml2_development" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2/tinyxml2-static-targets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2/tinyxml2-static-targets.cmake"
         "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2/tinyxml2-static-targets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2/tinyxml2-static-targets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets.cmake")
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets-debug.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets-minsizerel.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets-relwithdebinfo.cmake")
  endif()
  if(CMAKE_INSTALL_CONFIG_NAME MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/CMakeFiles/Export/a801c02ec1fcb42ccc21a747b2503e91/tinyxml2-static-targets-release.cmake")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "tinyxml2_development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/tinyxml2" TYPE FILE FILES
    "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/cmake/tinyxml2-config.cmake"
    "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/tinyxml2-config-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "tinyxml2_development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/tinyxml2.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "tinyxml2_development" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/tinyxml2.pc")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "C:/Users/Ocanath/OcanathProj/VS/robot-graphics/robot-graphics/robot-graphics/external/tinyxml2/build/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
