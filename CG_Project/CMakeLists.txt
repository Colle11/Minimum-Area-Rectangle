# Created by the script cgal_create_cmake_script
# This is the CMake script for compiling a CGAL application.

project (CG_Project)

cmake_minimum_required(VERSION 3.1)
if(NOT POLICY CMP0070 AND POLICY CMP0053)
  # Only set CMP0053 to OLD with CMake<3.10, otherwise there is a warning.
  cmake_policy(SET CMP0053 OLD)
endif()

if(POLICY CMP0071)
  cmake_policy(SET CMP0071 NEW)
endif()

find_package(CGAL COMPONENTS Qt5)

find_package(Qt5 QUIET COMPONENTS Xml Script OpenGL Svg)
include_directories (BEFORE ./include)
include_directories (BEFORE ../../../Matrix_search/include)


if ( CGAL_FOUND AND CGAL_Qt5_FOUND AND Qt5_FOUND )

set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

#----------------------------------------------
# CG Project
#----------------------------------------------
# UI files (Qt Designer files)
qt5_wrap_ui( DT_UI_FILES CG_Project.ui )

# qrc files (resources files, that contain icons, at least)
qt5_add_resources ( CGAL_Qt5_RESOURCE_FILES ./CG_Project.qrc )

# use the Qt MOC preprocessor on classes that derives from QObject
qt5_generate_moc( "CG_Project.cpp" "${CMAKE_CURRENT_BINARY_DIR}/CG_Project.moc" )

# The executable itself.
add_executable  ( CG_Project CG_Project.cpp CG_Project.moc ${DT_UI_FILES} ${CGAL_Qt5_RESOURCE_FILES} ${CGAL_Qt5_MOC_FILES})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS CG_Project )

target_link_libraries( CG_Project PRIVATE
  CGAL::CGAL CGAL::CGAL_Qt5 Qt5::Gui)

include(${CGAL_MODULES_DIR}/CGAL_add_test.cmake)
cgal_add_compilation_test(CG_Project)

include(${CGAL_MODULES_DIR}/CGAL_add_test.cmake)
cgal_add_compilation_test(CG_Project)

else()

  message(STATUS "NOTICE: CG Project requires CGAL and Qt5, and will not be compiled.")

endif()
