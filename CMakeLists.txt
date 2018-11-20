cmake_minimum_required(VERSION 3.5)
project(PointCloudViewer)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")
set(QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/icon.qrc)

find_package(Qt5Widgets CONFIG REQUIRED)
find_package(Qt5Core CONFIG REQUIRED)
find_package(Qt5Gui CONFIG REQUIRED)
find_package(OpenSceneGraph 3.2 REQUIRED COMPONENTS osgGA osgViewer osgDB osgUtil osg osgQt osgFX)
qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})

include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${OPENSCENEGRAPH_INCLUDE_DIRS})
file(GLOB HEAD_FILES "include/*.h")
aux_source_directory(src SOURCE_FILES)

add_executable(${PROJECT_NAME}
        ${SOURCE_FILES}
        ${HEAD_FILES}
        ${QRC_FILES})

target_link_libraries(${PROJECT_NAME}
        ${OPENSCENEGRAPH_LIBRARIES}
        Qt5::Widgets
        Qt5::Core
        )
#qt5_use_modules(${PROJECT_NAME} Core Widgets OpenGL Xml Gui)