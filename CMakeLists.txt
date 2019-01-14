cmake_minimum_required(VERSION 3.3)
project(PointCloudViewer)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(EXECUTABLE_OUTPUT_PATH "${CMAKE_SOURCE_DIR}/bin")
set(QRC_SOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/src/icon.qrc)

find_package(Qt5Widgets 5.11 CONFIG REQUIRED)
find_package(Qt5Core 5.11 CONFIG REQUIRED)
find_package(Qt5Gui 5.11 CONFIG REQUIRED)
find_package(Qt5Network 5.11 CONFIG REQUIRED)
qt5_add_resources(QRC_FILES ${QRC_SOURCE_FILES})
find_package(OpenSceneGraph 3.2 REQUIRED COMPONENTS osgGA osgViewer osgDB osgUtil osgText osg osgQt osgFX)
find_package(Draco REQUIRED)
find_package(libssh REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(libLAS REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        ${CMAKE_SOURCE_DIR}/include
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
        ${YAML_CPP_INCLUDE_DIR}
        ${libLAS_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        )

file(GLOB HEAD_FILES "include/*.h")
aux_source_directory(src SOURCE_FILES)

add_executable(${PROJECT_NAME}
        ${SOURCE_FILES}
        ${HEAD_FILES}
        ${QRC_FILES}
        )

target_link_libraries(${PROJECT_NAME}
        Qt5::Gui
        Qt5::Widgets
        Qt5::Core
        Qt5::Network
        ${OPENSCENEGRAPH_LIBRARIES}
        draco dracoenc dracodec
        ${LIBSSH_LIBRARIES}
        yaml-cpp
        las
        )