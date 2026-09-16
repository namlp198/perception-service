cmake_minimum_required(VERSION 3.22)

project(
    perception-service
    VERSION 0.1.0
    DESCRIPTION "Local perception and localization service for NVIDIA Jetson Orin"
    LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)

option(PERCEPTION_ENABLE_REALSENSE "Build the Intel RealSense camera backend" OFF)
option(PERCEPTION_ENABLE_GSTREAMER "Build the GStreamer RTSP backend" OFF)
option(PERCEPTION_ENABLE_OPENCV_TOOLS "Build local OpenCV viewer tools" OFF)
option(PERCEPTION_REQUIRE_PRODUCTION_DEPENDENCIES
       "Require the complete declared V0.1 dependency baseline" OFF)
option(PERCEPTION_WARNINGS_AS_ERRORS "Treat project warnings as errors" ON)
set(PERCEPTION_DEPENDENCY_ROOT "" CACHE PATH "Optional root containing locally extracted packages")

add_library(perception_warnings INTERFACE)
target_compile_features(perception_warnings INTERFACE cxx_std_20)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(perception_warnings INTERFACE -Wall -Wextra -Wpedantic -Wconversion
                                                          -Wshadow)
    if(PERCEPTION_WARNINGS_AS_ERRORS)
        target_compile_options(perception_warnings INTERFACE -Werror)
    endif()
elseif(MSVC)
    target_compile_options(perception_warnings INTERFACE /W4)
    if(PERCEPTION_WARNINGS_AS_ERRORS)
        target_compile_options(perception_warnings INTERFACE /WX)
    endif()
endif()

add_library(
    perception_core
    src/core/application.cpp
    src/core/config.cpp
    src/camera/realsense_camera.cpp
    src/streaming/rtsp_server.cpp
    src/streaming/stream_manager.cpp
    src/pipeline/capture_pipeline.cpp
    src/health/camera_health.cpp)
add_library(perception::core ALIAS perception_core)

target_include_directories(
    perception_core
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(perception_core PRIVATE perception_warnings)

if(PERCEPTION_REQUIRE_PRODUCTION_DEPENDENCIES)
    find_package(spdlog REQUIRED)
    find_package(yaml-cpp REQUIRED)
    find_package(Eigen3 REQUIRED)
    if(BUILD_TESTING)
        find_package(GTest REQUIRED)
    endif()
else()
    find_package(spdlog QUIET)
    find_package(yaml-cpp QUIET)
    if(BUILD_TESTING)
        find_package(GTest QUIET)
    endif()
endif()

if(spdlog_FOUND)
    target_link_libraries(perception_core PRIVATE spdlog::spdlog)
    target_compile_definitions(perception_core PRIVATE PERCEPTION_HAS_SPDLOG=1)
else()
    message(STATUS "spdlog not found: production logging integration is disabled")
endif()

if(yaml-cpp_FOUND)
    target_link_libraries(perception_core PRIVATE yaml-cpp)
    target_compile_definitions(perception_core PRIVATE PERCEPTION_HAS_YAML_CPP=1)
else()
    message(STATUS "yaml-cpp not found: configuration files cannot be loaded")
endif()

if(PERCEPTION_ENABLE_REALSENSE)
    find_package(realsense2 CONFIG REQUIRED)
    target_link_libraries(perception_core PRIVATE realsense2::realsense2)
    target_compile_definitions(perception_core PRIVATE PERCEPTION_HAS_REALSENSE=1)
endif()

if(PERCEPTION_ENABLE_GSTREAMER)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET gstreamer-1.0 gstreamer-app-1.0)
    target_link_libraries(perception_core PRIVATE PkgConfig::GSTREAMER)
    if(PERCEPTION_DEPENDENCY_ROOT)
        find_path(
            GST_RTSP_SERVER_INCLUDE_DIR
            NAMES gst/rtsp-server/rtsp-server.h
            PATHS "${PERCEPTION_DEPENDENCY_ROOT}/usr/include/gstreamer-1.0"
            NO_DEFAULT_PATH
            REQUIRED)
        find_library(
            GST_RTSP_SERVER_LIBRARY
            NAMES gstrtspserver-1.0
            PATHS "${PERCEPTION_DEPENDENCY_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
            NO_DEFAULT_PATH
            REQUIRED)
        target_include_directories(perception_core PRIVATE "${GST_RTSP_SERVER_INCLUDE_DIR}")
        target_link_libraries(perception_core PRIVATE "${GST_RTSP_SERVER_LIBRARY}")
    else()
        pkg_check_modules(GST_RTSP_SERVER REQUIRED IMPORTED_TARGET gstreamer-rtsp-server-1.0)
        target_link_libraries(perception_core PRIVATE PkgConfig::GST_RTSP_SERVER)
    endif()
    target_compile_definitions(perception_core PRIVATE PERCEPTION_HAS_GSTREAMER=1)
endif()

add_executable(perception-service app/perception_service.cpp)
target_link_libraries(perception-service PRIVATE perception::core perception_warnings)

add_executable(camera-info app/tools/camera_info.cpp)
target_link_libraries(camera-info PRIVATE perception::core perception_warnings)

add_executable(imu-info app/tools/imu_info.cpp)
target_link_libraries(imu-info PRIVATE perception::core perception_warnings)

add_executable(stream-test app/tools/stream_test.cpp)
target_link_libraries(stream-test PRIVATE perception::core perception_warnings)

if(PERCEPTION_ENABLE_OPENCV_TOOLS)
    find_package(OpenCV CONFIG REQUIRED COMPONENTS core highgui imgproc)
    add_executable(camera-viewer app/tools/camera_viewer.cpp)
    target_link_libraries(camera-viewer PRIVATE perception::core perception_warnings ${OpenCV_LIBS})
    target_compile_definitions(camera-viewer PRIVATE PERCEPTION_HAS_OPENCV=1)
endif()

if(BUILD_TESTING)
    add_subdirectory(tests)
endif()

install(TARGETS perception-service camera-info imu-info stream-test
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY config/ DESTINATION ${CMAKE_INSTALL_SYSCONFDIR}/perception-service)
