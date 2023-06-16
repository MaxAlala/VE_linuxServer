cmake_minimum_required(VERSION 3.0.0)
project(server VERSION 0.1.0)
set(PROJECT_NAME "server")
set(OPENSSL_ROOT_DIR "/usr/lib/ssl")
include(CTest)
enable_testing()
# Enable C++11
# set(CMAKE_CXX_STANDARD 11)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
#tcpServer.cpp
add_executable(server main.cpp PixelToMotorStepsConverter.cpp FaceDetector.cpp RobotSystem.cpp ServerController.cpp MotorController.cpp VoiceController.cpp Eye.cpp FaceDetector.cpp InverseForwardKinematicsModel.cpp Eye.cpp protocol.hpp WebServer.cpp)
target_link_libraries(server -I/usr/include -lwiringPi)
include_directories(-I/usr/include)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
include_directories(-I${PROJECT_SOURCE_DIR})
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
find_package(Eigen3 REQUIRED)

link_directories(${Eigen_INCLUDE_DIRS})
# set(Boost_USE_STATIC_LIBS OFF) 
# set(Boost_USE_MULTITHREADED ON)  
# set(Boost_USE_STATIC_RUNTIME OFF) 
# find_package(Boost 1.45.0 COMPONENTS system ) 
find_package( Boost 1.70 REQUIRED COMPONENTS thread filesystem system )
cmake_minimum_required(VERSION 3.1)
find_package(PkgConfig REQUIRED)

# Include dir
# find_path(Jsoncpp_INCLUDE_DIR
#   NAMES json/features.h
#   PATH_SUFFIXES jsoncpp
#   PATHS ${Jsoncpp_PKGCONF_INCLUDE_DIRS} # /usr/include/jsoncpp/json
# )
# include_directories(${Jsoncpp_INCLUDE_DIR})
pkg_check_modules(JSONCPP jsoncpp)

# find_package(Jsoncpp REQUIRED)

SET(OpenCV_DIR /home/pi/Desktop/installation/OpenCV-4.7.0/lib/cmake/opencv4)
find_package( OpenCV REQUIRED )
find_package(ZLIB)
find_package(OpenSSL REQUIRED)
# find_package(Drogon REQUIRED)
find_package(Drogon CONFIG REQUIRED)
find_package(Trantor CONFIG REQUIRED)
# find_package(UUID REQUIRED)
# find_package(UUID REQUIRED)
# find_package(Brotli)
add_definitions(-DBOOST_ERROR_CODE_HEADER_ONLY)
add_definitions(-DBOOST_SYSTEM_NO_DEPRECATED)

include(DrogonUtilities)
include_directories(backend)
drogon_create_views(server backend/templates ${CMAKE_CURRENT_BINARY_DIR}/backend)
file(COPY configs/webConfig.json DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
#define 
if(Boost_FOUND)
    include_directories( ${Boost_INCLUDE_DIRS})
    link_directories( ${Boost_LIBRARY_DIRS})
endif()

include_directories( ${OpenCV_INCLUDE_DIRS} /usr/local/include/drogon)
# ${UUID_LIBRARIES}  Brotli_lib
target_link_libraries( server -lboost_system -lboost_filesystem -lboost_thread Trantor::Trantor Drogon::Drogon std::filesystem ${OpenCV_LIBS}  Eigen3::Eigen ${Eigen_LIBRARIES} ${JSONCPP_LIBRARIES} Jsoncpp_lib /usr/local/lib/aarch64-linux-gnu/libdrogon.a ${CMAKE_DL_LIBS} ZLIB::ZLIB)

