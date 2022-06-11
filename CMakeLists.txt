cmake_minimum_required(VERSION 3.0.0)
project(server VERSION 0.1.0)

include(CTest)
enable_testing()
#tcpServer.cpp
add_executable(server tcpServer.cpp MotorController.cpp)
target_link_libraries(server -I/usr/include -lpthread -lboost_system -lwiringPi)
include_directories(-I/usr/include)
set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.45.0 COMPONENTS system thread) 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries( server ${OpenCV_LIBS} )

if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS}) 
    target_link_libraries(server ${Boost_LIBRARIES})
endif()
