cmake_minimum_required (VERSION 2.8.5 FATAL_ERROR)
add_compile_options(-std=c++0x)

set(ProjectName Sample-01-cvShowImage)
project(${ProjectName})
set(Project_SRC Sample.cpp ObstacleScenario.cpp PathPlaner.cpp Simulator.cpp)
add_executable(${ProjectName} ${Project_SRC}) 
if(WIN32)
	find_package(OpenCV REQUIRED HINTS C:/Dev/OpenCV/2.4.10)
	file(TO_NATIVE_PATH ${OpenCV_LIB_PATH}/../bin/opencv_core2410.dll SRC)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
	file(TO_NATIVE_PATH ${OpenCV_LIB_PATH}/../bin/opencv_highgui2410.dll SRC)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
	file(TO_NATIVE_PATH ${OpenCV_LIB_PATH}/../bin/opencv_imgproc2410.dll SRC)
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy ${SRC} ${DST}) 
elseif(UNIX OR APPLE)
	find_package(OpenCV REQUIRED)
endif()
target_link_libraries(${ProjectName} ${DUO_LIBRARY} ${OpenCV_LIBS}) 


find_package( PCL 1.2 REQUIRED )

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)

find_package(Boost COMPONENTS filesystem REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})



if(WIN32)
	target_link_libraries(${ProjectName} DUOlib ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
elseif(UNIX OR APPLE)
	target_link_libraries(${ProjectName} duo ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES})
endif()

message("Setting up installation")
if(UNIX)
    get_filename_component(PARENT_DIR ${CMAKE_CURRENT_BINARY_DIR} DIRECTORY)
    file(MAKE_DIRECTORY var/lib/autonomousCar)
    install(DIRECTORY ${PARENT_DIR}/InstallData/ DESTINATION var/lib/autonomousCar)
elseif(WIN32)
endif()
