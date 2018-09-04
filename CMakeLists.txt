cmake_minimum_required(VERSION 2.8.3)
project(humanoid)

find_package(Eigen3 REQUIRED)

find_package(PythonLibs 2.7 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR} ${PYTHON_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#for test
#set(SOURCE_FILES main.cpp util/Riccati.cpp util/Riccati.h patten/PreviewController.cpp patten/PreviewController.h trajectory/TrajectoryGenerator.cpp trajectory/TrajectoryGenerator.h ZMP.cpp ZMP.h FootTrajectory.cpp FootTrajectory.h Common.h test/matplotlibcpp.h test/utest.cpp test/utest.h Cycloid.cpp Cycloid.h CycloidFootTrajectory.cpp CycloidFootTrajectory.h)
#set(SOURCE_FILES main.cpp)

#for library
set(SOURCE_FILES util/Riccati.cpp patten/PreviewController.cpp trajectory/TrajectoryGenerator.cpp ZMP.cpp
        FootTrajectory.cpp test/utest.cpp Cycloid.cpp CycloidFootTrajectory.cpp OrientationTrajectory.cpp)

set(INCLUDE_FILES util/Riccati.h patten/PreviewController.h  trajectory/TrajectoryGenerator.h  ZMP.h
         FootTrajectory.h Common.h test/matplotlibcpp.h test/utest.h Cycloid.h CycloidFootTrajectory.h OrientationTrajectory.h)

set(CONFIG_FILES HumanoidConfig.cmake)

set(CMAKE_BUILD_TYPE Release)
add_library(humanoid SHARED ${SOURCE_FILES} ${INCLUDE_FILES})

if(UNIX AND NOT APPLE)
    #install header file
    foreach ( file ${INCLUDE_FILES} )
        get_filename_component( dir ${file} DIRECTORY )
        install( FILES ${file} DESTINATION /usr/local/include/humanoid/${dir} )
    endforeach()

    #install shared library
    install(TARGETS humanoid DESTINATION /usr/local/lib)

    #install config file
    install( FILES ${CONFIG_FILES} DESTINATION /usr/lib/cmake/humanoid )
endif()

target_link_libraries(humanoid ${PYTHON_LIBRARIES})

#for test
#add_executable(humanoid ${SOURCE_FILES})
#
#target_link_libraries(humanoid ${PYTHON_LIBRARIES})