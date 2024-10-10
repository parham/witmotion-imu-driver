
cmake_minimum_required(VERSION 3.10)

project(FODCameraDriver LANGUAGES CXX C)

message(STATUS "=========== WitMotion IMU Driver ============")
message(STATUS "== Developer  | Parham Nooralishahi        ==")
message(STATUS "== Contact | parham.nooralishahi@gmail.com ==")
message(STATUS "== Organization | PHM66                    ==")
message(STATUS "=============================================")

if(NOT ${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    message(FATAL_ERROR "This library is only supports Linux")
endif ()

# Make the directory for logs
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/logs)

############# Build Environment #################
set(CMAKE_CXX_FLAGS "-std=c++17 -pthread -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")
set(CMAKE_C_COMPILER "gcc")
set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-D_GNU_SOURCE)

set(WMID_LIB_NAME phmwmid)

######################################
#### Source code Definition and Source code indexing
######################################
message("Indexing source files ...")
file(GLOB_RECURSE WMID_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE WMID_LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.h)

file(GLOB_RECURSE WM_LIB_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/witmotion/*.c)
file(GLOB_RECURSE WM_LIB_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/witmotion/*.h)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(${WMID_LIB_NAME} SHARED
    ${WM_LIB_SOURCES}
    ${WM_LIB_HEADERS}
    ${WMID_LIB_SOURCES}
    ${WMID_LIB_HEADERS}
        include/Serial.h
        src/Serial.cpp
)

target_include_directories(${WMID_LIB_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_compile_options(${WMID_LIB_NAME}
        PUBLIC
        -fstack-protector-all
        PRIVATE
        -fsigned-char
)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8)
        # Must use std::experimental namespace if older than GCC8
        message("GCC version less than 8. Using std::experimental namespace.")
        target_compile_definitions(${WMID_LIB_NAME} PRIVATE USE_EXPERIMENTAL_FS)
    endif()

    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
        # Must explicitly link separate std::filesystem if older than GCC9
        message("GCC version less than 9. Explicitly linking separate std::filesystem library.")
        target_link_libraries(${WMID_LIB_NAME} PRIVATE stdc++fs)
    endif()
endif()

#####################################
#### Required Libraries
#####################################

# Boost
find_package(Boost REQUIRED)
SET(Boost_USE_STATIC_LIBS ON)
FIND_PACKAGE(Boost COMPONENTS log REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${WMID_LIB_NAME} PRIVATE ${Boost_LIBRARIES})

#####################################

install(TARGETS ${WMID_LIB_NAME} DESTINATION .)

