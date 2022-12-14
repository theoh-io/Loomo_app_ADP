cmake_minimum_required(VERSION 3.4.1)
project(Algo)

#####################################
#config APP module
#####################################
# MACROS for selecting modules
#set(APP_MODULE "TEST")
#set(APP_MODULE "LOCALMAPPING_TEST")
#set(APP_MODULE "TENSORFLOW_SAMPLE")
#set(APP_MODULE "SOCKET")
#set(APP_MODULE "FOLLOW")
#set(APP_MODULE "VITA_TESTING")
set(APP_MODULE "ADP")
#set(APP_MODULE "TENSORFLOW_PERSON_DETECTOR")

#####################################
# Set the compile related flags....
#####################################
set(CMAKE_C_FLAGS "-O3 -march=atom -ffast-math -mtune=atom -mssse3 -mfpmath=sse -funroll-loops")
set(CMAKE_CXX_FLAGS "-std=c++11 -fexceptions -frtti -Wattributes -Werror=return-type" )
set(CMAKE_LIBRARY_LINKER_FLAGS "-flto -fPIE")

#####################################
# Set the directories / paths ....
#####################################

include_directories(${ANDROID_SYSROOT}/usr/include/x86_64-linux-android)

#sdk DIR....
set(ALGO_SDK_DIR "${PROJECT_SOURCE_DIR}/../../../..")
message(" SDK path is ${ALGO_SDK_DIR}")

#depencency DIR....
set(Dependency_DIR "${ALGO_SDK_DIR}/dependency")
message(" Dep path is ${Dependency_DIR}")

#algo lib DIR...
set(ALGO_LIB_DIR "${Dependency_DIR}/algo_lib")
message("algo lib path is ${ALGO_LIB_DIR}")

set(ALGO_INCLUDE_DIR "${Dependency_DIR}/algo_include")
message(" algo include path is ${ALGO_INCLUDE_DIR}")

#3rd party lib DIR...
set(ALGO_3RD_DIR "${Dependency_DIR}/3rdparty_android")
message(" 3rdparty path is ${ALGO_3RD_DIR}")

set(JAVA_MAIN_LIB_DIR "${PROJECT_SOURCE_DIR}/../libs/x86_64/")


###########################################
# for package external libs into java apk #
###########################################
macro(package_lib lib_path)
    set(PACK_LIB_LIST ${PACK_LIB_LIST} ${lib_path})
endmacro()


#####################################
#opencv dir  && ref opencv libs
#####################################
IF(CMAKE_SYSTEM_NAME MATCHES "Android")
set(OpenCV_DIR "${Dependency_DIR}/3rdparty_android/opencv-3.4.0_shared/sdk/native/jni")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
ENDIF()

#####################################
#Switchs for the libraries init
#####################################
option(USING_LIB_LOCAL "Using the libs from local potions is " OFF)
option(USE_TENSORFLOW       "using tensorflow is "             OFF)
option(USE_LOCALMAPPING     "using localmaping is "            OFF)


#####################################
#Switchs for the libraries set
#####################################

string(COMPARE EQUAL ${APP_MODULE} "TEST" USING_TEST_MODULE)
if(USING_TEST_MODULE)
#    set(USE_LIBREALSENSE ON)
endif(USING_TEST_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "LOCALMAPPING_TEST" USING_LOCALMAPPING_TEST_MODULE)
if(USING_LOCALMAPPING_TEST_MODULE)
    set(USE_LOCALMAPPING ON)
endif(USING_LOCALMAPPING_TEST_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "TENSORFLOW_SAMPLE" USING_TENSORFLOW_SAMPLE_MODULE)
if(USING_TENSORFLOW_SAMPLE_MODULE)
    set(USE_TENSORFLOW ON)
endif(USING_TENSORFLOW_SAMPLE_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "SOCKET" USING_SOCKET_MODULE)
if(USING_SOCKET_MODULE)
    set(USE_LOCALMAPPING ON)
#    set(USE_LIBREALSENSE ON)
endif(USING_SOCKET_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "FOLLOW" USING_FOLLOW_MODULE)
if(USING_FOLLOW_MODULE)
    set(USE_LOCALMAPPING ON)
#    set(USE_LIBREALSENSE ON)
endif(USING_FOLLOW_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "ADP" USING_ADP_MODULE)
if(USING_ADP_MODULE)
    set(USE_LOCALMAPPING ON)
    #    set(USE_LIBREALSENSE ON)
endif(USING_ADP_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "VITA_TESTING" USING_VITA_TESTING_MODULE)
if(USING_VITA_TESTING_MODULE)
    set(USE_LOCALMAPPING ON)
#    set(USE_LIBREALSENSE ON)
endif(USING_VITA_TESTING_MODULE)

string(COMPARE EQUAL ${APP_MODULE} "TENSORFLOW_PERSON_DETECTOR" USING_TENSORFLOW_PERSON_DETECTOR_MODULE)
if(USING_TENSORFLOW_PERSON_DETECTOR_MODULE)
    set(USE_TENSORFLOW ON)
endif(USING_TENSORFLOW_PERSON_DETECTOR_MODULE)

#####################################
#link AlgoBase   #maybe as a subdirectory better ?
#####################################
set(ALGO_BASE_DIR "${Dependency_DIR}/algobaseInclude")
add_library( AlgoBase
             SHARED
             IMPORTED )
set_target_properties( # Specifies the target library.
                       AlgoBase
                       # Specifies the parameter you want to define.
                       PROPERTIES IMPORTED_LOCATION
                       # Provides the path to the library you want to import.
                       ${Dependency_DIR}/aar/libAlgoBase.so)
set(LINK_LIBRARIES_LIST "AlgoBase")

link_directories(${ALGO_LIB_DIR})
set(LINK_LIBRARIES_LIST ${LINK_LIBRARIES_LIST} "RealSense")
package_lib("${ALGO_LIB_DIR}/libRealSense.so")

##########################################################################
#using localmapping libs
##########################################################################
if(${USE_LOCALMAPPING} STREQUAL "ON")
    link_directories(${ALGO_LIB_DIR})
    set(LINK_LIBRARIES_LIST ${LINK_LIBRARIES_LIST} "local_mapping")
    package_lib("${ALGO_LIB_DIR}/liblocal_mapping.so")
endif()

#####################################
#using tensorflow
#####################################
if(${USE_TENSORFLOW} STREQUAL "ON")

    set(TENSORFLOW_DIR "${ALGO_3RD_DIR}/Tensorflow")
    set(TENSORFLOW_LIB_DIR "${TENSORFLOW_DIR}/native/x86_64")
    set(TENSORFLOW_INCLUDE_DIR "${TENSORFLOW_DIR}/include/proto" "${TENSORFLOW_DIR}/include" "${TENSORFLOW_DIR}/include/eigen")

    include_directories(${TENSORFLOW_INCLUDE_DIR})
    link_directories(${TENSORFLOW_LIB_DIR})
    SET (TensorflowCoreLib -Wl,--allow-multiple-definition,--whole-archive ${TENSORFLOW_LIB_DIR}/libtensorflow-core.a -Wl,--no-whole-archive)
    set(LINK_LIBRARIES_LIST ${LINK_LIBRARIES_LIST} ${TensorflowCoreLib})
    set(LINK_LIBRARIES_LIST ${LINK_LIBRARIES_LIST} "${TENSORFLOW_LIB_DIR}/libprotobuf.a")

endif()

#####################################
#general files include / src
#####################################

include_directories(
    ${PROJECT_SOURCE_DIR}/core
    ${ALGO_3RD_DIR}
    ${ALGO_INCLUDE_DIR}/general
    ${ALGO_INCLUDE_DIR}/tools/linear_math
    ${ALGO_INCLUDE_DIR}/tools/impl
    ${ALGO_BASE_DIR}/core
    ${ALGO_BASE_DIR}/utils
    ${ALGO_BASE_DIR}/interface
)

set(GENERA_TAGET_FILES
    ${GENERA_TAGET_FILES}
    ${PROJECT_SOURCE_DIR}/core/Entry.cpp
    ${PROJECT_SOURCE_DIR}/core/Algo.cpp
    ${PROJECT_SOURCE_DIR}/core/AlgoImpl.cpp
)

#####################################
#compile the app test
#####################################
if(USING_TEST_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_test)
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_test/AlgoTest.cpp
        ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
    )
    add_definitions("-DAPP_TEST")
endif()

#####################################
#compile the localmapping test
#####################################
if(USING_LOCALMAPPING_TEST_MODULE)
    set(LOOMOGO_NAV_DIR "${PROJECT_SOURCE_DIR}/../../../../loomogo_nav_app/src/main/jni")
    include_directories(${PROJECT_SOURCE_DIR}/app_localmapping_test
                        ${LOOMOGO_NAV_DIR})
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_localmapping_test/AlgoLocalmappingTest.cpp
        ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
    )
    add_definitions("-DAPP_LOCALMAPPING_TEST")
endif()

#####################################
#compile the tensorflow sample
#####################################
if(USING_TENSORFLOW_SAMPLE_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_tensorflow_sample)
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_tensorflow_sample/AlgoTensorflowSample.cpp
    )
    add_definitions("-DAPP_TENSORFLOW_SAMPLE")
endif()

#####################################
#compile the tensorflow person detector
#####################################
if(USING_TENSORFLOW_PERSON_DETECTOR_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_tensorflow_person_detector)
    set(TARGET_FILES
            ${GENERA_TAGET_FILES}
            ${PROJECT_SOURCE_DIR}/app_tensorflow_person_detector/AlgoTensorflowPersonDetector.cpp
            )
    add_definitions("-DAPP_TENSORFLOW_PERSON_DETECTOR")
endif()

#####################################
#compile the app socket
#####################################
if(USING_SOCKET_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_socket)
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_socket/AlgoSocket.cpp
        ${PROJECT_SOURCE_DIR}/app_socket/SocketServer.cpp
        ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
    )
    add_definitions("-DAPP_SOCKET")
endif()

#####################################
#compile the app follow
#####################################
if(USING_FOLLOW_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_follow)
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_follow/AlgoFollow.cpp
        ${PROJECT_SOURCE_DIR}/app_follow/SocketServer.cpp
        ${PROJECT_SOURCE_DIR}/app_follow/PID.cpp
        ${PROJECT_SOURCE_DIR}/app_follow/DepthPreprocess.cpp
        ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
    )
    add_definitions("-DAPP_FOLLOW")
endif()

#####################################
#compile the app ADP
#####################################
if(USING_ADP_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_ADP)
    set(TARGET_FILES
            ${GENERA_TAGET_FILES}
            ${PROJECT_SOURCE_DIR}/app_ADP/AlgoADP.cpp
            ${PROJECT_SOURCE_DIR}/app_ADP/SocketServer.cpp
            ${PROJECT_SOURCE_DIR}/app_ADP/PID.cpp
            ${PROJECT_SOURCE_DIR}/app_ADP/DepthPreprocess.cpp
            ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
            )
    add_definitions("-DAPP_ADP")
endif()

#####################################
#compile the app vita_testing
#####################################
if(USING_VITA_TESTING_MODULE)
    include_directories(${PROJECT_SOURCE_DIR}/app_VITA_testing)
    set(TARGET_FILES
        ${GENERA_TAGET_FILES}
        ${PROJECT_SOURCE_DIR}/app_VITA_testing/VITA_testing.cpp
        ${PROJECT_SOURCE_DIR}/app_VITA_testing/SocketServer.cpp
        ${PROJECT_SOURCE_DIR}/app_VITA_testing/PID.cpp
        ${PROJECT_SOURCE_DIR}/app_VITA_testing/DepthPreprocess.cpp
        ${ALGO_BASE_DIR}/utils/AlgoUtils.cpp
    )
    add_definitions("-DAPP_VITA_TESTING")
endif()

#####################################
#finally compiles it
#####################################
message("LINK_LIBRARIES_LIST: ${LINK_LIBRARIES_LIST}")
add_library(
    ${PROJECT_NAME}
    SHARED
    ${TARGET_FILES}
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${LINK_LIBRARIES_LIST}
    log
    z
    dl
    m
    c
    jnigraphics
    android
)

#########################################
#then give package info to the scpirpts #
#########################################
set(OUTPUT_PATH_OF_COPY_LIBS ${JAVA_MAIN_LIB_DIR})
message("the lib list is ${LIB_LIST}")
configure_file(
    "${PROJECT_SOURCE_DIR}/scripts/lib_list.cfg.in"
    "${PROJECT_SOURCE_DIR}/scripts/lib_list.cfg"
)
