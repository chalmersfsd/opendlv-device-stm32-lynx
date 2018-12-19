cmake_minimum_required(VERSION 3.2)
project(opendlv-device-stm32-lynx)

################################################################################
# Defining the relevant versions of OpenDLV Standard Message Set and libcluon.
set(OPENDLV_STANDARD_MESSAGE_SET opendlv-standard-message-set-v0.9.1.odvd)
set(CLUON_COMPLETE cluon-complete-v0.0.114.hpp)
set(CLUON_PATH ${CMAKE_CURRENT_SOURCE_DIR})
################################################################################
# This projeect requires C++14 or newer.
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTIONSIONS OFF)
#Strip unneeded symbols from binaries
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s")
#Build a static binary.
set(CMAKE_EXE_LINKER_FLAGS "-static-libgcc -static-libstdc++")
#Add further warning levels.
#Change flag back to 2 or 3 when not debugging
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
	-D_XOPEN_SOURCE=700 \
	-D_FORTIFY_SOURCE=2 \
	-g \
	-O0 \
	-fstack-protector \
	-fomit-frame-pointer \
	-pipe \
	-pedantic -pedantic-errors \
	-Wall -Wextra -Wshadow -Wdeprecated \
	-Wdiv-by-zero -Wfloat-equal -Wpointer-arith \
	-Wuninitialized -Wunreachable-code \
	-Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn")
#Threads are necessary for linking the resulting binaries as UDPReceiver is running in parallel.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/cluon-msc
	WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/include/${CLUON_COMPLETE} ${CMAKE_BINARY_DIR}/cluon-complete.hpp
	COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_BINARY_DIR}/cluon-complete.hpp ${CMAKE_BINARY_DIR}/cluon-complete.cpp
    COMMAND ${CMAKE_CXX_COMPILER} -o ${CMAKE_BINARY_DIR}/cluon-msc ${CMAKE_BINARY_DIR}/cluon-complete.cpp -std=c++14 -pthread -D HAVE_CLUON_MSC
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${CLUON_COMPLETE})

################################################################################
# Generate opendlv-standard-message-set.{hpp,cpp} from ${OPENDLV_STANDARD_MESSAGE_SET} file.
add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/cluon-msc --cpp --out=${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp ${CMAKE_CURRENT_SOURCE_DIR}/include/${OPENDLV_STANDARD_MESSAGE_SET}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/${OPENDLV_STANDARD_MESSAGE_SET} ${CMAKE_BINARY_DIR}/cluon-msc)

# Add current build directory as include directory as it contains generated files.
include_directories(SYSTEM ${CMAKE_BINARY_DIR})

################################################################################
# Gather all object code first to avoid double compilation. This needs to be done for all added objects, can become very long? Why is this object and not static which works
add_library(${PROJECT_NAME}-core STATIC ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/proxy-stm32.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/request.cpp)
add_custom_target(generate_opendlv_standard_message_set_hpp DEPENDS ${CMAKE_BINARY_DIR}/opendlv-standard-message-set.hpp)
add_dependencies(${PROJECT_NAME}-core generate_opendlv_standard_message_set_hpp)

###############################################################################
# OpenMP
OPTION (USE_OpenMP "Use OpenMP" ON)
IF(USE_OpenMP)
  FIND_PACKAGE(OpenMP)
  IF(OPENMP_FOUND)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  ENDIF()
ENDIF()

###############################################################################
#TBB

################################################################################
#Link libs
include_directories(SYSTEM /usr/include)
#include_directories(SYSTEM /usr/local/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

#Add custom libs h
set(LIBRARIES ${LIBRARIES} Threads::Threads ${TBB_LIBRARIES} rt)
MESSAGE(STATUS ${LIBRARIES})

################################################################################
#Create executables.
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}.cpp)
#Link the executable with Libraries
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}-core ${LIBRARIES})

################################################################################
#Enable unit testing
enable_testing()
add_executable(${PROJECT_NAME}-runner ${CMAKE_CURRENT_SOURCE_DIR}/test/test-${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}-runner ${LIBRARIES} ${PROJECT_NAME}-core)
add_test(NAME ${PROJECT_NAME}-runner COMMAND ${PROJECT_NAME}-runner)

################################################################################
#Install executable
install(TARGETS ${PROJECT_NAME} DESTINATION bin COMPONENT ${PROJECT_NAME})
