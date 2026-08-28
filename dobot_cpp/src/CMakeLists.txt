cmake_minimum_required(VERSION 3.8)
project(connect_dobot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    # -Wconversion
    -Wnon-virtual-dtor
    -Wold-style-cast
    # -Werror=return-type
    -fdiagnostics-color=always
    -fdiagnostics-show-option
  )
endif()


# find dependencies
find_package(Boost REQUIRED COMPONENTS system serialization thread date_time)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(connect_dobot src/connection_test.cpp)
target_link_libraries(connect_dobot
  PRIVATE
    Boost::system
    Boost::serialization
    Boost::thread
    Boost::date_time
)

target_include_directories(connect_dobot
    PRIVATE
    ${Boost_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)


install(TARGETS
  connect_dobot
  DESTINATION lib/${PROJECT_NAME}
)

