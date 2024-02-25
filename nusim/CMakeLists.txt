cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nuturtle_interfaces REQUIRED)

find_package(turtlelib REQUIRED)

set(NUSIM_DEPENDENCIES
    rclcpp
    visualization_msgs
    std_msgs
    nuturtlebot_msgs
    nav_msgs
    std_srvs
    tf2_ros
    tf2_geometry_msgs
    nuturtle_interfaces
)

add_executable(nusim src/nusim.cpp)
ament_target_dependencies(nusim ${NUSIM_DEPENDENCIES})
target_link_libraries(nusim turtlelib::turtlelib)

install(TARGETS
    nusim
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME}
)


# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

if(NOT CMAKE_CROSSCOMPILING)
    find_package(Doxygen REQUIRED)

    # build just because Doxygen is missing
    if(${DOXYGEN_FOUND} AND ${BUILD_DOCS})
        # Turn the README.md into the homepage of the doxygen docs
        set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

        # Tell Doxygen where to find the documentation
        doxygen_add_docs(doxygen include/ src/ README.md ALL)

        # The documentation will be in the build/html directory
        # The main page is build/html/index.html
    endif()

    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    endif()
endif()

ament_package()
