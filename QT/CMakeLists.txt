cmake_minimum_required(VERSION 3.16)
project(RocketDynamics VERSION 0.1 LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Qt6 modules
find_package(OpenGL REQUIRED)
find_package(Qt6 COMPONENTS Widgets OpenGLWidgets OpenGL REQUIRED)

# Explicitly add UI file
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    Simulation.cpp
    Simulation.h
)

# Create the executable
qt_add_executable(RocketDynamics
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
)

# Include Eigen (adjust path as needed)
include_directories("C:/Toolbox/eigen-3.4.0")

# Link to Qt6 and OpenGLWidgets
target_link_libraries(RocketDynamics PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    OpenGL::GL
)

# Mark it as a GUI application on macOS/Windows
set_target_properties(RocketDynamics PROPERTIES
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

# Install rules
include(GNUInstallDirs)
install(TARGETS RocketDynamics
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Finalize the executable (Qt6 requirement)
qt_finalize_executable(RocketDynamics)
