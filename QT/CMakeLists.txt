cmake_minimum_required(VERSION 3.16)
project(RocketDynamics VERSION 0.1 LANGUAGES CXX C)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find Qt6 modules
find_package(OpenGL REQUIRED)
find_package(Qt6 COMPONENTS Widgets OpenGLWidgets OpenGL Network REQUIRED)

# CCCP Library sources
set(CCCP_SOURCES
    cccp/src/cccp.c
    cccp/src/cccp_ll.c
    cccp/src/cccp_prototype_f.c
    cccp/src/crc.c
    cccp/example/flexible_circular_buffer/Src/flexible_circular_buffer.c
)

set(CCCP_HEADERS
    cccp/inc/cccp.h
    cccp/inc/cccp_prototype_f.h
    cccp/inc/cccp_systems.h
    cccp/inc/cccp_types.h
    cccp/inc/crc.h
    cccp/example/flexible_circular_buffer/Inc/flexible_circular_buffer.h
)

# Project sources
set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    Simulation.cpp
    Simulation.h
    CCCPManager.cpp
    CCCPManager.h
    CCCPConfig.h
)

# Create the executable
qt_add_executable(RocketDynamics
    MANUAL_FINALIZATION
    ${PROJECT_SOURCES}
    ${CCCP_SOURCES}
    ${CCCP_HEADERS}
)

# Include directories
target_include_directories(RocketDynamics PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/cccp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/cccp
    "C:/Toolbox/eigen-3.4.0"  # Adjust path as needed
)

# Link to Qt6 and OpenGL
target_link_libraries(RocketDynamics PRIVATE
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Qt6::OpenGL
    Qt6::Network
    OpenGL::GL
)

# Platform specific networking
if(WIN32)
    target_link_libraries(RocketDynamics PRIVATE ws2_32)
endif()

# Define CCCP configuration
target_compile_definitions(RocketDynamics PRIVATE
    CCCP_HAS_CUSTOM_CONFIG
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
