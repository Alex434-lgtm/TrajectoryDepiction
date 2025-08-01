#ifndef CCCP_CONFIG_H
#define CCCP_CONFIG_H

// CCCP System Configuration for Flight Path Imaging System
namespace CCCPConfig {
// Network Configuration
constexpr int SERVER_PORT = 25565;
constexpr const char* SERVER_IP = "0.0.0.0";  // Listen on all interfaces

// CCCP Protocol Configuration
constexpr int TYPE = 1;           // CCCP_MODE_ENDPOINT
constexpr int SYSTEM_ID = 2;      // MISSION_CONTROL
constexpr int SUBSYSTEM_ID = 1;   // MISSION_CONTROL subsystem

// Message Types we care about for flight visualization
enum FlightDataType {
    ACCELEROMETER_DATA = 1,  // Maps to CCCP_TYPE_GSEC_CURRENT_LOOP
    ATTITUDE_DATA = 4,       // Maps to CCCP_TYPE_GSEC_IOCARD_SERVOSTATE (or custom)
    TELEMETRY_DATA = 11      // Maps to CCCP_TYPE_GSEC_TELEMETRY
};
}

#endif // CCCP_CONFIG_H
