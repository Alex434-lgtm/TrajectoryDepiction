#ifndef CCCP_MANAGER_H
#define CCCP_MANAGER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <memory>
#include <array>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Forward declarations
struct AccelerometerData {
    float x, y, z;  // m/s²
    uint32_t timestamp;
};

struct AttitudeData {
    // Quaternion components
    float w, x, y, z;
    uint32_t timestamp;

    // Convert to Eigen quaternion
    Eigen::Quaterniond toQuaternion() const {
        return Eigen::Quaterniond(w, x, y, z);
    }
};

class CCCPManager : public QObject {
    Q_OBJECT

public:
    explicit CCCPManager(QObject *parent = nullptr);
    ~CCCPManager();

    bool initialize();
    void start();
    void stop();
    bool isRunning() const { return m_running; }

signals:
    // Emitted when new flight data is received
    void accelerometerDataReceived(const AccelerometerData& data);
    void attitudeDataReceived(const AttitudeData& data);
    void telemetryReceived(const QString& message);
    void connectionStatusChanged(bool connected);
    void errorOccurred(const QString& error);

public:
    // Static members for C callback access
    static CCCPManager* s_instance;  // For C callback access
    static void cccp_message_callback(void *data, unsigned int size,
                                      void* ll_head, void* tl_head,
                                      int msgType);

private slots:
    void processPendingDatagrams();
    void handleCCCP();

private:
    void processMessage(void *data, unsigned int size, int msgType);
    void parseAccelerometerData(const uint8_t* data, unsigned int size);
    void parseAttitudeData(const uint8_t* data, unsigned int size);
    void parseTelemetryData(const uint8_t* data, unsigned int size);

    // Network
    std::unique_ptr<QUdpSocket> m_socket;
    QByteArray m_receiveBuffer;
    static constexpr int PACKET_SIZE = 69;  // CCCP frame size + CRC

    // Timers
    std::unique_ptr<QTimer> m_handleTimer;

    // State
    bool m_initialized = false;
    bool m_running = false;
    uint32_t m_lastMessageTime = 0;
};

#endif // CCCP_MANAGER_H
