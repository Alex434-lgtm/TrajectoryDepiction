#include "CCCPManager.h"
#include "CCCPConfig.h"
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QDebug>
#include <cstring>
#include <QDateTime>

// Include CCCP C headers
extern "C" {
#include "cccp.h"
#include "cccp_prototype_f.h"
#include "crc.h"
}

// Static instance for C callbacks
CCCPManager* CCCPManager::s_instance = nullptr;

// C function implementations required by CCCP
extern "C" {
// We only receive, so these can be stubs
int CCCP_LORA_Transmit(CCCP_frame_t frame) {
    Q_UNUSED(frame);
    return 0;
}

int CCCP_ETHERNET_Transmit(const char* data, int size) {
    Q_UNUSED(data);
    Q_UNUSED(size);
    return 0;
}

int CCCP_CAN_Transmit(CCCP_frame_t frame) {
    Q_UNUSED(frame);
    return 0;
}

unsigned int CCCP_Millis() {
    return QDateTime::currentMSecsSinceEpoch() & 0xFFFFFFFF;
}

void CCCP_Print(const char *fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);
    qDebug() << "[CCCP]" << buffer;
}

void CCCP_CALLBACK(void *data, unsigned int size, CCCP_LL_Header_t ll_head,
                   CCCP_TL_Header_t tl_head, enum CCCP_MSG_TYPE msgType) {
    if (CCCPManager::s_instance) {
        CCCPManager::cccp_message_callback(data, size, &ll_head, &tl_head, msgType);
    }
}
}

CCCPManager::CCCPManager(QObject *parent)
    : QObject(parent),
    m_socket(std::make_unique<QUdpSocket>(this)),
    m_handleTimer(std::make_unique<QTimer>(this))
{
    s_instance = this;
}

CCCPManager::~CCCPManager() {
    stop();
    s_instance = nullptr;
}

bool CCCPManager::initialize() {
    if (m_initialized) {
        return true;
    }

    // Initialize CCCP
    unsigned int subsystemId = CCCPConfig::SUBSYSTEM_ID;
    CCCP_Init(static_cast<CCCP_MODE>(CCCPConfig::TYPE),
              static_cast<CCCP_SYSTEM_ID>(CCCPConfig::SYSTEM_ID),
              &subsystemId, 1);

    // Bind UDP socket
    if (!m_socket->bind(QHostAddress::AnyIPv4, CCCPConfig::SERVER_PORT)) {
        emit errorOccurred(QString("Failed to bind to port %1: %2")
                               .arg(CCCPConfig::SERVER_PORT)
                               .arg(m_socket->errorString()));
        return false;
    }

    // Connect signals
    connect(m_socket.get(), &QUdpSocket::readyRead,
            this, &CCCPManager::processPendingDatagrams);

    // Setup periodic CCCP handling
    connect(m_handleTimer.get(), &QTimer::timeout,
            this, &CCCPManager::handleCCCP);
    m_handleTimer->setInterval(10);  // 10ms, 100Hz

    m_initialized = true;
    qDebug() << "CCCP Manager initialized on port" << CCCPConfig::SERVER_PORT;
    return true;
}

void CCCPManager::start() {
    if (!m_initialized) {
        if (!initialize()) {
            return;
        }
    }

    m_running = true;
    m_handleTimer->start();
    emit connectionStatusChanged(true);
    qDebug() << "CCCP Manager started";
}

void CCCPManager::stop() {
    m_running = false;
    m_handleTimer->stop();
    emit connectionStatusChanged(false);
    qDebug() << "CCCP Manager stopped";
}

void CCCPManager::processPendingDatagrams() {
    while (m_socket->hasPendingDatagrams()) {
        QNetworkDatagram datagram = m_socket->receiveDatagram();
        QByteArray data = datagram.data();

        m_receiveBuffer.append(data);

        // Process complete packets
        while (m_receiveBuffer.size() >= PACKET_SIZE) {
            QByteArray packet = m_receiveBuffer.left(PACKET_SIZE);
            m_receiveBuffer.remove(0, PACKET_SIZE);

            // Pass to CCCP
            int result = CCCP_LL_ETHERNET_Receive(packet.data(), packet.size());
            if (result < 0) {
                qDebug() << "CCCP frame processing failed";
            }
        }
    }
}

void CCCPManager::handleCCCP() {
    CCCP_HANDLE();

    // Check connection timeout (5 seconds)
    uint32_t now = CCCP_Millis();
    if (m_lastMessageTime > 0 && (now - m_lastMessageTime) > 5000) {
        emit connectionStatusChanged(false);
        m_lastMessageTime = 0;
    }
}

void CCCPManager::cccp_message_callback(void *data, unsigned int size,
                                        void* ll_head, void* tl_head,
                                        int msgType) {
    Q_UNUSED(ll_head);
    Q_UNUSED(tl_head);

    if (s_instance) {
        s_instance->processMessage(data, size, msgType);
    }
}

void CCCPManager::processMessage(void *data, unsigned int size, int msgType) {
    m_lastMessageTime = CCCP_Millis();
    emit connectionStatusChanged(true);

    const uint8_t* msgData = static_cast<const uint8_t*>(data);

    switch (msgType) {
    case CCCPConfig::ACCELEROMETER_DATA:
        parseAccelerometerData(msgData, size);
        break;

    case CCCPConfig::ATTITUDE_DATA:
        parseAttitudeData(msgData, size);
        break;

    case CCCPConfig::TELEMETRY_DATA:
        parseTelemetryData(msgData, size);
        break;

    default:
        qDebug() << "Received unknown message type:" << msgType;
        break;
    }
}

void CCCPManager::parseAccelerometerData(const uint8_t* data, unsigned int size) {
    // Assuming data comes as 16-bit values in channels (similar to GSEC_CURRENT_LOOP)
    // Channels 0,1,2 = accelerometer X,Y,Z
    if (size >= 6) {  // Need at least 3 channels * 2 bytes
        AccelerometerData accelData;

        // Convert from raw ADC values to m/s²
        // You'll need to adjust these scaling factors based on your sensor
        const float ACCEL_SCALE = 9.81f / 2048.0f;  // Example: ±2g range, 12-bit ADC

        uint16_t rawX = (data[1] << 8) | data[0];
        uint16_t rawY = (data[3] << 8) | data[2];
        uint16_t rawZ = (data[5] << 8) | data[4];

        accelData.x = (rawX - 2048) * ACCEL_SCALE;
        accelData.y = (rawY - 2048) * ACCEL_SCALE;
        accelData.z = (rawZ - 2048) * ACCEL_SCALE;
        accelData.timestamp = CCCP_Millis();

        emit accelerometerDataReceived(accelData);
    }
}

void CCCPManager::parseAttitudeData(const uint8_t* data, unsigned int size) {
    // Parse quaternion attitude data
    // Expecting 4 float values (w, x, y, z) or 4 int16 values
    if (size >= 16) {  // 4 floats * 4 bytes each
        AttitudeData attitude;

        // If data is sent as floats
        float* floatData = (float*)data;
        attitude.w = floatData[0];
        attitude.x = floatData[1];
        attitude.y = floatData[2];
        attitude.z = floatData[3];

        // Normalize quaternion
        float norm = sqrt(attitude.w*attitude.w + attitude.x*attitude.x +
                          attitude.y*attitude.y + attitude.z*attitude.z);
        if (norm > 0.0001f) {
            attitude.w /= norm;
            attitude.x /= norm;
            attitude.y /= norm;
            attitude.z /= norm;
        }

        attitude.timestamp = CCCP_Millis();
        emit attitudeDataReceived(attitude);
    }
    else if (size >= 8) {  // 4 int16 values * 2 bytes each
        AttitudeData attitude;

        // Convert from int16 normalized values (-32768 to 32767 maps to -1 to 1)
        const float QUAT_SCALE = 1.0f / 32767.0f;

        int16_t rawW = (data[1] << 8) | data[0];
        int16_t rawX = (data[3] << 8) | data[2];
        int16_t rawY = (data[5] << 8) | data[4];
        int16_t rawZ = (data[7] << 8) | data[6];

        attitude.w = rawW * QUAT_SCALE;
        attitude.x = rawX * QUAT_SCALE;
        attitude.y = rawY * QUAT_SCALE;
        attitude.z = rawZ * QUAT_SCALE;

        // Normalize quaternion
        float norm = sqrt(attitude.w*attitude.w + attitude.x*attitude.x +
                          attitude.y*attitude.y + attitude.z*attitude.z);
        if (norm > 0.0001f) {
            attitude.w /= norm;
            attitude.x /= norm;
            attitude.y /= norm;
            attitude.z /= norm;
        }

        attitude.timestamp = CCCP_Millis();
        emit attitudeDataReceived(attitude);
    }
}

void CCCPManager::parseTelemetryData(const uint8_t* data, unsigned int size) {
    // Parse telemetry messages for debugging/status
    if (size > 0) {
        QString message = QString::fromUtf8(reinterpret_cast<const char*>(data), size);
        emit telemetryReceived(message);
    }
}
