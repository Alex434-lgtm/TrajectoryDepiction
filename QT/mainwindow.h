#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QKeyEvent>
#include <QLabel>
#include <QMainWindow>
#include <QOpenGLFunctions>
#include <QTimer>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include "Simulation.h"
// Include Eigen if you still use the quaternion logic
#include <QWheelEvent>
#include <QtMath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "ui_mainwindow.h"
#include <deque>

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
protected:
    // Override keyPressEvent for WASD, Q/E, I/K, J/L
    void keyPressEvent(QKeyEvent *event) override;
private slots:
    void updateRocketPosition();
    void on_pushButton_clicked();
    void on_pushButton_released();
    void on_checkBox_checkStateChanged(const Qt::CheckState &arg1);
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void handlePageChange(int index);
    void on_comboBox_currentIndexChanged(int index);

private:
    // Simple custom OpenGL widget
    class RocketOpenGLWidget : public QOpenGLWidget, protected QOpenGLFunctions
    {
    public:
        //triada aktivni rizeni kdy bylo zapnuty rewind funkce zoom mouse orbital control
        explicit RocketOpenGLWidget(QWidget *parent = nullptr);
        ~RocketOpenGLWidget();

        // Camera transforms
        void moveCamera(float dx, float dy, float dz);
        void rotateCameraPitch(float angle);
        void rotateCameraYaw(float angle);

        // Rocket position
        void setRocketPosition(float x, float y, float z);

        // Stash small marker positions if you like
        void addMarker(float x, float y, float z);

        // Drawing functions
        void drawXYPlane(float size, int divisions);
        void drawTriad(float size);
        void drawOrientedTriad(float size, const Eigen::Quaterniond& orientation);
        void drawCone(float height, float radius, int segments);

        // Public camera movement parameters
        float cameraX = 0.0f;
        float cameraY = 0.0f;
        float cameraZ = 20.0f; // Put camera a bit far on Z to start
        float cameraPitch = 0.0f; // in degrees
        float cameraYaw = 0.0f;   // in degrees
        size_t currentIndex;
        std::vector<double> a;
        bool boolean = true;
        Eigen::Quaterniond currentOrientation; // Store current orientation

        float cameraFov = 45.0f;
        void adjustCameraFov(float delta);
        void moveCameraForward(float distance);
        void lookAt(float cx, float cy, float cz);
        float rocketX = 0.0f;
        float rocketY = 0.0f;
        float rocketZ = 0.0f;
        std::vector<QVector3D> markers;
        std::vector<QVector3D> antimarkers;

    protected:
        void initializeGL() override;
        void resizeGL(int w, int h) override;
        void paintGL() override;
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void mouseMoveEvent(QMouseEvent *event) override;

    private:
        // Rocket position


        // Marker positions (e.g. to visualize trajectory)
 // For reverse playback

        // Mouse control
        bool rightMousePressed = false;
        QPoint lastMousePos;

        // Drawing helper functions
        void drawSphere(float radius, int slices, int stacks);
        void drawXMesh(float size);
    };

    // Helper methods
    void setupStackedWidgetConnections();
    void initializeOpenGLWidgets();

    // Rocket simulation data
    double *x; // rocket X
    double *h; // rocket Z  (assuming 2D ground plane)

    // Orientation quaternions, velocities, etc.
    std::vector<Eigen::Quaterniond> orientationQuaternions;
    std::vector<double> v;
    std::vector<double> a;
    size_t currentIndex;
    float elapsedTime = 0.0f;

    // UI
    RocketOpenGLWidget *glWidget;
    RocketOpenGLWidget *glWidget2;
    QLabel *timerLabel;
    QTimer *timer;

    // Movement and rotation parameters
    float translateStep = 0.5f; // speed for WASDEQ
    float rotateAngle = 2.0f;   // degrees for rotations
    Ui_MainWindow *ui;

    // Coordinates storage for reverse playback
    std::deque<std::tuple<double, double>> coordinates;
    std::deque<std::tuple<double, double>>::reverse_iterator it;
    bool reversal = false;
    bool follow=false;
    void setupComboBox();
};

#endif // MAINWINDOW_H
