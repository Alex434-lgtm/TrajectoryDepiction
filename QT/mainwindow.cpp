#include "mainwindow.h"
#include <QOpenGLFunctions>
#include <QVBoxLayout>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <cmath>
#include <iostream>
#include <deque>

//---------------------------------------
// MainWindow Constructor
//---------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , currentIndex(0)
    , elapsedTime(0.0f),
     ui(new Ui_MainWindow),
    coordinates{std::tuple<double,double>{0.0,0.0}}
{
    ui->setupUi(this);
    
    // Initialize rocket position variables
    h = new double(0.0);
    x = new double(0.0);

    // Generate rocket dynamics simulation data
    auto state = simulateRocketDynamics();
    orientationQuaternions.reserve(state[0].size());
    
    // Convert flight path angles to quaternions for 3D rotation
    for (double angle : state[0]) {
        double half_angle = angle * 0.5;
        double w = std::cos(half_angle);
        double xx = 0.0;
        double yy = std::sin(half_angle);
        double zz = 0.0;
        Eigen::Quaterniond q(w, xx, yy, zz);
        orientationQuaternions.push_back(q);
    }
    v = state[1];
    a = state[2];
    
    // Create OpenGL widgets for dual viewport display
    glWidget = new RocketOpenGLWidget(this);
    glWidget2 = new RocketOpenGLWidget(this);
    
    // Configure first viewport
    ui->widget_2->setLayout(new QVBoxLayout(ui->widget_2));
    ui->widget_2->layout()->addWidget(glWidget);
    ui->widget_2->layout()->setContentsMargins(0, 0, 0, 0);
    
    // Configure second viewport
    ui->widget->setLayout(new QVBoxLayout(ui->widget));
    ui->widget->layout()->addWidget(glWidget2);
    ui->widget->layout()->setContentsMargins(0, 0, 0, 0);
    
    // Share acceleration data with widgets
    glWidget->a = this->a;
    glWidget2->a = this->a;
    glWidget2->boolean = false;
    
    // Create and configure timer display
    timerLabel = new QLabel(this);
    timerLabel->setStyleSheet("color: black; font-size: 20px;");
    timerLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    timerLabel->setText("Time: 0.0 s");
    timerLabel->setGeometry(10, 10, 200, 50);
    timerLabel->raise();

    // Setup animation timer for real-time updates
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateRocketPosition);
    
    // Initialize UI connections
    setupStackedWidgetConnections();
    setupComboBox();
    initializeOpenGLWidgets();
    
    // Start animation at 100 FPS
    timer->start(10);
}

MainWindow::~MainWindow()
{
    delete x;
    delete h;
    delete ui;
}

//---------------------------------------
// Camera Forward Movement
//---------------------------------------
void MainWindow::RocketOpenGLWidget::moveCameraForward(float distance)
{
    // Convert camera angles to radians
    float pitchRad = qDegreesToRadians(cameraPitch);
    float yawRad = qDegreesToRadians(cameraYaw);

    // Calculate forward direction vector based on camera orientation
    float forwardX = cos(pitchRad) * -std::sin(yawRad);
    float forwardY = std::sin(pitchRad);
    float forwardZ = cos(pitchRad) * -std::cos(yawRad);

    // Move camera along forward vector
    cameraX += forwardX * distance;
    cameraY += forwardY * distance;
    cameraZ += forwardZ * distance;

    update();
}

//---------------------------------------
// Update Rocket Position Each Frame
//---------------------------------------
void MainWindow::updateRocketPosition()
{
    static int placeHolder = 0;
    
    // Check if simulation is complete
    if (currentIndex >= orientationQuaternions.size()) {
        timer->stop();
        return;
    }
    
    // Get current orientation quaternion
    Eigen::Quaterniond q = orientationQuaternions[currentIndex];
    Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

    // Calculate rocket's forward direction
    Eigen::Vector3d xUnit(1.0, 0.0, 0.0);
    Eigen::Vector3d rotatedx = rotationMatrix * xUnit;
    Eigen::Vector3d rotatedxnorm = rotatedx.normalized();

    // Calculate position update based on velocity
    Eigen::Vector3d transl = rotatedxnorm * v[currentIndex];
    double currentoffsetx = 0.01 * transl.x();
    double currentoffsetz = 0.01 * transl.z();

    // Store coordinate for playback functionality
    coordinates.push_back(std::make_tuple(*x, *h));
    
    // Update rocket position
    *x += currentoffsetx;
    *h += currentoffsetz;

    // Update first widget
    glWidget->setRocketPosition(float(*x), 0.0f, float(*h));
    glWidget->currentOrientation = q;

    // Update second widget
    glWidget2->setRocketPosition(float(*x), 0.0f, float(*h));
    glWidget2->currentOrientation = q;

    // Update camera position if in follow mode
    if (follow) {
        glWidget->cameraYaw = 0;
        glWidget->cameraPitch = 270;
        glWidget2->cameraYaw = 0;
        glWidget2->cameraPitch = 270;
        glWidget2->cameraX = float(*x);
        glWidget2->cameraY = float(1000);
        glWidget2->cameraZ = float(*h);
        glWidget->cameraX = float(*x);
        glWidget->cameraY = float(100);
        glWidget->cameraZ = float(*h);
    }
    
    // Add initial reference markers
    if (placeHolder == 0) {
        glWidget->addMarker(1000, 0, 0);
        glWidget2->addMarker(-1000, 0, 0);
        placeHolder++;
    }
    
    // Add trajectory markers
    glWidget->addMarker(float(*x), 0.0f, float(*h));
    glWidget2->addMarker(float(*x), 0.0f, float(*h));

    // Update time display
    elapsedTime += 0.01f;
    timerLabel->setText(QString("Time: %1 s").arg(elapsedTime, 0, 'f', 2));

    // Update frame indices
    ++currentIndex;
    glWidget->currentIndex = this->currentIndex;
    glWidget2->currentIndex = this->currentIndex;
}

//---------------------------------------
// Keyboard Input Handler
//---------------------------------------
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        return;
    }

    switch (event->key()) {
    // Camera translation controls
    case Qt::Key_W:
        glWidget->moveCamera(
            std::sin(glWidget->cameraYaw * M_PI / 180) * translateStep, 
            -std::cos(glWidget->cameraYaw * M_PI / 180) * translateStep, 
            -std::cos(glWidget->cameraPitch * M_PI / 180) * translateStep
        );
        break;
    case Qt::Key_S:
        glWidget->moveCamera(0.0f, 0.0f, -translateStep);
        glWidget2->moveCamera(0.0f, 0.0f, -translateStep);
        break;
    case Qt::Key_A:
        glWidget->moveCamera(-translateStep, 0.0f, 0.0f);
        break;
    case Qt::Key_D:
        glWidget->moveCamera(translateStep, 0.0f, 0.0f);
        break;

    // Camera rotation controls
    case Qt::Key_I:
        glWidget->rotateCameraPitch(-rotateAngle);
        break;
    case Qt::Key_K:
        glWidget->rotateCameraPitch(rotateAngle);
        break;
    case Qt::Key_J:
        glWidget->rotateCameraYaw(-rotateAngle);
        break;
    case Qt::Key_L:
        glWidget->rotateCameraYaw(rotateAngle);
        break;
        
    // Movement speed controls
    case Qt::Key_M:
        translateStep *= 5;
        break;
    case Qt::Key_N:
        translateStep *= 0.2;
        break;

    // Trajectory playback controls
    case Qt::Key_Left:
        if (reversal == true) {
            // Step backward through trajectory (20 frames)
            for (int i = 0; i < 20; i++) {
                if (it != coordinates.rend() - 1) {
                    it++;
                    glWidget->rocketX = std::get<0>(*it);
                    glWidget->rocketZ = std::get<1>(*it);
                    
                    // Move marker to antimarkers list
                    QVector3D t = *(--glWidget->markers.end());
                    glWidget->markers.pop_back();
                    glWidget->antimarkers.push_back(t);
                    glWidget->update();
                }
            }
        }
        break;
        
    case Qt::Key_Right:
        if (reversal == true) {
            // Step forward through trajectory (20 frames)
            for (int i = 0; i < 20; i++) {
                if ((it != coordinates.rbegin() - 1) && (it != coordinates.rbegin())) {
                    it--;
                    glWidget->rocketX = std::get<0>(*it);
                    glWidget->rocketZ = std::get<1>(*it);
                    
                    // Move antimarker back to markers list
                    QVector3D t = *(--glWidget->antimarkers.end());
                    glWidget->antimarkers.pop_back();
                    glWidget->markers.push_back(t);
                    glWidget->update();
                }
            }
        }
        break;
        
    default:
        QMainWindow::keyPressEvent(event);
        return;
    }

    event->accept();
}

//---------------------------------------
// RocketOpenGLWidget Constructor
//---------------------------------------
MainWindow::RocketOpenGLWidget::RocketOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    // Set initial camera position
    cameraX = 5000.0f;
    cameraY = 5000.0f;
    cameraZ = 0.0f;
    cameraPitch = 0.0f;
    cameraYaw = 0.0f;
    
    // Enable keyboard and mouse input
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    currentIndex = 0;
}

MainWindow::RocketOpenGLWidget::~RocketOpenGLWidget() {}

//---------------------------------------
// OpenGL Initialization
//---------------------------------------
void MainWindow::RocketOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    
    // Set white background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

//---------------------------------------
// Handle Widget Resize
//---------------------------------------
void MainWindow::RocketOpenGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    // Setup perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    
    GLfloat aspect = GLfloat(w) / GLfloat(h ? h : 1);
    const float nearPlane = 0.1f;
    const float farPlane = 10000.0f;
    float fovRad = cameraFov * 3.14159f / 180.0f;
    float f = 1.0f / std::tan(fovRad * 0.5f);
    float A = (farPlane + nearPlane) / (nearPlane - farPlane);
    float B = (2.0f * farPlane * nearPlane) / (nearPlane - farPlane);

    GLfloat perspective[16] = {f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, A, -1, 0, 0, B, 0};
    glLoadMatrixf(perspective);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

//---------------------------------------
// Render Scene
//---------------------------------------
void MainWindow::RocketOpenGLWidget::paintGL()
{
    static int staticplaceholder = 0;
    static float staticplaceholder2 = -10000.0f;
    static int drawCounter = 0;
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Apply camera transformations
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Apply camera rotations
    glRotatef(-cameraPitch, 1.0f, 0.0f, 0.0f);
    glRotatef(-cameraYaw, 0.0f, 0.0f, 1.0f);

    // Apply camera translation
    glTranslatef(-cameraX, -cameraY, -cameraZ);

    // Initialize camera orientation on first frame
    if (staticplaceholder == 0) {
        cameraPitch = 270.0f;
        cameraYaw = 0.0f;
        staticplaceholder++;
    }

    // Draw reference grid
    drawXYPlane(5000.0f, 2000);

    // Draw rocket
    glPushMatrix();
    {
        if (markers.size() > 0 && markers[0].x() == 1000) {
            // Position rocket
            glTranslatef(rocketX, rocketY, rocketZ);

            // Draw orientation triad
            drawOrientedTriad(5.0f, currentOrientation);

            // Draw rocket sphere
            glColor3f(1.0f, 0.0f, 0.0f);
            drawSphere(1.0f, 20, 20);
        }
    }
    glPopMatrix();

    // Update marker color transition point
    if (a[currentIndex] < 0) {
        // Acceleration is negative (no thrust)
    } else {
        staticplaceholder++;
    }
    
    // Draw trajectory markers
    for (int i = 0; i < markers.size(); i++) {
        // First widget - detailed trajectory view
        if (markers[0].x() == 1000 && i > 0) {
            // Color based on flight phase
            if (i < staticplaceholder) {
                glColor3f(0.0f, 0.0f, 1.0f);  // Blue - powered flight
            } else {
                glColor3f(0.0f, 0.5f, 0.0f);  // Green - coast phase
            }
            glPushMatrix();
            glTranslatef(markers[i].x(), markers[i].y(), markers[i].z());
            drawXMesh(2.0f);
            glPopMatrix();
        }
        // Second widget - sparse visualization
        else if (markers[0].x() == -1000 && i > 0) {
            // Draw markers at intervals
            if (std::abs(markers[i].x() - staticplaceholder2) > 25.0) {
                staticplaceholder2 = markers[i].x();

                glPushMatrix();
                glTranslatef(markers[i].x(), markers[i].y(), markers[i].z());

                // Draw orientation triad
                drawOrientedTriad(5.0f, currentOrientation);

                // Color based on flight phase
                if (i < staticplaceholder) {
                    glColor3f(0.0f, 0.0f, 1.0f);  // Blue
                } else {
                    glColor3f(0.0f, 0.5f, 0.0f);  // Green
                }

                drawSphere(1.0f, 20, 20);
                glPopMatrix();
                drawCounter++;
            }
        }
    }
}

//---------------------------------------
// Camera Movement
//---------------------------------------
void MainWindow::RocketOpenGLWidget::moveCamera(float dx, float dy, float dz)
{
    cameraX += dx;
    cameraY += dy;
    cameraZ += dz;
    update();
}

//---------------------------------------
// Camera Rotation
//---------------------------------------
void MainWindow::RocketOpenGLWidget::rotateCameraPitch(float angle)
{
    cameraPitch += angle;
    update();
}

void MainWindow::RocketOpenGLWidget::rotateCameraYaw(float angle)
{
    cameraYaw += angle;
    update();
}

//---------------------------------------
// Set Rocket Position
//---------------------------------------
void MainWindow::RocketOpenGLWidget::setRocketPosition(float x, float y, float z)
{
    rocketX = x;
    rocketY = y;
    rocketZ = z;
    update();
}

//---------------------------------------
// Add Trajectory Marker
//---------------------------------------
void MainWindow::RocketOpenGLWidget::addMarker(float x, float y, float z)
{
    markers.emplace_back(x, y, z);
}

//---------------------------------------
// Draw 3D Sphere
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawSphere(float radius, int slices, int stacks)
{
    // Generate sphere using latitude/longitude approach
    for (int i = 0; i < stacks; ++i) {
        float lat0 = M_PI * (-0.5f + float(i) / stacks);
        float z0 = std::sin(lat0);
        float zr0 = std::cos(lat0);

        float lat1 = M_PI * (-0.5f + float(i + 1) / stacks);
        float z1 = std::sin(lat1);
        float zr1 = std::cos(lat1);

        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= slices; ++j) {
            float lng = 2.0f * M_PI * float(j) / slices;
            float x = std::cos(lng);
            float y = std::sin(lng);

            // Top vertex
            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);

            // Bottom vertex
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }
}

//---------------------------------------
// Draw X-shaped Marker
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawXMesh(float size)
{
    glBegin(GL_LINES);
    
    // First diagonal
    glVertex3f(-size, -size, 0.0f);
    glVertex3f(size, size, 0.0f);

    // Second diagonal
    glVertex3f(size, -size, 0.0f);
    glVertex3f(-size, size, 0.0f);
    
    glEnd();
}

//---------------------------------------
// Draw XY Plane Grid
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawXYPlane(float size, int divisions)
{
    float step = (2.0f * size) / divisions;
    float halfSize = size;

    // Draw grid lines
    glColor3f(0.7f, 0.7f, 0.7f);
    glLineWidth(1.0f);

    glBegin(GL_LINES);

    // Horizontal lines
    for (int i = 0; i <= divisions; ++i) {
        float y = -halfSize + i * step;
        glVertex3f(-halfSize, y, 0.0f);
        glVertex3f(halfSize, y, 0.0f);
    }

    // Vertical lines
    for (int i = 0; i <= divisions; ++i) {
        float x = -halfSize + i * step;
        glVertex3f(x, -halfSize, 0.0f);
        glVertex3f(x, halfSize, 0.0f);
    }

    glEnd();

    // Draw coordinate axes
    glLineWidth(2.0f);

    // X-axis
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(-halfSize, 0.0f, 0.0f);
    glVertex3f(halfSize, 0.0f, 0.0f);
    glEnd();

    // Y-axis
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, -halfSize, 0.0f);
    glVertex3f(0.0f, halfSize, 0.0f);
    glEnd();

    glLineWidth(1.0f);
}

void MainWindow::on_pushButton_clicked()
{
}

void MainWindow::on_pushButton_released()
{
}

//---------------------------------------
// Toggle Reverse Playback Mode
//---------------------------------------
void MainWindow::on_checkBox_checkStateChanged(const Qt::CheckState &arg1)
{
    if (reversal == false) {
        reversal = true;
        it = coordinates.rbegin();
    } else {
        reversal = false;
    }
}

//---------------------------------------
// Mouse Press Handler
//---------------------------------------
void MainWindow::RocketOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        rightMousePressed = true;
        lastMousePos = event->pos();
    }
    event->accept();
}

//---------------------------------------
// Mouse Release Handler
//---------------------------------------
void MainWindow::RocketOpenGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        rightMousePressed = false;
    }
    event->accept();
}

//---------------------------------------
// Mouse Movement Handler
//---------------------------------------
void MainWindow::RocketOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (rightMousePressed) {
        // Calculate mouse delta
        QPoint delta = event->pos() - lastMousePos;

        // Update camera orientation
        float sensitivity = 0.2f;
        rotateCameraYaw(-delta.x() * sensitivity);
        rotateCameraPitch(-delta.y() * sensitivity);

        // Store current position
        lastMousePos = event->pos();
    }
    event->accept();
}

//---------------------------------------
// Handle Stacked Widget Page Changes
//---------------------------------------
void MainWindow::handlePageChange(int index)
{
    // Ensure proper OpenGL context when switching pages
    if (index == 0) {
        // First page active
        if (glWidget) {
            QTimer::singleShot(10, [this](){
                glWidget->makeCurrent();
                glWidget->update();
            });
        }
    } else if (index == 1) {
        // Second page active
        if (glWidget2) {
            QTimer::singleShot(10, [this](){
                glWidget2->makeCurrent();
                glWidget2->update();
            });
        }
    }
}

//---------------------------------------
// Switch to Second View
//---------------------------------------
void MainWindow::on_pushButton_2_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);

    // Update OpenGL context
    QTimer::singleShot(10, [this](){
        if (glWidget2) {
            glWidget2->makeCurrent();
            glWidget2->update();
        }
    });
}

//---------------------------------------
// Switch to First View
//---------------------------------------
void MainWindow::on_pushButton_3_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);

    // Update OpenGL context
    QTimer::singleShot(10, [this](){
        if (glWidget) {
            glWidget->makeCurrent();
            glWidget->update();
        }
    });
}

//---------------------------------------
// Setup Stacked Widget Connections
//---------------------------------------
void MainWindow::setupStackedWidgetConnections()
{
    // Connect page change signal
    connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, &MainWindow::handlePageChange);

    // Connect view switching buttons
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::on_pushButton_2_clicked);
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::on_pushButton_3_clicked);
}

//---------------------------------------
// Initialize OpenGL Widgets
//---------------------------------------
void MainWindow::initializeOpenGLWidgets()
{
    // Ensure both widgets are properly initialized
    ui->stackedWidget->setCurrentIndex(0);
    QApplication::processEvents();

    // Initialize first widget
    glWidget->makeCurrent();
    glWidget->update();

    // Initialize second widget with top-down view
    glWidget2->makeCurrent();
    glWidget2->cameraX = 0.0f;
    glWidget2->cameraY = 20.0f;
    glWidget2->cameraZ = 0.0f;
    glWidget2->cameraPitch = 270.0f;
    glWidget2->cameraYaw = 0.0f;
    glWidget2->update();

    // Set initial orientations
    if (!orientationQuaternions.empty()) {
        glWidget->currentOrientation = orientationQuaternions[0];
        glWidget2->currentOrientation = orientationQuaternions[0];
    }

    // Restore original page
    if (ui->stackedWidget->currentIndex() != 0) {
        ui->stackedWidget->setCurrentIndex(1);
        QApplication::processEvents();
    }
}

//---------------------------------------
// Draw Coordinate Axes Triad
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawTriad(float size)
{
    // Save current line width
    GLfloat currentWidth;
    glGetFloatv(GL_LINE_WIDTH, &currentWidth);

    glLineWidth(2.0f);

    glBegin(GL_LINES);

    // X-axis (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size, 0.0f, 0.0f);

    // Y-axis (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, size, 0.0f);

    // Z-axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, size);

    glEnd();

    // Restore line width
    glLineWidth(currentWidth);
}

//---------------------------------------
// Draw 3D Cone
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawCone(float height, float radius, int segments)
{
    float angleStep = 2.0f * M_PI / segments;

    // Draw cone surface
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, height);

    for (int i = 0; i <= segments; i++) {
        float angle = i * angleStep;
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);
        glVertex3f(x, y, 0.0f);
    }
    glEnd();

    // Draw cone base
    glBegin(GL_TRIANGLE_FAN);
    glVertex3f(0.0f, 0.0f, 0.0f);

    for (int i = segments; i >= 0; i--) {
        float angle = i * angleStep;
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);
        glVertex3f(x, y, 0.0f);
    }
    glEnd();
}

//---------------------------------------
// Draw Oriented Coordinate Triad
//---------------------------------------
void MainWindow::RocketOpenGLWidget::drawOrientedTriad(float size, const Eigen::Quaterniond& orientation)
{
    // Save current line width
    GLfloat currentWidth;
    glGetFloatv(GL_LINE_WIDTH, &currentWidth);

    glLineWidth(2.0f);

    // Get rotation matrix from quaternion
    Eigen::Matrix3d rotationMatrix = orientation.toRotationMatrix();

    // Transform standard basis vectors
    Eigen::Vector3d xAxis = rotationMatrix * Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d yAxis = rotationMatrix * Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d zAxis = rotationMatrix * Eigen::Vector3d(0.0, 0.0, 1.0);

    // Draw oriented axes
    glBegin(GL_LINES);

    // X-axis (red) - flight direction
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * xAxis.x(), size * xAxis.y(), size * xAxis.z());

    // Y-axis (green)
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * yAxis.x(), size * yAxis.y(), size * yAxis.z());

    // Z-axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * zAxis.x(), size * zAxis.y(), size * zAxis.z());

    glEnd();

    // Draw arrow cones at axis endpoints
    // X-axis arrow
    glColor3f(1.0f, 0.0f, 0.0f);
    glPushMatrix();
    glTranslatef(size * xAxis.x(), size * xAxis.y(), size * xAxis.z());

    // Align cone with axis direction
    Eigen::Vector3d stdZ(0, 0, 1);
    Eigen::Vector3d rotAxis = stdZ.cross(xAxis);
    if (rotAxis.norm() > 1e-6) {
        rotAxis.normalize();
        float rotAngle = acos(stdZ.dot(xAxis)) * 180.0f / M_PI;
        glRotatef(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());
    } else if (xAxis.z() < 0) {
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    }

    drawCone(size * 0.2f, size * 0.1f, 8);
    glPopMatrix();

    // Y-axis arrow
    glColor3f(0.0f, 1.0f, 0.0f);
    glPushMatrix();
    glTranslatef(size * yAxis.x(), size * yAxis.y(), size * yAxis.z());

    rotAxis = stdZ.cross(yAxis);
    if (rotAxis.norm() > 1e-6) {
        rotAxis.normalize();
        float rotAngle = acos(stdZ.dot(yAxis)) * 180.0f / M_PI;
        glRotatef(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());
    } else if (yAxis.z() < 0) {
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    }

    drawCone(size * 0.2f, size * 0.1f, 8);
    glPopMatrix();

    // Z-axis arrow
    glColor3f(0.0f, 0.0f, 1.0f);
    glPushMatrix();
    glTranslatef(size * zAxis.x(), size * zAxis.y(), size * zAxis.z());

    rotAxis = stdZ.cross(zAxis);
    if (rotAxis.norm() > 1e-6) {
        rotAxis.normalize();
        float rotAngle = acos(stdZ.dot(zAxis)) * 180.0f / M_PI;
        glRotatef(rotAngle, rotAxis.x(), rotAxis.y(), rotAxis.z());
    } else if (zAxis.z() < 0) {
        glRotatef(180.0f, 1.0f, 0.0f, 0.0f);
    }

    drawCone(size * 0.2f, size * 0.1f, 8);
    glPopMatrix();

    // Restore line width
    glLineWidth(currentWidth);
}

//---------------------------------------
// Setup Camera Mode ComboBox
//---------------------------------------
void MainWindow::setupComboBox()
{
    // Configure camera mode selector
    ui->comboBox->blockSignals(true);

    ui->comboBox->clear();
    ui->comboBox->addItem("Free Camera");
    ui->comboBox->addItem("Follow Rocket");

    ui->comboBox->setCurrentIndex(0);
    follow = false;

    ui->comboBox->blockSignals(false);

    // Connect signal
    disconnect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
               this, &MainWindow::on_comboBox_currentIndexChanged);
    connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::on_comboBox_currentIndexChanged);
}

//---------------------------------------
// Handle Camera Mode Change
//---------------------------------------
void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    std::cout << "ComboBox changed to index: " << index << std::endl;

    if (index == 1) {
        // Enable follow mode
        follow = true;
        std::cout << "Follow mode enabled" << std::endl;

        // Position cameras behind rocket
        glWidget->cameraX = float(*x);
        glWidget->cameraY = 100.0f;
        glWidget->cameraZ = float(*h);
        glWidget->cameraYaw = 0.0f;

        glWidget2->cameraX = float(*x);
        glWidget2->cameraY = 1000.0f;
        glWidget2->cameraZ = float(*h);

        glWidget->update();
        glWidget2->update();
    }
    else if (index == 0) {
        // Disable follow mode
        follow = false;
        std::cout << "Free camera mode enabled" << std::endl;
    }
}