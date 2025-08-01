#include "mainwindow.h"
#include <QOpenGLFunctions>
#include <QVBoxLayout>
#include <QtOpenGLWidgets/QOpenGLWidget>
#include <cmath>
#include <iostream>
#include <deque>
#include <QMessageBox>

//---------------------------------------
// Simulate rocket: placeholder example
//---------------------------------------

//---------------------------------------
// MainWindow
//---------------------------------------
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , currentIndex(0)
    , elapsedTime(0.0f),
     ui(new Ui_MainWindow),
    coordinates{std::tuple<double,double>{0.0,0.0}}
{

    ui->setupUi(this);
    // Allocate some state
    h = new double(0.0);
    x = new double(0.0);

    // Simulate rocket
    auto state = simulateRocketDynamics();
    orientationQuaternions.reserve(state[0].size());
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
    // Create our custom OpenGL widget
    glWidget = new RocketOpenGLWidget(this);
    glWidget2= new RocketOpenGLWidget(this);
    ui->widget_2->setLayout(new QVBoxLayout(ui->widget_2));
    ui->widget_2->layout()->addWidget(glWidget);
    ui->widget_2->layout()->setContentsMargins(0, 0, 0, 0);
    ui->widget->setLayout(new QVBoxLayout(ui->widget));
    ui->widget->layout()->addWidget(glWidget2);
    ui->widget->layout()->setContentsMargins(0, 0, 0, 0);
    //PAK TAM PRIJDE 2HA widgeta s jen xy poloze
    glWidget->a = this->a;
    glWidget2->a = this->a;
    glWidget2->boolean=false;
    // Timer label
    timerLabel = new QLabel(this);
    timerLabel->setStyleSheet("color: black; font-size: 20px;");
    timerLabel->setAlignment(Qt::AlignTop | Qt::AlignLeft);
    timerLabel->setText("Time: 0.0 s");
    // Place it in the corner
    timerLabel->setGeometry(10, 10, 200, 50);
    timerLabel->raise(); // Ensure it stays on top

    // Animation timer
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateRocketPosition);
    setupStackedWidgetConnections();
    setupComboBox();
    setupCCCP();
    this->initializeOpenGLWidgets();
    timer->start(10); // 10 ms -> 100 FPS max
}

MainWindow::~MainWindow()
{
    if (m_cccp && m_cccp->isRunning()) {
        m_cccp->stop();
    }
    delete x;
    delete h;
    delete ui;
}
void MainWindow::RocketOpenGLWidget::moveCameraForward(float distance)
{
    // Convert pitch/yaw (in degrees) to radians
    float pitchRad = qDegreesToRadians(cameraPitch);
    float yawRad = qDegreesToRadians(cameraYaw);

    // If your coordinate system uses:
    //  - cameraPitch as rotation around X
    //  - cameraYaw   as rotation around Z
    //
    // Then an approximate "forward" direction might be:
    //   forwardX =  cos(pitch) * (-sin(yaw))
    //   forwardY =  sin(pitch)
    //   forwardZ =  cos(pitch) * (-cos(yaw))
    //
    // (Minus signs come from your original code hint that
    //  negative Z is "forward," etc. Adjust to taste!)

    float forwardX = cos(pitchRad) * -std::sin(yawRad);
    float forwardY = std::sin(pitchRad);
    float forwardZ = cos(pitchRad) * -std::cos(yawRad);

    // Move the camera along this forward vector
    cameraX += forwardX * distance;
    cameraY += forwardY * distance;
    cameraZ += forwardZ * distance;

    // Trigger a redraw
    update();
}

//---------------------------------------
// Update rocket each frame
//---------------------------------------
void MainWindow::updateRocketPosition()
{
    if (m_useLiveTelemetry) {
        return;  // Let CCCP callbacks handle updates
    }
    static int placeHolder=0;
    if (currentIndex >= orientationQuaternions.size()) {
        timer->stop();
    } else {
        // Orientation from quaternion
        Eigen::Quaterniond q = orientationQuaternions[currentIndex];
        Eigen::Matrix3d rotationMatrix = q.toRotationMatrix();

        // Rotate the X unit vector
        Eigen::Vector3d xUnit(1.0, 0.0, 0.0);
        Eigen::Vector3d rotatedx = rotationMatrix * xUnit;
        Eigen::Vector3d rotatedxnorm = rotatedx.normalized();

        // v[currentIndex] is forward speed
        Eigen::Vector3d transl = rotatedxnorm * v[currentIndex];

        // Update rocket position in XY-plane (with Z as "height" or vice versa)
        double currentoffsetx = 0.01 * transl.x();
        double currentoffsetz = 0.01 * transl.z();

        coordinates.push_back(std::make_tuple(*x,*h));
        *x += currentoffsetx;
        *h += currentoffsetz;

        glWidget->setRocketPosition(float(*x), 0.0f, float(*h));
        glWidget->currentOrientation = q;  // Pass quaternion to first widget

        // Also update second widget position and orientation
        glWidget2->setRocketPosition(float(*x), 0.0f, float(*h));
        glWidget2->currentOrientation = q;  // Pass quaternion to second widget


        if(follow){
        glWidget->cameraYaw=0;
        glWidget->cameraPitch=270;
        glWidget2->cameraYaw=0;
        glWidget2->cameraPitch=270;
        glWidget2->cameraX=float(*x);
        glWidget2->cameraY=float(1000);
        glWidget2->cameraZ=float(*h);
        glWidget->cameraX=float(*x);
        glWidget->cameraY=float(100);
        glWidget->cameraZ=float(*h);
        }
        // Add markers for first widget
        if(placeHolder==0){
        glWidget->addMarker(1000,0,0);
        glWidget2->addMarker(-1000,0,0);
        placeHolder++;
        }
        glWidget->addMarker(float(*x), 0.0f, float(*h));
        glWidget2->addMarker(float(*x), 0.0f, float(*h));

        // Update time
        elapsedTime += 0.01f;
        timerLabel->setText(QString("Time: %1 s").arg(elapsedTime, 0, 'f', 2));

        // Update indices
        ++currentIndex;
        glWidget->currentIndex = this->currentIndex;
        glWidget2->currentIndex = this->currentIndex;
    }
}

//---------------------------------------
// Key press: WASD/QE and IJKL
//---------------------------------------
void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (event->isAutoRepeat()) {
        // Optionally ignore repeats
        // return;
    }

    switch (event->key()) {
    // Translation
    case Qt::Key_W:
        glWidget->moveCamera(std::sin(glWidget->cameraYaw*M_PI/180)*translateStep, -std::cos(glWidget->cameraYaw*M_PI/180)*translateStep, -std::cos(glWidget->cameraPitch*M_PI/180)*translateStep); // forward
        break;
    case Qt::Key_S:
        glWidget->moveCamera(0.0f, 0.0f, -translateStep);
       glWidget2->moveCamera(0.0f, 0.0f, -translateStep);        // back
        break;
    case Qt::Key_A:
        glWidget->moveCamera(-translateStep, 0.0f, 0.0f); // left
        break;
    case Qt::Key_D:
        glWidget->moveCamera(translateStep, 0.0f, 0.0f); // right
        break;
    // case Qt::Key_Q:
    //     glWidget->moveCamera(0.0f, -translateStep, 0.0f); // up
    //     break;
    // case Qt::Key_E:
    //     glWidget->moveCamera(0.0f, translateStep, 0.0f); // down
    //     break;

        // Camera rotation: pitch (I/K) and yaw (J/L)
    case Qt::Key_I:
        // tilt camera up => pitch up is negative rotation about camera's X axis
        glWidget->rotateCameraPitch(-rotateAngle);
        break;
    case Qt::Key_K:
        // tilt camera down => pitch down
        glWidget->rotateCameraPitch(rotateAngle);
        break;
    case Qt::Key_J:
        // pan left => yaw left (negative)
        glWidget->rotateCameraYaw(-rotateAngle);
        break;
    case Qt::Key_L:
        // pan right => yaw right (positive)
        glWidget->rotateCameraYaw(rotateAngle);
        break;
    case Qt::Key_M:

        // if (glWidget) {
        //     // Negative delta => reduce FOV => "zoom in"
        //     //glWidget->adjustCameraFov(-100.0f);
        //     glWidget->adjustCameraFov(-1.0f);
        // }
        translateStep *= 5;
        break;

        // Zoom out (increase FOV) => Key N
    case Qt::Key_N:
        // if (glWidget) {
        //     // Positive delta => increase FOV => "zoom out"
        //     glWidget->adjustCameraFov(1.0f);
        // }
        translateStep *= 0.2;
        break;
    case Qt::Key_Up:
    {
        if(reversal==true){

        }else{

        }
    }
     case Qt::Key_Down:
    {
        if(reversal==true){

        }else{

        }
    }
case Qt::Key_Left:
    {
    //here
        if(reversal==true){
        for(int i{0};i<20;i++){
            if(it!=coordinates.rend()-1){
            it++;
            glWidget->rocketX=std::get<0>(*it);
            glWidget->rocketZ=std::get<1>(*it);
            QVector3D t=*(--glWidget->markers.end());
            glWidget->markers.pop_back();
            glWidget->antimarkers.push_back(t);
            glWidget->update();
                }
        }
        }else{

        }
        break;
    }
case Qt::Key_Right:
    {
        if(reversal==true){
        for(int i{0};i<20;i++){
            if((it!=coordinates.rbegin()-1)&&(it!=coordinates.rbegin())){
            it--;
            glWidget->rocketX=std::get<0>(*it);
            glWidget->rocketZ=std::get<1>(*it);
            QVector3D t=*(--glWidget->antimarkers.end());
            glWidget->antimarkers.pop_back();
            glWidget->markers.push_back(t);
            glWidget->update();
            }
        }
        }else{

        }
        break;
    }
    default:

        QMainWindow::keyPressEvent(event);
        return;
    }

    event->accept();
}
//----------------------------
// RocketOpenGLWidget methods
//----------------------------
MainWindow::RocketOpenGLWidget::RocketOpenGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
    // Initialize camera to a nice position
    cameraX = 5000.0f;
    cameraY = 5000.0f;
    cameraZ = 0.0f;
    cameraPitch = 0.0f;
    cameraYaw = 0.0f;
    setFocusPolicy(Qt::StrongFocus);
    setMouseTracking(true);
    currentIndex = 0;
}

MainWindow::RocketOpenGLWidget::~RocketOpenGLWidget() {}

void MainWindow::RocketOpenGLWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);

    // White background
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void MainWindow::RocketOpenGLWidget::resizeGL(int w, int h)
{
    glViewport(0, 0, w, h);

    // Set up a simple perspective projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    GLfloat aspect = GLfloat(w) / GLfloat(h ? h : 1);
    // Using GLU-style perspective
    // If you don’t have glu.h, you can do your own frustum or use a matrix library
    // but let’s assume we can use gluPerspective:
    // gluPerspective(45.0, aspect, 0.1, 1000.0);

    // Without glu, you can do something like this (quick manual perspective):
    //const float fovY = 45.0f;
    const float nearPlane = 0.1f;
    const float farPlane = 10000.0f;
    float fovRad = cameraFov * 3.14159f / 180.0f; // or use qDegreesToRadians
    float f = 1.0f / std::tan(fovRad * 0.5f);
    float A = (farPlane + nearPlane) / (nearPlane - farPlane);
    float B = (2.0f * farPlane * nearPlane) / (nearPlane - farPlane);

    GLfloat perspective[16] = {f / aspect, 0, 0, 0, 0, f, 0, 0, 0, 0, A, -1, 0, 0, B, 0};
    glLoadMatrixf(perspective);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void MainWindow::RocketOpenGLWidget::paintGL()
{
    static int staticplaceholder = 0;
    static float staticplaceholder2 = -10000.0f;  // Initialize to a very negative value
    static int drawCounter = 0;
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // ---------------------------------------------------
    // CAMERA TRANSFORM (inverse of camera's position/orient)
    // ---------------------------------------------------
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 1. Rotate by -pitch around X
    glRotatef(-cameraPitch, 1.0f, 0.0f, 0.0f);
    // 2. Rotate by -yaw around Z
    glRotatef(-cameraYaw, 0.0f, 0.0f, 1.0f);

    // 3. Translate by -camera position
    glTranslatef(-cameraX, -cameraY, -cameraZ);

    // Initial camera setup (only first time)
    if (staticplaceholder == 0) {
        cameraPitch = 270.0f;
        cameraYaw = 0.0f;
        staticplaceholder++;
    }

    // Draw the XY plane grid
    drawXYPlane(5000.0f, 2000);

    // ---------------------------------------------------
    // DRAW ROCKET AND ORIENTED TRIAD
    // ---------------------------------------------------
    glPushMatrix();
    {
        if(markers.size()>0&&markers[0].x()==1000){
        // Move to rocket position
        glTranslatef(rocketX, rocketY, rocketZ);

        // Draw the oriented triad at the rocket position
        drawOrientedTriad(5.0f, currentOrientation);

        // Draw the rocket (red sphere)
        //zmenit podle faze letu
        glColor3f(1.0f, 0.0f, 0.0f); // red
        drawSphere(1.0f, 20, 20);
        }
    }
    glPopMatrix();

    // ---------------------------------------------------
    // DRAW MARKERS (small blue/green marks)
    // ---------------------------------------------------
    // Update marker colors (optional)
    if (a[currentIndex] < 0) {
        int gh = 60;
        gh++;
    } else {
        staticplaceholder++;
    }
    for (int i = 0; i < markers.size(); i++) {
        // First widget (trajectory view)
        if (markers[0].x() == 1000 && i > 0) {
            if (i < staticplaceholder) {
                glColor3f(0.0f, 0.0f, 1.0f);  // Blue
            } else {
                glColor3f(0.0f, 0.5f, 0.0f);  // Green (fixed value for 128.0f/255.0f)
            }
            glPushMatrix();
            glTranslatef(markers[i].x(), markers[i].y(), markers[i].z());
            drawXMesh(2.0f);
            glPopMatrix();
        }

        // Second widget (more sparse visualization)
        else if (markers[0].x() == -1000 && i > 0) {
            // Draw every 5 markers or if we've moved at least 2.0 units in X direction
            if (std::abs(markers[i].x() - staticplaceholder2) > 25.0) {
                staticplaceholder2 = markers[i].x();  // Update last drawn X position

                glPushMatrix();
                glTranslatef(markers[i].x(), markers[i].y(), markers[i].z());

                // Draw the oriented triad at the marker position
                drawOrientedTriad(5.0f, currentOrientation);

                // Color based on index
                if (i < staticplaceholder) {
                    glColor3f(0.0f, 0.0f, 1.0f);  // Blue
                } else {
                    glColor3f(0.0f, 0.5f, 0.0f);  // Green
                }

                // Draw the sphere
                drawSphere(1.0f, 20, 20);
                glPopMatrix();

                // Optionally increment a counter to track how many we've drawn
                drawCounter++;
            }
        }
    }


    // // Draw anti-markers if any (for reverse playback)
    // for (const auto& marker : antimarkers) {
    //     glColor3f(1.0f, 0.5f, 0.0f); // Orange to distinguish
    //     glPushMatrix();
    //     glTranslatef(marker.x(), marker.y(), marker.z());
    //     drawXMesh(2.0f);
    //     glPopMatrix();
    // }
}
//---------------------------------------
// Move camera in X, Y, Z (local or global axes)
//---------------------------------------
void MainWindow::RocketOpenGLWidget::moveCamera(float dx, float dy, float dz)
{
    // For simplicity, treat dx, dy, dz as movement in world space aligned with
    // camera axes. A more advanced approach rotates them by cameraYaw/pitch
    // so that W always moves “forward,” etc.

    // If you want W to move “forward” in the camera’s direction, you’d do something like:
    //    float radYaw   = qDegreesToRadians(cameraYaw);
    //    float forwardX = -std::sin(radYaw);
    //    float forwardZ = -std::cos(radYaw);
    //    cameraX += forwardX * dz;
    //    cameraZ += forwardZ * dz;
    // etc.
    // But here, we just do a naive translation in world coordinates:
    cameraX += dx;
    cameraY += dy;
    cameraZ += dz;

    update();
}

//---------------------------------------
// Rotate camera
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
// Set rocket position
//---------------------------------------
void MainWindow::RocketOpenGLWidget::setRocketPosition(float x, float y, float z)
{
    rocketX = x;
    rocketY = y;
    rocketZ = z;
    update();
}

//---------------------------------------
// Markers for trajectory or debug
//---------------------------------------
void MainWindow::RocketOpenGLWidget::addMarker(float x, float y, float z)
{
    markers.emplace_back(x, y, z);
}

// ?---------------------------------------
// Simple immediate-mode sphere
// ---------------------------------------
void MainWindow::RocketOpenGLWidget::drawSphere(float radius, int slices, int stacks)
{
    // Basic “UV” sphere
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

            // top vertex
            glNormal3f(x * zr0, y * zr0, z0);
            glVertex3f(radius * x * zr0, radius * y * zr0, radius * z0);

            // bottom vertex
            glNormal3f(x * zr1, y * zr1, z1);
            glVertex3f(radius * x * zr1, radius * y * zr1, radius * z1);
        }
        glEnd();
    }
}
void MainWindow::RocketOpenGLWidget::drawXMesh(float size)
{
    // For convenience, let's assume size is half the full line length.
    // So an X of size=1.0 extends from -1..+1 in both X and Y.

    glBegin(GL_LINES);

    // First diagonal: (-size, -size) to (+size, +size)
    glVertex3f(-size, -size, 0.0f);
    glVertex3f(size, size, 0.0f);

    // Second diagonal: (+size, -size) to (-size, +size)
    glVertex3f(size, -size, 0.0f);
    glVertex3f(-size, size, 0.0f);

    glEnd();
}
void MainWindow::RocketOpenGLWidget::drawXYPlane(float size, int divisions)
{
    // Draw a grid on the XY plane
    // size: total size of the grid from -size to +size in both X and Y directions
    // divisions: number of cells in each direction

    float step = (2.0f * size) / divisions;
    float halfSize = size;

    // Set color for the grid lines
    glColor3f(0.7f, 0.7f, 0.7f); // Light gray
    glLineWidth(1.0f);

    glBegin(GL_LINES);

    // Draw horizontal lines (parallel to X-axis)
    for (int i = 0; i <= divisions; ++i) {
        float y = -halfSize + i * step;
        glVertex3f(-halfSize, y, 0.0f);
        glVertex3f(halfSize, y, 0.0f);
    }

    // Draw vertical lines (parallel to Y-axis)
    for (int i = 0; i <= divisions; ++i) {
        float x = -halfSize + i * step;
        glVertex3f(x, -halfSize, 0.0f);
        glVertex3f(x, halfSize, 0.0f);
    }

    glEnd();

    // Draw coordinate axes with thicker lines and different colors
    glLineWidth(2.0f);

    // X-axis (red)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex3f(-halfSize, 0.0f, 0.0f);
    glVertex3f(halfSize, 0.0f, 0.0f);
    glEnd();

    // Y-axis (green)
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, -halfSize, 0.0f);
    glVertex3f(0.0f, halfSize, 0.0f);
    glEnd();

    // Reset line width
    glLineWidth(1.0f);
}
// void MainWindow::RocketOpenGLWidget::lookAt(float cx, float cy, float cz)
// {
//     float dx = cx - cameraX;
//     float dy = cy - cameraY;
//     float dz = cz - cameraZ;

//     // Compute yaw
//     // If the camera is originally looking down -Z,
//     // we often do: yaw = atan2(dx, -dz).
//     float newYaw = std::atan2(dx, -dz) * (180.0f / M_PI);

//     // Compute pitch
//     float horizontalLen = std::sqrt(dx*dx + dz*dz);
//     float newPitch = std::atan2(dy, horizontalLen) * (180.0f / M_PI);

//     cameraYaw   = newYaw;
//     cameraPitch = newPitch;
// }

void MainWindow::on_pushButton_clicked()
{

}


void MainWindow::on_pushButton_released()
{

}


void MainWindow::on_checkBox_checkStateChanged(const Qt::CheckState &arg1)
{
    if(reversal==false){
        reversal=true;
        it=coordinates.rbegin();
    }else{reversal=false;}
}
void MainWindow::RocketOpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        rightMousePressed = true;
        lastMousePos = event->pos();
    }
    event->accept();
}

void MainWindow::RocketOpenGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton) {
        rightMousePressed = false;
    }
    event->accept();
}

void MainWindow::RocketOpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (rightMousePressed) {
        // Calculate mouse movement delta
        QPoint delta = event->pos() - lastMousePos;

        // Update camera yaw and pitch based on mouse movement
        // You may need to adjust these sensitivity values
        float sensitivity = 0.2f;
        rotateCameraYaw(-delta.x() * sensitivity);
        rotateCameraPitch(-delta.y() * sensitivity);

        // Update last position
        lastMousePos = event->pos();
    }
    event->accept();
}

void MainWindow::handlePageChange(int index)
{
    // When the page changes, we need to ensure the OpenGL context is properly maintained
    if (index == 0) {
        // First page is active
        if (glWidget) {
            // Request a delayed update to ensure the widget is visible first
            QTimer::singleShot(10, [this](){
                glWidget->makeCurrent();
                glWidget->update();
            });
        }
    } else if (index == 1) {
        // Second page is active
        if (glWidget2) {
            // Request a delayed update to ensure the widget is visible first
            QTimer::singleShot(10, [this](){
                glWidget2->makeCurrent();
                glWidget2->update();
            });
        }
    }
}

void MainWindow::on_pushButton_2_clicked()
{
    ui->stackedWidget->setCurrentIndex(1);

    // Request a delayed update to ensure the widget is visible first
    QTimer::singleShot(10, [this](){
        if (glWidget2) {
            glWidget2->makeCurrent();
            glWidget2->update();
        }
    });
}

void MainWindow::on_pushButton_3_clicked()
{
    ui->stackedWidget->setCurrentIndex(0);

    // Request a delayed update to ensure the widget is visible first
    QTimer::singleShot(10, [this](){
        if (glWidget) {
            glWidget->makeCurrent();
            glWidget->update();
        }
    });
}
void MainWindow::setupStackedWidgetConnections()
{
    // Connect the stacked widget's currentChanged signal to handle OpenGL widget visibility
    connect(ui->stackedWidget, &QStackedWidget::currentChanged, this, &MainWindow::handlePageChange);

    // Connect the buttons to switch pages
    connect(ui->pushButton_2, &QPushButton::clicked, this, &MainWindow::on_pushButton_2_clicked);
    connect(ui->pushButton_3, &QPushButton::clicked, this, &MainWindow::on_pushButton_3_clicked);
}
void MainWindow::initializeOpenGLWidgets()
{
    // Don't directly call initializeGL() as that's Qt's job
    // Instead, prepare both widgets and force an update

    // Force both widgets to be initially visible to ensure proper initialization
    ui->stackedWidget->setCurrentIndex(0);
    QApplication::processEvents(); // Process the widget show event

    // Set up first widget
    glWidget->makeCurrent();
    glWidget->update();

    // Set up second widget with different camera perspective
    glWidget2->makeCurrent();
    glWidget2->cameraX = 0.0f;
    glWidget2->cameraY = 20.0f;  // Top-down view
    glWidget2->cameraZ = 0.0f;
    glWidget2->cameraPitch = 270.0f;  // Look straight down
    glWidget2->cameraYaw = 0.0f;
    glWidget2->update();

    // Make sure both widgets are initialized with orientation data
    if (!orientationQuaternions.empty()) {
        glWidget->currentOrientation = orientationQuaternions[0];
        glWidget2->currentOrientation = orientationQuaternions[0];
    }

    // Restore original page if needed
    if (ui->stackedWidget->currentIndex() != 0) {
        ui->stackedWidget->setCurrentIndex(1);
        QApplication::processEvents();
    }
}
void MainWindow::RocketOpenGLWidget::drawTriad(float size)
{
    // Draw XYZ axes with distinctive colors
    // size: length of each axis line

    // Save current line width
    GLfloat currentWidth;
    glGetFloatv(GL_LINE_WIDTH, &currentWidth);

    // Set line width for better visibility
    glLineWidth(2.0f);

    glBegin(GL_LINES);

    // X-axis (red)
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size, 0.0f, 0.0f);

    // Y-axis (green)
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, size, 0.0f);

    // Z-axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, size);

    glEnd();

    // Restore previous line width
    glLineWidth(currentWidth);
}

void MainWindow::RocketOpenGLWidget::drawCone(float height, float radius, int segments)
{
    float angleStep = 2.0f * M_PI / segments;

    // Draw cone sides
    glBegin(GL_TRIANGLE_FAN);
    // Cone apex
    glVertex3f(0.0f, 0.0f, height);

    // Base vertices
    for (int i = 0; i <= segments; i++) {
        float angle = i * angleStep;
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);
        glVertex3f(x, y, 0.0f);
    }
    glEnd();

    // Draw cone base
    glBegin(GL_TRIANGLE_FAN);
    // Base center
    glVertex3f(0.0f, 0.0f, 0.0f);

    // Base perimeter
    for (int i = segments; i >= 0; i--) {
        float angle = i * angleStep;
        float x = radius * cosf(angle);
        float y = radius * sinf(angle);
        glVertex3f(x, y, 0.0f);
    }
    glEnd();
}

void MainWindow::RocketOpenGLWidget::drawOrientedTriad(float size, const Eigen::Quaterniond& orientation)
{
    // Draw XYZ axes oriented according to rocket's rotation
    // X-axis will be in the direction of flight
    // size: length of each axis line

    // Save current line width
    GLfloat currentWidth;
    glGetFloatv(GL_LINE_WIDTH, &currentWidth);

    // Set line width for better visibility
    glLineWidth(2.0f);

    // Get rotation matrix from quaternion
    Eigen::Matrix3d rotationMatrix = orientation.toRotationMatrix();

    // Get the three basis vectors (direction vectors for each axis)
    Eigen::Vector3d xAxis = rotationMatrix * Eigen::Vector3d(1.0, 0.0, 0.0);
    Eigen::Vector3d yAxis = rotationMatrix * Eigen::Vector3d(0.0, 1.0, 0.0);
    Eigen::Vector3d zAxis = rotationMatrix * Eigen::Vector3d(0.0, 0.0, 1.0);

    // Draw the oriented axes
    glBegin(GL_LINES);

    // X-axis (red) - direction of flight
    glColor3f(1.0f, 0.0f, 0.0f); // Red
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * xAxis.x(), size * xAxis.y(), size * xAxis.z());

    // Y-axis (green)
    glColor3f(0.0f, 1.0f, 0.0f); // Green
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * yAxis.x(), size * yAxis.y(), size * yAxis.z());

    // Z-axis (blue)
    glColor3f(0.0f, 0.0f, 1.0f); // Blue
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(size * zAxis.x(), size * zAxis.y(), size * zAxis.z());

    glEnd();

    // Draw arrow cones at the end of each axis
    // X-axis arrow (red)
    glColor3f(1.0f, 0.0f, 0.0f);
    glPushMatrix();
    glTranslatef(size * xAxis.x(), size * xAxis.y(), size * xAxis.z());

    // Calculate rotation to align the cone with the axis
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

    // Y-axis arrow (green)
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

    // Z-axis arrow (blue)
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

    // Restore previous line width
    glLineWidth(currentWidth);
}

void MainWindow::setupComboBox()
{
    // Block signals temporarily to prevent triggering the slot during setup
    ui->comboBox->blockSignals(true);

    // Clear and add items
    ui->comboBox->clear();
    ui->comboBox->addItem("Free Camera");
    ui->comboBox->addItem("Follow Rocket");

    // Set initial index without triggering the slot
    ui->comboBox->setCurrentIndex(0);
    follow = false; // Initialize to free camera mode

    // Re-enable signals
    ui->comboBox->blockSignals(false);

    // Connect the signal to the slot (use == for comparison, not =)
    disconnect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
               this, &MainWindow::on_comboBox_currentIndexChanged);
    connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::on_comboBox_currentIndexChanged);
}

void MainWindow::on_comboBox_currentIndexChanged(int index)
{
    // Debug output to verify function is being called
    std::cout << "ComboBox changed to index: " << index << std::endl;

    // Use == for comparison, not = (which is assignment)
    if (index == 1) {
        follow = true;
        std::cout << "Follow mode enabled" << std::endl;

        // Set initial follow camera position
        glWidget->cameraX = float(*x);
        glWidget->cameraY = 100.0f;
        glWidget->cameraZ = float(*h);
        glWidget->cameraYaw = 0.0f;

        glWidget2->cameraX = float(*x);
        glWidget2->cameraY = 1000.0f;
        glWidget2->cameraZ = float(*h);

        // Update the widgets
        glWidget->update();
        glWidget2->update();
    }
    else if (index == 0) {
        follow = false;
        std::cout << "Free camera mode enabled" << std::endl;
    }
}
void MainWindow::setupCCCP() {
    // Create CCCP manager
    m_cccp = std::make_unique<CCCPManager>(this);

    // Connect signals
    connect(m_cccp.get(), &CCCPManager::accelerometerDataReceived,
            this, &MainWindow::onAccelerometerData);
    connect(m_cccp.get(), &CCCPManager::attitudeDataReceived,
            this, &MainWindow::onAttitudeData);
    connect(m_cccp.get(), &CCCPManager::telemetryReceived,
            this, &MainWindow::onTelemetryReceived);
    connect(m_cccp.get(), &CCCPManager::connectionStatusChanged,
            this, &MainWindow::onConnectionStatusChanged);
    connect(m_cccp.get(), &CCCPManager::errorOccurred,
            this, &MainWindow::onCCCPError);

    // Add status labels to UI
    m_connectionStatus = new QLabel(this);
    m_connectionStatus->setStyleSheet("color: red; font-size: 16px;");
    m_connectionStatus->setText("Disconnected");
    m_connectionStatus->setGeometry(10, 70, 200, 30);

    m_telemetryLabel = new QLabel(this);
    m_telemetryLabel->setStyleSheet("color: white; font-size: 14px;");
    m_telemetryLabel->setGeometry(10, 100, 300, 60);

    // Note: LiveTelemetry checkbox is now handled by on_LiveTelemtry_checkStateChanged slot

    // Initialize but don't start automatically
    if (!m_cccp->initialize()) {
        QMessageBox::warning(this, "CCCP Error",
                             "Failed to initialize CCCP communication");
    }
}

void MainWindow::onAccelerometerData(const AccelerometerData& data) {
    m_currentAccel = data;

    if (m_useLiveTelemetry) {
        // Update visualization with real accelerometer data
        updateRocketFromTelemetry();

        // Display current values
        QString accelText = QString("Accel: X=%1 Y=%2 Z=%3 m/s²")
                                .arg(data.x, 0, 'f', 2)
                                .arg(data.y, 0, 'f', 2)
                                .arg(data.z, 0, 'f', 2);
        m_telemetryLabel->setText(accelText);
    }
}

void MainWindow::onAttitudeData(const AttitudeData& data) {
    m_currentAttitude = data;

    if (m_useLiveTelemetry && glWidget) {
        // Use quaternion directly
        Eigen::Quaterniond q = data.toQuaternion();

        // Update rocket orientation
        glWidget->currentOrientation = q;
        glWidget2->currentOrientation = q;

        // Convert to Euler angles for display
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX convention
        double yaw = euler[0];
        double pitch = euler[1];
        double roll = euler[2];

        // Display attitude
        QString attitudeText = QString("\nAttitude: R=%1° P=%2° Y=%3°")
                                   .arg(qRadiansToDegrees(roll), 0, 'f', 1)
                                   .arg(qRadiansToDegrees(pitch), 0, 'f', 1)
                                   .arg(qRadiansToDegrees(yaw), 0, 'f', 1);
        m_telemetryLabel->setText(m_telemetryLabel->text() + attitudeText);

        // Also display quaternion
        QString quatText = QString("\nQuat: w=%1 x=%2 y=%3 z=%4")
                               .arg(data.w, 0, 'f', 3)
                               .arg(data.x, 0, 'f', 3)
                               .arg(data.y, 0, 'f', 3)
                               .arg(data.z, 0, 'f', 3);
        m_telemetryLabel->setText(m_telemetryLabel->text() + quatText);
    }
}

void MainWindow::onTelemetryReceived(const QString& message) {
    qDebug() << "Telemetry:" << message;
}

void MainWindow::onConnectionStatusChanged(bool connected) {
    if (connected) {
        m_connectionStatus->setStyleSheet("color: green; font-size: 16px;");
        m_connectionStatus->setText("Connected");
    } else {
        m_connectionStatus->setStyleSheet("color: red; font-size: 16px;");
        m_connectionStatus->setText("Disconnected");
    }
}

void MainWindow::onCCCPError(const QString& error) {
    qDebug() << "CCCP Error:" << error;
    QMessageBox::warning(this, "CCCP Error", error);
}

void MainWindow::updateRocketFromTelemetry() {
    if (!m_useLiveTelemetry) return;

    // Simple integration of acceleration to update position
    static double velocityX = 0, velocityZ = 0;
    const double dt = 0.01;  // 10ms update rate

    // Update velocity from acceleration
    velocityX += m_currentAccel.x * dt;
    velocityZ += m_currentAccel.z * dt;

    // Update position from velocity
    *x += velocityX * dt;
    *h += velocityZ * dt;

    // Update rocket visualization
    glWidget->setRocketPosition(float(*x), 0.0f, float(*h));
    glWidget2->setRocketPosition(float(*x), 0.0f, float(*h));

    // Add position marker
    glWidget->addMarker(float(*x), 0.0f, float(*h));
    glWidget2->addMarker(float(*x), 0.0f, float(*h));

    // Store coordinates for replay
    coordinates.push_back(std::make_tuple(*x, *h));
}

void MainWindow::on_LiveTelemtry_checkStateChanged(const Qt::CheckState &arg1)
{
    // Update the telemetry flag based on checkbox state
    m_useLiveTelemetry = (arg1 == Qt::Checked);

    if (m_useLiveTelemetry) {
        // Start CCCP if not already running
        if (m_cccp && !m_cccp->isRunning()) {
            m_cccp->start();
            qDebug() << "Live telemetry enabled - CCCP started";
        }

        // Stop simulation timer to avoid conflicts
        if (timer && timer->isActive()) {
            timer->stop();
            qDebug() << "Simulation timer stopped";
        }

        // Update status display
        if (m_connectionStatus) {
            m_connectionStatus->setText("Waiting for connection...");
        }
    } else {
        // Stop CCCP when unchecked
        if (m_cccp && m_cccp->isRunning()) {
            m_cccp->stop();
            qDebug() << "Live telemetry disabled - CCCP stopped";
        }

        // Restart simulation timer
        if (timer && !timer->isActive()) {
            timer->start(10);  // 10ms = 100Hz
            qDebug() << "Simulation timer restarted";
        }

        // Update status display
        if (m_connectionStatus) {
            m_connectionStatus->setText("Disconnected");
            m_connectionStatus->setStyleSheet("color: red; font-size: 16px;");
        }

        // Clear telemetry display
        if (m_telemetryLabel) {
            m_telemetryLabel->clear();
        }
    }
}

