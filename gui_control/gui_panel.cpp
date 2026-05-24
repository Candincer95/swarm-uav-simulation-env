#include <QApplication>
#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QLabel>
#include <QSlider>
#include <QString>
#include <QPainter>  // For drawing map
#include <QPixmap>   // For map ground
#include <QPen>      // For border lines
#include <QColor>    // For colors
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>
#include <cmath>     // For math
#include <cstdlib> 
#include <thread> 

class ControlPanel : public QWidget {
private:
    int sockfd;
    struct sockaddr_in servaddr;
    QLabel *statusLabel;
    QSlider *windSlider;
    QSlider *dirSlider;

    QPixmap *mapPixmap;
    QLabel *mapRenderLabel;

    QLabel *fireAlertLabel;
    
    // Radar pixel dimensions on the UI
    const int mapCanvasWidth = 400;
    const int mapCanvasHeight = 440;

    // Real-world radar limits (According to the limits in Forest_world.sdf)
    const float min_x = -100.0f;
    const float max_x = 100.0f;
    const float min_y = -10.0f;
    const float max_y = 170.0f;

    float worldWidth() const { return max_x - min_x; }
    float worldHeight() const { return max_y - min_y; }

public:
    ControlPanel(QWidget *parent = nullptr) : QWidget(parent) {

        // Initialize UDP Socket
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        memset(&servaddr, 0, sizeof(servaddr));
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(9090); // Target port
        servaddr.sin_addr.s_addr = inet_addr("127.0.0.1"); // Localhost

        // Main Window Settings
        setWindowTitle("Swarm UAV Mission Control");
        resize(500, 950);
        QVBoxLayout *mainLayout = new QVBoxLayout(this);

        // Title
        QLabel *titleLabel = new QLabel("Swarm Control Center", this);
        titleLabel->setAlignment(Qt::AlignCenter);
        titleLabel->setStyleSheet("font-size: 16pt; font-weight: bold; margin-bottom: 10px;");
        mainLayout->addWidget(titleLabel);

        // Sensor Fault Injection
        QGroupBox *attackGroup = new QGroupBox("Sensor Fault Injection (GPS)", this);
        attackGroup->setStyleSheet("font-weight: bold;");
        QVBoxLayout *attackLayout = new QVBoxLayout(attackGroup);

        QPushButton *btnDeny = new QPushButton("Inject GPS Denial (Block)", this);
        QPushButton *btnSpoof = new QPushButton("Inject GPS Noise", this);
        QPushButton *btnRecover = new QPushButton("Recover to Normal", this);

        btnDeny->setStyleSheet("background-color: #ff4d4d; color: white; padding: 10px; border-radius: 5px;");
        btnSpoof->setStyleSheet("background-color: #ff9933; color: white; padding: 10px; border-radius: 5px;");
        btnRecover->setStyleSheet("background-color: #4CAF50; color: white; padding: 10px; border-radius: 5px;");

        attackLayout->addWidget(btnDeny);
        attackLayout->addWidget(btnSpoof);
        attackLayout->addWidget(btnRecover);
        mainLayout->addWidget(attackGroup);

        // Environment Dynamics Section (WIND SPEED AND DIRECTION)
        QGroupBox *envGroup = new QGroupBox("Environment Dynamics", this);
        envGroup->setStyleSheet("font-weight: bold; margin-top: 10px;");
        QVBoxLayout *envLayout = new QVBoxLayout(envGroup);

        // Wind Speed
        QLabel *windLabel = new QLabel("Wind Intensity: 0%", this);
        windSlider = new QSlider(Qt::Horizontal, this);
        windSlider->setRange(0, 100);
        windSlider->setValue(0);

        // Wind Direction
        QLabel *dirLabel = new QLabel("Wind Direction: 0° (North)", this);
        dirSlider = new QSlider(Qt::Horizontal, this);
        dirSlider->setRange(0,359);
        dirSlider->setValue(0);

        QPushButton *btnWind = new QPushButton("Apply Wind", this);
        btnWind->setStyleSheet("background-color: #3498db; color: white; padding: 10px; border-radius: 5px; font-weight: bold;");

        envLayout->addWidget(windLabel);
        envLayout->addWidget(windSlider);
        envLayout->addSpacing(10);
        envLayout->addWidget(dirLabel);
        envLayout->addWidget(dirSlider);
        envLayout->addSpacing(10);
        envLayout->addWidget(btnWind);
        mainLayout->addWidget(envGroup);

        // MAP (RADAR) SECTION
        QGroupBox *mapGroup = new QGroupBox("Swarm Mission Area (Forest Radar)", this);
        mapGroup->setStyleSheet("font-weight: bold; margin-top: 10px");
        QVBoxLayout *mapLayout = new QVBoxLayout(mapGroup);

        mapRenderLabel = new QLabel(this);
        mapRenderLabel->setFixedSize(mapCanvasWidth, mapCanvasHeight);

        mapPixmap = new QPixmap(mapCanvasWidth, mapCanvasHeight);
        mapPixmap->fill(QColor("#1e3f20")); // Forest ground (Dark Green)

        QPainter painter(mapPixmap);
        painter.setPen(QPen(Qt::red, 2, Qt::DashLine));

        int forest_px_left = static_cast<int>((-80.0f - min_x) / worldWidth() * mapCanvasWidth);
        int forest_px_right = static_cast<int>((80.0f - min_x) / worldWidth() * mapCanvasWidth);
        int forest_py_top = static_cast<int>((max_y -150.0f) / worldHeight() * mapCanvasHeight);
        int forest_py_bottom = static_cast<int>((max_y - 10.0f) / worldHeight() * mapCanvasHeight);

        painter.drawRect(forest_px_left, forest_py_top, forest_px_right - forest_px_left, forest_py_bottom - forest_py_top);

        painter.setPen(Qt::white);
        painter.drawText(forest_px_left + 5, forest_py_top + 15, "Target Forest Area");
        painter.end();

        mapRenderLabel->setPixmap(*mapPixmap);
        mapLayout->addWidget(mapRenderLabel);
        mainLayout->addWidget(mapGroup);

        // FIRE ALERT BANNER
        fireAlertLabel = new QLabel("DANGER: FIRE DETECTED AT SECTOR!", this);
        fireAlertLabel->setAlignment(Qt::AlignCenter);
        fireAlertLabel->setStyleSheet("background-color: red; color: white; font-size: 16pt; font-weight: bold; padding: 15px; border-radius: 8px;");
        fireAlertLabel->hide(); // Hidden at the beginning
        mainLayout->addWidget(fireAlertLabel);

        // Status Bar
        statusLabel = new QLabel("System Ready. Waiting for telemetry...", this);
        statusLabel->setAlignment(Qt::AlignCenter);
        statusLabel->setStyleSheet("color: gray; font-style: italic; margin-top: 15px; font-weight: normal;");
        mainLayout->addWidget(statusLabel);

        // DYNAMIC LABEL UPDATES
        QObject::connect(windSlider, &QSlider::valueChanged, [windLabel](int val) {
            windLabel->setText(QString("Wind Intensity: %1%").arg(val));
        });
        
        QObject::connect(dirSlider, &QSlider::valueChanged, [dirLabel](int val) {
            dirLabel->setText(QString("Wind Direction: %1°").arg(val));
        });

        // Connect Attack Buttons to UDP Sender
        QObject::connect(btnDeny, &QPushButton::clicked, [this]() { sendString("1"); });
        QObject::connect(btnSpoof, &QPushButton::clicked, [this]() { sendString("2"); });
        QObject::connect(btnRecover, &QPushButton::clicked, [this]() { sendString("3"); });

        
        QObject::connect(btnWind, &QPushButton::clicked, [this]() { 
            int wind_percentage = windSlider->value();
            int direction_deg = dirSlider->value();

            
            float wind_m_s = (wind_percentage / 100.0f) * 10.0f;
            float rad = direction_deg * (M_PI / 180.0f);
            
            float wind_x = wind_m_s * std::cos(rad);
            float wind_y = wind_m_s * std::sin(rad);

            
            QString str_x = QString::number(wind_x, 'f', 2).replace(",", ".");
            QString str_y = QString::number(wind_y, 'f', 2).replace(",", ".");

            
            QString command = QString("gz topic -t \"/world/Forest_world/wind\" -m gz.msgs.Wind -p 'linear_velocity {x: %1 y: %2 z: 0.0} enable_wind: true' > /dev/null 2>&1")
                              .arg(str_x)
                              .arg(str_y);

            
            std::system(command.toStdString().c_str());

            
            statusLabel->setText(QString("Direct Environment Set: %1% Intensity at %2°").arg(wind_percentage).arg(direction_deg));
            statusLabel->setStyleSheet("color: #3498db; font-style: italic; margin-top: 15px; font-weight: bold;");
        });

        // TELEMETRY LISTENER
        std::thread([this]() {
            int listen_fd = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in listen_addr;
            memset(&listen_addr, 0, sizeof(listen_addr));
            listen_addr.sin_family = AF_INET;
            listen_addr.sin_addr.s_addr = INADDR_ANY;
            listen_addr.sin_port = htons(9091); // GUI listens on Port 9091
            
            bind(listen_fd, (const struct sockaddr *)&listen_addr, sizeof(listen_addr));
            char buffer[256];
            
            while(true) {
                int n = recvfrom(listen_fd, (char *)buffer, sizeof(buffer) -1, MSG_WAITALL, NULL, NULL);
                if (n >= 0) {
                    buffer[n] = '\0';
                    std::string msg(buffer);
                    
                    if (msg[0] == 'T') {
                        int id;
                        float x, y, fov;
                        if (sscanf(msg.c_str(), "T:%d:%f:%f:%f", &id, &x, &y, &fov) == 4) {

                            std::cout << "[GUI] Drone " << id << " -> X: " << x << " | Y: " << y << " | FOV: " << fov << std::endl;

                            QMetaObject::invokeMethod(this, [this, x, y, fov]() {
                                this->updateSwarmCoverage(x, y, fov);
                            });
                        } 
                    } 
                    else if (msg[0] == 'F') {
                        int id;
                        float f_x, f_y;
                        if (sscanf(msg.c_str(), "F:%d:%f:%f", &id, &f_x, &f_y) == 3) {
                            
                            
                            QMetaObject::invokeMethod(this, [this, id, f_x, f_y]() {
                                fireAlertLabel->setText(QString("🔥 FIRE DETECTED BY DRONE %1 AT X: %2 | Y: %3 🔥").arg(id).arg(f_x, 0, 'f', 1).arg(f_y, 0, 'f', 1));
                                fireAlertLabel->show(); // Make the alarm visible
                            });
                        }
                    }
                }
            }
        }).detach();
    }

    ~ControlPanel() {
        if (sockfd >= 0) ::close(sockfd);
    }

    void sendString(const std::string& cmdStr) {
        int n = sendto(sockfd, cmdStr.c_str(), cmdStr.length(),
                       MSG_CONFIRM, (const struct sockaddr *) &servaddr,
                       sizeof(servaddr));
        
        if (n < 0) {
            statusLabel->setText("Network Error: Failed to send command!");
            statusLabel->setStyleSheet("color: red; font-weight: bold;");
        } else {
            
            statusLabel->setText(QString("Last Signal Sent: Fault Injection Command %1").arg(cmdStr.c_str()));
            statusLabel->setStyleSheet("color: gray; font-style: italic; margin-top: 15px; font-weight: normal;");
        }
    }

    void updateSwarmCoverage(float drone_x, float drone_y, float fov_radius_m) {
        if (drone_x < min_x || drone_x > max_x || drone_y < min_y || drone_y > max_y) return;

        int px = static_cast<int>((drone_x - min_x) / worldWidth() * mapCanvasWidth);
        int py = static_cast<int>((max_y - drone_y) / worldHeight() * mapCanvasHeight);
        int p_radius = static_cast<int>(fov_radius_m / worldWidth() * mapCanvasWidth);

        QPainter painter(mapPixmap);

        painter.setBrush(QColor(135, 211, 124, 60));
        painter.setPen(Qt::NoPen);
        painter.drawEllipse(QPoint(px, py), p_radius, p_radius);

        painter.setBrush(QColor(255, 215, 0));
        painter.drawEllipse(QPoint(px,py), 3, 3);

        painter.end();
        mapRenderLabel->setPixmap(*mapPixmap);
    }
};

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    ControlPanel panel;
    panel.show();
    return app.exec();
}