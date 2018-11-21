#include <QtCore/QDateTime>
#include <QtCore/QDataStream>

#include "Tracer.h"
#include "NetworkManager.h"

const int SAMPLE_CNT = 4;
const int MAX_POINT_CNT = 1000;

NetworkManager::NetworkManager(QObject *parent) :
    QObject(parent),
    socket_(new QUdpSocket) {
}

NetworkManager::~NetworkManager() = default;

void NetworkManager::getPointCloudData() {
    static uint32_t point_cnt = 0;

    QByteArray buffer;
    buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(buffer.data(), buffer.size(), &sender, &senderPort);

    PointArray points;
    points.reserve(MAX_POINT_CNT);

    QDataStream out(&buffer, QIODevice::ReadOnly);
    double oriX, oriY, oriZ;  //utm
    out >> oriX >> oriY >> oriZ;
    if (std::isnan(oriX) || std::isnan(oriY) || std::isnan(oriZ)) return;
    if (point_cnt++ % SAMPLE_CNT == 0) {
        Point ori_point(oriX, oriY, oriZ);

        //points.push_back(ori_point);
    }

    int32_t offX, offY, offZ;
    while (!out.atEnd()) {
        out >> offX >> offY >> offZ;
        if (std::isnan(offX) || std::isnan(offY) || std::isnan(offZ)) continue;
        double rx = 0.001 * offX;  //relative pos
        double ry = 0.001 * offY;
        double rz = 0.001 * offZ;
        if(rx > 5e3 || rx < -5e3 || ry> 5e3 || ry<-5e3 || rz>1e3 || rz<-1e3) continue;
        if (point_cnt++ % SAMPLE_CNT == 0) {
            double ax = oriX + rx - g_utm_x;
            double ay = oriY + ry - g_utm_y;
            double az = oriZ + rz;

            Point ori_point(ax, ay, az);
            points.push_back(ori_point);
        }
    }
//    if(points.size() <= 100) return;

    emit emitPointCloud(points);
}

void NetworkManager::setPortNum(unsigned short port_num) {
    std::cout << "Network Manager is listening to port: " << port_num << std::endl;
    socket_->bind(QHostAddress::AnyIPv4, port_num);
    connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getPointCloudData()));
}
