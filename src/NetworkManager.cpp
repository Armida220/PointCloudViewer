#include <memory>

#include <QtCore/QDateTime>
#include <QtCore/QDataStream>

#include <draco/io/ply_decoder.h>
#include <draco/core/decoder_buffer.h>

#include "Tracer.h"
#include "NetworkManager.h"

const int SAMPLE_CNT = 4;
const int MAX_POINT_CNT = 1000;
extern bool g_is_debug_mode;

NetworkManager::NetworkManager(QObject *parent) :
    QObject(parent),
    socket_(new QUdpSocket) {
}

NetworkManager::~NetworkManager() = default;

void NetworkManager::getRawPointCloudData() {
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
    std::cout << "Network Manager is listening to port: " << port_num << " mode: " << g_is_debug_mode << std::endl;
    socket_->bind(QHostAddress::AnyIPv4, port_num);
    connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getRawPointCloudData()));
    if(g_is_debug_mode) connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getRawPointCloudData()));
    else connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getDracoPointCloudData()));
}

void NetworkManager::getDracoPointCloudData() {
    QByteArray udp_buffer;
    udp_buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(udp_buffer.data(), udp_buffer.size(), &sender, &senderPort);

    std::unique_ptr<draco::DecoderBuffer> draco_buffer(new draco::DecoderBuffer);
    draco_buffer->Init(udp_buffer.data(), udp_buffer.size());

    std::unique_ptr<draco::PointCloud> draco_point_cloud(new draco::PointCloud); 
    draco::PlyDecoder ply_decoder;
    bool result = ply_decoder.DecodeFromBuffer(draco_buffer.get(), draco_point_cloud.get());
    if(!result)
    {
        std::cout << "unknown udp buffer, plz check." << std::endl;
        return;
    } else {
        std::cout << "receive udp buffer size: " << udp_buffer.size() << std::endl;
    }

    auto num_points = draco_point_cloud->num_points();
    //std::cout << "convert draco to pcd, point num: " << num_points << std::endl;

    auto pc_att = draco_point_cloud->GetNamedAttribute(draco::GeometryAttribute::POSITION);

    float att_val[3];
    PointArray points;
    points.reserve(num_points);
    for (draco::AttributeValueIndex i(0); i < pc_att->size(); ++i) {
        pc_att->GetValue(i, att_val);

        //if(i.value() % 500 == 0) std::cout << i.value() << ": " << att_val[0] << " " << att_val[1] << " " << att_val[2] << std::endl;
        points.push_back(Point(att_val[0], att_val[1], att_val[2]));
    }

    //all done
    emit emitPointCloud(points);
}
