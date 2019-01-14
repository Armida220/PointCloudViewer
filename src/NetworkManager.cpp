#include <memory>

#include <QtCore/QDateTime>
#include <QtCore/QDataStream>

#include <draco/io/ply_decoder.h>
#include <draco/core/decoder_buffer.h>
#include <draco/compression/decode.h>
#include <include/NetworkManager.h>


#include "Tracer.h"
#include "NetworkManager.h"

const int SAMPLE_CNT = 4;
const int MAX_POINT_CNT = 1000;

NetworkManager::NetworkManager(QObject *parent) :
    QObject(parent),
    socket_(new QUdpSocket),
    test_socket_(new QUdpSocket),
    mode(MODE::POINTCLOUD) {
    test_socket_->bind(QHostAddress::AnyIPv4, 1234);
}

NetworkManager::~NetworkManager() = default;

void NetworkManager::setPortNum(unsigned short port_num) {
    std::cout << this->objectName().toStdString() << " is listening to port: " << port_num << std::endl;
    socket_->bind(QHostAddress::AnyIPv4, port_num);
    if(mode == MODE::POINTCLOUD) connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getDracoPointCloudData()));
    else if(mode == MODE::STATUSINFO) connect(socket_.data(), SIGNAL(readyRead()), this, SLOT(getStatusInfo()));
    else std::cout << "aha?" << std::endl;
}

void NetworkManager::getDracoPointCloudData() {
    QByteArray udp_buffer;
    udp_buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(udp_buffer.data(), udp_buffer.size(), &sender, &senderPort);

    draco::DecoderBuffer draco_buffer;
    draco_buffer.Init(udp_buffer.data(), udp_buffer.size());
    std::cout << this->objectName().toStdString() << " receive udp buffer size: " << udp_buffer.size() << std::endl;

    std::unique_ptr<draco::PointCloud> draco_point_cloud;
    auto type_statusor = draco::Decoder::GetEncodedGeometryType(&draco_buffer);
    if (!type_statusor.ok()) {
        std::cout << "get a type statusor error, plz check" << std::endl;
        return;
    }
    const draco::EncodedGeometryType geom_type = type_statusor.value();
    if (geom_type == draco::POINT_CLOUD) {
        draco::Decoder decoder;
        auto statusor = decoder.DecodePointCloudFromBuffer(&draco_buffer);
        if (!statusor.ok()) {
            std::cout << "get a decode error" << std::endl;
            return;
        }
        draco_point_cloud = std::move(statusor).value();
        if (draco_point_cloud == nullptr) {
            std::cout << "Failed to decode the input file" << std::endl;
            return;
        }
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

    setPacket(points);
    //all done
    emit emitPointCloud(points);
}

void NetworkManager::getStatusInfo() {
    QByteArray udp_buffer;
    udp_buffer.resize(socket_->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    socket_->readDatagram(udp_buffer.data(), udp_buffer.size(), &sender, &senderPort);
    std::cout << this->objectName().toStdString() << " get udp buffer size: " << udp_buffer.size() << std::endl;

    GpsMsgs status_info;
    memcpy(&status_info, udp_buffer, sizeof(GpsMsgs));

    Point uav_pos(status_info.x, status_info.y, status_info.z);
    std::cout << "uav_pos: " << uav_pos << std::endl;
    emit emitUAVPos(uav_pos);

    Point gps_loc(status_info.lat, status_info.lon, status_info.hei);
    emit emitGPSLocation(gps_loc);

    QString satellite_num_str = QString::number(status_info.satellites);
    emit emitSatelliteNum(satellite_num_str);

    bool rtk_state = status_info.rtk_state == 50;
    emit emitRTKStatus(rtk_state);

    emit emitGPSLock(status_info.gps_lock);

    emit emitKineticAlignment(status_info.kinetic_alignment);
}

void NetworkManager::setMode(const NetworkManager::MODE &m) {
    mode = m;
}

void NetworkManager::setPacket(const PointArray &pointArray) {
    if(pointArray.empty())
        return;

    QByteArray buffer;
    QDataStream out(&buffer, QIODevice::ReadWrite);
    for(const auto& p : pointArray) {
        out << p.x << p.y << p.z;
    }

    test_socket_->writeDatagram(buffer, QHostAddress::LocalHost, 1234);
}
