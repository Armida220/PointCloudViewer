/*
 * Copyright (c) 2018 Ally of Intelligence Technology Co., Ltd. All rights reserved.
 *
 * Created by WuKun on 11/19/18.
 * Contact with:wk707060335@gmail.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http: *www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
*/

#ifndef POINTCLOUDVIEWER_NETWORKMANAGER_H
#define POINTCLOUDVIEWER_NETWORKMANAGER_H

#include <QtCore/QObject>
#include <QtNetwork/QUdpSocket>

#include "Common.h"

class NetworkManager : public QObject {
    Q_OBJECT
public:
    explicit NetworkManager(QObject* parent = nullptr);
    ~NetworkManager() final;

    enum class MODE {
        POINTCLOUD,
        STATUSINFO
    };

    void setMode(const MODE& m);
    void setPortNum(unsigned short port_num);

    Q_DISABLE_COPY(NetworkManager);
private:
    void setPacket(const PointArray& pointArray);

    QScopedPointer<QUdpSocket> socket_;
    QScopedPointer<QUdpSocket> test_socket_;
    MODE mode;
Q_SIGNALS:
    void emitPointCloud(PointArray);
    void emitUAVPos(Point);
    void emitGPSLocation(Point);
    void emitSatelliteNum(QString);
    void emitRTKStatus(bool);
    void emitGPSLock(bool);
    void emitKineticAlignment(bool);

private Q_SLOTS:
    void getDracoPointCloudData();
    void getStatusInfo();
};


#endif //POINTCLOUDVIEWER_NETWORKMANAGER_H
