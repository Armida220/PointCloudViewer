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

#ifndef POINTCLOUDVIEWER_ROSNODE_H
#define POINTCLOUDVIEWER_ROSNODE_H

#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <QtCore/QThread>
#include <QtCore/QString>
#include <QtCore/QVector>
#include <QtCore/QPair>
#include <Common.h>

typedef QPair<QString, unsigned int> RosTopicPair;

class ROSNode : public QThread {
    Q_OBJECT
public:
    ROSNode(int argc, char **argv);
    ~ROSNode() final;

    bool init(const QVector<RosTopicPair>& ros_topics);

    Q_DISABLE_COPY(ROSNode)
protected:
    void run() final;

private:
    void callbackGetPointCloudData(const sensor_msgs::PointCloud2ConstPtr& msg);
    void callbackGetUAVPose(const nav_msgs::Odometry& msg);
    void callbackGetGPSInfo(const nav_msgs::OdometryConstPtr& msg);

    int init_argc;
    char **init_argv;

    ros::Subscriber pose_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber origin_sub_;

Q_SIGNALS:
    void emitPointCloud(PointArray);
    void emitUAVPos(Point);
    void emitGPSLocation(Point);
    void emitSatelliteNum(QString);
    void emitRTKStatus(bool);
};


#endif //POINTCLOUDVIEWER_ROSNODE_H
