#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <include/ROSNode.h>


#include "ROSNode.h"
#include "Tracer.h"

ROSNode::ROSNode(int argc, char **argv) :
    init_argc(argc),
    init_argv(argv) {

}

ROSNode::~ROSNode() {
    if(ros::isStarted()){
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
    std::cout << "ROSNode was safely deleted." << std::endl;
}

bool ROSNode::init(const QVector<RosTopicPair>& ros_topics) {
    ros::init(init_argc, init_argv, "PointCloudViewer");
    if(!ros::master::check()){
        std::cout << "RosNode waiting..." << std::endl;
        return false;
    }
    if (ros_topics.size() < 2)  return false;

    ros::start();
    ros::NodeHandle nodeHandle;

    pose_sub_ = nodeHandle.subscribe(ros_topics[0].first.toStdString(),
                                     ros_topics[0].second, &ROSNode::callbackGetUAVPose, this);

    gps_sub_ = nodeHandle.subscribe(ros_topics[1].first.toStdString(),
                                    ros_topics[1].second, &ROSNode::callbackGetGPSInfo, this);

    std::cout << "RosNode init with: " << std::endl;
    for(const auto& ros_topic : ros_topics)
    {
        std::cout << ros_topic.first.toStdString() << " " << ros_topic.second << std::endl;
    }
    //start thread
    QThread::start();
    return true;
}

void ROSNode::run() {
    ros::spin();
}

void ROSNode::callbackGetPointCloudData(const sensor_msgs::PointCloud2ConstPtr& input) {

    PointArray points;
    {
        pcl::PointCloud<pcl::PointXYZI> point_cloud;
        pcl::fromROSMsg(*input, point_cloud);

        for(const auto& p : point_cloud) {
            if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;

            points.push_back(Point(p.x, p.y, p.z));
        }
    }
    if(points.isEmpty()) return;

    emit emitPointCloud(points);
}

void ROSNode::callbackGetUAVPose(const nav_msgs::Odometry &msg) {
    const std::string& frame_id = msg.header.frame_id;

    Point position;
    position.x = msg.pose.pose.position.x;
    position.y = msg.pose.pose.position.y;
    position.z = msg.pose.pose.position.z;

    osg::Vec4d orientation;
    orientation.x() = msg.pose.pose.orientation.x;
    orientation.y() = msg.pose.pose.orientation.y;
    orientation.z() = msg.pose.pose.orientation.z;
    orientation.w() = msg.pose.pose.orientation.w;

    emit emitUAVPos(position);
}

void ROSNode::callbackGetGPSInfo(const nav_msgs::OdometryConstPtr &msg) {
    double lat, lon, height;
    lat = msg->pose.pose.position.x;
    lon = msg->pose.pose.position.y;
    height = msg->pose.pose.position.z;
    Point gps_location(lat, lon, height);
    emit emitGPSLocation(gps_location);

    auto satellite_num = static_cast<int>(msg->pose.covariance[4]);
    QString satellite_num_str = QString::number(satellite_num);
    emit emitSatelliteNum(satellite_num_str);

    auto rtk_valid_check = [&]() {
        bool position_int = round(msg->pose.covariance[5]) == 50;
        bool heading_int = round(msg->pose.covariance[7]) == 50;
        return position_int&&heading_int;
    };
    bool is_rtk_valid = rtk_valid_check();
    emit emitRTKStatus(is_rtk_valid);
}
