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

bool ROSNode::init(const std::string& topic, unsigned int queue_size) {
    ros::init(init_argc, init_argv, "PointCloudViewer");
    if(!ros::master::check()){
        std::cout << "RosNode waiting..." << std::endl;
        return false;
    }
    ros::start();

    ros::NodeHandle nodeHandle;

    point_cloud_data_sub = nodeHandle.subscribe(topic, queue_size, &ROSNode::callbackGetUAVPose, this);

    std::cout << "RosNode init with topic: " << topic << " and queue size: " << queue_size << std::endl;
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
