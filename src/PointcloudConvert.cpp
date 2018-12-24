#include <deque>
#include <fstream>
#include <iostream>

#include <liblas/liblas.hpp>
#include "Config.h"

#include "PostBag.h"
#include "MsfSensorTypes.h"
#include "PointcloudConvert.h"

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

uint64_t init_time;

std::shared_ptr<Config> Config::config_ = nullptr;

std::deque<POSSMeas_::Ptr> poseQue;

Eigen::Isometry3d lidarImuTrans;

POSSMeas::Ptr poseInterpolation(uint64_t timestamp){
    while (poseQue.size() > 1){
        if ((*(poseQue.begin() + 1))->timestamp > timestamp) {
            break;
        }
        else {
            poseQue.pop_front();
        }
    }

    if (poseQue.size() > 1 && poseQue.front()->timestamp < timestamp) {

        double t = (double)(timestamp-poseQue.front()->timestamp)*1e-9 / (double)((*(poseQue.begin() + 1))->timestamp - poseQue.front()->timestamp)*1e-9;
        POSSMeas::Ptr pose(new POSSMeas);
        pose->p = poseQue.front()->p + t*((*(poseQue.begin() + 1))->p - poseQue.front()->p);
        pose->q = poseQue.front()->q.slerp(t, (*(poseQue.begin() + 1))->q);
        return pose;
    }
    return nullptr;
}

void readPose(const std::string& pose_file){
    std::ifstream poseFS(pose_file.c_str());
    double timestamp, yaw, pitch, roll;
    while (poseFS.peek() != EOF) {
        POSSMeas::Ptr outposs(new POSSMeas);
        poseFS >> timestamp >> outposs->p[0] >> outposs->p[1] >> outposs->p[2]
              >> yaw >> pitch >> roll;
        Eigen::AngleAxisd rollAngle(DEG2RAD(roll), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(DEG2RAD(pitch), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(DEG2RAD(yaw), Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
        outposs->timestamp = (uint64_t)(timestamp*1e9);
        outposs->q = q;
        poseQue.push_back(outposs);
//        std::cout << outposs->p[0] << outposs->p[1] << outposs->p[2]<< std::endl;
    }
    if(poseQue.empty()) return;
    poseQue.pop_back();
    poseFS.close();
}

Eigen::Vector3d transformPoint(const Eigen::Vector3d& point, const POSSMeas::Ptr& pose){
    Eigen::Isometry3d imuWorldTrans;
    imuWorldTrans = pose->q.toRotationMatrix();
    imuWorldTrans.translation() = pose->p;
    Eigen::Vector3d trans_point;
    trans_point = imuWorldTrans * lidarImuTrans * point;
    return trans_point;
}

void writeLasFromPointCloud(const char* strInPointsName, const char* strOutLasName, uint64_t time_min, uint64_t time_max)
{
    std::ifstream ifs(strInPointsName, std::ios::in | std::ios::binary);
    std::ofstream ofs(strOutLasName, std::ios::out | std::ios::binary);
    if (!ofs.is_open())
    {
        std::cout << "err  to  open  file  las....." << std::endl;
        return;
    }
    liblas::Header header;
    header.SetDataFormatId(liblas::ePointFormat0);
    liblas::Writer writer(ofs, header);
    liblas::Point point(&header);
    size_t i = 0;
    PointT_t point_t;
    while(ifs.read((char *)&point_t, sizeof(point_t))){
        if (ifs.gcount() == sizeof(point_t)){
            if (point_t.timestamp <= time_min || point_t.timestamp >= time_max){
                //std::cout << "point not in period" << std::endl;
                continue;
            }
            Eigen::Vector3d point_lidar(point_t.x, point_t.y, point_t.z);
            POSSMeas::Ptr pose = poseInterpolation(point_t.timestamp);
            if (pose == nullptr) {
                    //std::cout << point_t.timestamp << " no pose" << std::endl;
                continue;
            }
            Eigen::Vector3d trans_point;
            trans_point = transformPoint(point_lidar, pose);
            point.SetCoordinates(trans_point[0], trans_point[1], trans_point[2]);
            writer.WritePoint(point);
            i++;
        }
        else {
            std::cout << "all points" << std::endl;
        }
    }
    header.SetPointRecordsCount(i);
    writer.SetHeader(header);
}

void ConvertToLas(const std::string& bag_path, std::function<void(void)> callback) {
    std::string /*bag_path,*/ out_file, pose_file, points_file;
    std::string laserTopic;
    double y, p, r;
    double tx, ty, tz;
    double lidar_delay, lidar_period;

    //bag_path = Config::get<std::string>("bag_path");
    out_file = bag_path + Config::get<std::string>("cloud_out", "./outcloud.las");
    pose_file = bag_path + Config::get<std::string>("pose_file");
    points_file = bag_path + Config::get<std::string>("points_file");
    y = Config::get<double>("lidar_yaw", 180);
    p = Config::get<double>("lidar_pitch", 0);
    r = Config::get<double>("lidar_roll", 90);
    tx = Config::get<double>("lidar_x", 0.047);
    ty = Config::get<double>("lidar_y", 0.1209);
    tz = Config::get<double>("lidar_z", 0.02571);
    lidar_delay = Config::get<double>("lidar_delay", 30);
    lidar_period = Config::get<double>("lidar_period", 80);

    Eigen::Vector3d ea0(DEG2RAD(y), DEG2RAD(p), DEG2RAD(r)); //yaw,pitching,roll
//    Eigen::Vector3d ea0(DEG2RAD(-180), DEG2RAD(15), DEG2RAD(-180)); //yaw,pitching,roll
    Eigen::Matrix3d R;
    R = Eigen::AngleAxisd(ea0[0], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(ea0[1], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(ea0[2], Eigen::Vector3d::UnitX());
    lidarImuTrans = R;
    lidarImuTrans.translation() = Eigen::Vector3d(tx,ty, tz);

    readPose(pose_file);
    uint64_t time_min = poseQue.front()->timestamp;
    uint64_t time_max = poseQue.back()->timestamp;
    writeLasFromPointCloud(points_file.c_str(), out_file.c_str(), time_min, time_max);

    callback();
    std::cout << "convert done!" << std::endl;
}
