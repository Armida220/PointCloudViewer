//
// Created by fanxia on 12/11/18.
//

#ifndef POST_DATA_POST_BAG_H
#define POST_DATA_POST_BAG_H


#pragma pack(1)
float scale = 1e-8;
typedef struct RawIMUFrame{
    double GPSTime;
    int GyroX;
    int GyroY;
    int GyroZ;
    int AccelX;
    int AccelY;
    int AccelZ;
} RawIMUFrame_t;

typedef struct PoseFrame{
    double UTCtime;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
} PoseFrame_t;

typedef struct PointT{
    uint64_t timestamp;
    float x;
    float y;
    float z;
} PointT_t;

#pragma pack()


#endif //POST_DATA_POST_BAG_H
