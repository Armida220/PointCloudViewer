//
// Created by zhkj on 8/28/18.
//

#ifndef ETHZASL_MSF_NOROS_MSF_TYPES_H
#define ETHZASL_MSF_NOROS_MSF_TYPES_H
#include <Eigen/Dense>
#include <memory>

typedef struct Meas_ {
    typedef std::shared_ptr<Meas_> Ptr;
    uint64_t timestamp;
    uint32_t seq;
    enum { IMU, POSITION, POSS};
    virtual int type() const = 0;
} Meas;

typedef struct IMUMeas_ : public Meas_ {
    typedef std::shared_ptr<IMUMeas_> Ptr;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
    virtual int type() const { return IMU; }
} IMUMeas;

typedef struct POSITIONMeas_ : public Meas_ {
    typedef std::shared_ptr<POSITIONMeas_> Ptr;
    Eigen::Vector3d p;
    Eigen::Matrix3d p_cov;
    virtual int type() const { return POSITION; }
} POSITIONMeas;

typedef struct POSSMeas_ : public Meas_ {
    typedef std::shared_ptr<POSSMeas_> Ptr;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    virtual int type() const { return POSS; }
} POSSMeas;

#endif //ETHZASL_MSF_NOROS_MSF_TYPES_H
