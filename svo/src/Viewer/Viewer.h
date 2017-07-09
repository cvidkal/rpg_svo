//
// Created by fg on 08/07/2017.
//

#ifndef SLAM_VIEWER_H
#define SLAM_VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Dense>


class Viewer {
    typedef std::pair<double,Eigen::Matrix4d> Estimate;
public:
    Viewer(int width, int height);

    ~Viewer();

    void run();

    void pushIMUEstimate(double timestamp, Eigen::Matrix<double, 4, 4, 0, 4, 4> Twi);

    void pushVisionEstimate(double timestamp, Eigen::Matrix<double, 4, 4, 0, 4, 4> Twf);

    void setCameraK(const Eigen::Matrix3d&K){K_ = K; Kinv_ = K.inverse();}
private:
    void mainLoop();

    bool mbRunning;

    pangolin::OpenGlRenderState cameraMatrix_;

    int Mode_;
    std::vector<Estimate,Eigen::aligned_allocator<Estimate>> estimates_imu_,estimate_vision_;

    Eigen::Matrix3d Kinv_,K_;
};


#endif //SLAM_VIEWER_H
