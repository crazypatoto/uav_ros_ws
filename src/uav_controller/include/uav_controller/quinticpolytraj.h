#ifndef QUINTICPOLYTRAJ_H
#define QUINTICPOLYTRAJ_H

#include <Eigen/Dense>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace Eigen;

class QuinticPolyTraj3D
{
private:
    Eigen::MatrixXd coeffX_, coeffY_, coeffZ_;
    Eigen::Vector3d initPos_, targetPos_;
    double timeSpan_;

public:
    QuinticPolyTraj3D(const Eigen::Vector3d &initPos, const Eigen::Vector3d &targetPos, const Eigen::Vector3d &initVel, const double timeSpan);
    Eigen::Vector3d getPosition(double t);
    Eigen::Vector3d getVelocity(double t);
    Eigen::Vector3d getAcceleration(double t);
    double totalTimeSpan();
};

#endif