#ifndef CUBICPOLYTRAJ_H
#define CUBICPOLYTRAJ_H

#include <Eigen/Dense>
#include <cstdlib>
#include <cstdio>

using namespace std;
using namespace Eigen;

class CubicPolyTraj3D
{
private:
    Eigen::MatrixXd coeffX_, coeffY_, coeffZ_;
    Eigen::Vector3d initPos_, targetPos_;
    double timeSpan_;

public:
    CubicPolyTraj3D(const Eigen::Vector3d &initPos, const Eigen::Vector3d &targetPos, const double timeSpan);
    Eigen::Vector3d getPosition(double t);
    Eigen::Vector3d getVelocity(double t);
    Eigen::Vector3d getAcceleration(double t);
    double totalTimeSpan();
};

#endif