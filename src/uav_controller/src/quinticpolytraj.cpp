#include "uav_controller/quinticpolytraj.h"

QuinticPolyTraj3D::QuinticPolyTraj3D(const Eigen::Vector3d &initPos, const Eigen::Vector3d &targetPos, const Eigen::Vector3d &initVel, const double timeSpan)
{
    initPos_ = initPos;
    targetPos_ = targetPos;
    timeSpan_ = timeSpan;

    Eigen::MatrixXd t(6, 6);
    t << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
        1.0, timeSpan, pow(timeSpan, 2), pow(timeSpan, 3), pow(timeSpan, 4), pow(timeSpan, 5),
        0, 1.0, 2.0 * timeSpan, 3.0 * pow(timeSpan, 2), 4.0 * pow(timeSpan, 3), 5.0 * pow(timeSpan, 4),
        0, 0.0, 2.0, 6.0 * timeSpan, 12.0 * pow(timeSpan, 2), 20.0 * pow(timeSpan, 3);

    Eigen::MatrixXd qX(6, 1), qY(6, 1), qZ(6, 1);
    qX << initPos(0), initVel(0), 0, targetPos(0), 0, 0;
    qY << initPos(1), initVel(1), 0, targetPos(1), 0, 0;
    qZ << initPos(2), initVel(2), 0, targetPos(2), 0, 0;

    coeffX_ = t.inverse() * qX;
    coeffY_ = t.inverse() * qY;
    coeffZ_ = t.inverse() * qZ;
}

Eigen::Vector3d QuinticPolyTraj3D::getPosition(double t)
{
    if (t <= 0)
        return initPos_;
    else if (t >= timeSpan_)
        return targetPos_;

    Eigen::Vector3d p;

    p << coeffX_(0) + coeffX_(1) * t + coeffX_(2) * pow(t, 2) + coeffX_(3) * pow(t, 3) + coeffX_(4) * pow(t, 4) + coeffX_(5) * pow(t, 5),
        coeffY_(0) + coeffY_(1) * t + coeffY_(2) * pow(t, 2) + coeffY_(3) * pow(t, 3) + coeffY_(4) * pow(t, 4) + coeffY_(5) * pow(t, 5),
        coeffZ_(0) + coeffZ_(1) * t + coeffZ_(2) * pow(t, 2) + coeffZ_(3) * pow(t, 3) + coeffZ_(4) * pow(t, 4) + coeffZ_(5) * pow(t, 5);

    return p;
}

Eigen::Vector3d QuinticPolyTraj3D::getVelocity(double t)
{
    if ((t <= 0) || (t >= timeSpan_))
        return Eigen::Vector3d(0, 0, 0);

    Eigen::Vector3d v;

    v << coeffX_(1) + 2.0 * coeffX_(2) * t + 3.0 * coeffX_(3) * pow(t, 2) + 4.0 * coeffX_(4) * pow(t, 3) + 5.0 * coeffX_(5) * pow(t, 4),
        coeffY_(1) + 2.0 * coeffY_(2) * t + 3.0 * coeffY_(3) * pow(t, 2) + 4.0 * coeffY_(4) * pow(t, 3) + 5.0 * coeffY_(5) * pow(t, 4),
        coeffZ_(1) + 2.0 * coeffZ_(2) * t + 3.0 * coeffZ_(3) * pow(t, 2) + 4.0 * coeffZ_(4) * pow(t, 3) + 5.0 * coeffZ_(5) * pow(t, 4);

    return v;
}

Eigen::Vector3d QuinticPolyTraj3D::getAcceleration(double t)
{
    if ((t <= 0) || (t >= timeSpan_))
        return Eigen::Vector3d(0, 0, 0);

    Eigen::Vector3d a;

    a << 2 * coeffX_(2) + 6.0 * coeffX_(3) * t + 12.0 * coeffX_(4) * pow(t, 2) + 20.0 * coeffX_(5) * pow(t, 3),
        2 * coeffY_(2) + 6.0 * coeffY_(3) * t + 12.0 * coeffY_(4) * pow(t, 2) + 20.0 * coeffY_(5) * pow(t, 3),
        2 * coeffZ_(2) + 6.0 * coeffZ_(3) * t + 12.0 * coeffZ_(4) * pow(t, 2) + 20.0 * coeffZ_(5) * pow(t, 3);

    return a;
}

double QuinticPolyTraj3D::totalTimeSpan()
{
    return timeSpan_;
}