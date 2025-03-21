

// // KalmanFilter.h
// #ifndef KALMANFILTER_H
// #define KALMANFILTER_H

// #include <Eigen/Dense>

// namespace artery
// {

// class KalmanFilter {
// private:
//     double dt;
//     Eigen::Vector2d u;
//     Eigen::Vector4d x;
//     Eigen::Matrix4d A;
//     Eigen::Matrix<double, 4, 2> B;
//     Eigen::Matrix<double, 2, 4> H;
//     Eigen::Matrix4d Q;
//     Eigen::Matrix2d R;
//     Eigen::Matrix4d P;

// public:
//     // Default constructor
//     KalmanFilter() : dt(1.0), u(0.0, 0.0), x(Eigen::Vector4d::Zero()), A(Eigen::Matrix4d::Identity()),
//                      B(Eigen::Matrix<double, 4, 2>::Zero()), H(Eigen::Matrix<double, 2, 4>::Zero()),
//                      Q(Eigen::Matrix4d::Zero()), R(Eigen::Matrix2d::Zero()), P(Eigen::Matrix4d::Identity() * 500) {}

//     // Parameterized constructor
//     KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas);

//     Eigen::Vector2d predict();
//     void update(const Eigen::Vector2d& z);
//     Eigen::Vector2d getPosition();
// };

// }

// #endif // KALMANFILTER_H

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

namespace artery {

class KalmanFilter {
private:
    double dt;                                   // Time step
    Eigen::Vector2d u;                          // Control input (acceleration)
    Eigen::Vector4d x;                          // State vector [x, y, vx, vy]
    Eigen::Matrix4d A;                          // State transition matrix
    Eigen::Matrix<double, 4, 2> B;              // Control input matrix
    Eigen::Matrix<double, 2, 4> H;              // Measurement matrix
    Eigen::Matrix4d Q;                          // Process noise covariance
    Eigen::Matrix2d R;                          // Measurement noise covariance
    Eigen::Matrix4d P;                          // Estimate error covariance

public:
    // Constructor
    KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas);

    // Predict step
    Eigen::Vector2d predict();

    // Update step
    void update(const Eigen::Vector2d& z);

    // Get position (x, y)
    Eigen::Vector2d getPosition();

    // Reset the filter (default or with custom initial state)
    // void reset(const Eigen::Vector4d& initial_state = Eigen::Vector4d::Zero());
};

} // namespace artery

#endif // KALMANFILTER_H
