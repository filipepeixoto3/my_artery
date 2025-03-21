// #ifndef KALMANFILTER_H
// #define KALMANFILTER_H

// #include <Eigen/Dense>
// #include <vector>

// class KalmanFilter {
// private:
//     // Private member variables
//     double dt;  // Sampling time
//     Eigen::Vector2d u;  // Control input (acceleration in x and y)
//     Eigen::Vector4d x;  // State vector [x, y, vx, vy]
//     Eigen::Matrix4d A;  // State transition matrix
//     Eigen::Matrix<double, 4, 2> B;  // Control input matrix
//     Eigen::Matrix<double, 2, 4> H;  // Measurement mapping matrix
//     Eigen::Matrix4d Q;  // Process noise covariance
//     Eigen::Matrix2d R;  // Measurement noise covariance
//     Eigen::Matrix4d P;  // Covariance matrix
//     std::vector<Eigen::Vector2d> pred_tracks;  // Predicted tracks

// public:
//     // Constructor
//     KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas);

//     // Public methods
//     void update_measurement_noise(double confidence);  // Update R based on confidence
//     Eigen::Vector2d predict();  // Predict the next state
//     Eigen::Vector2d update(const Eigen::Vector2d& z);  // Update the state based on measurement

//     // Optionally add getters for accessing internal states if needed
//     Eigen::Vector4d getState() const { return x; }
//     Eigen::Matrix4d getCovariance() const { return P; }
// };

// #endif  // KALMANFILTER_H

// KalmanFilter.h
#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <Eigen/Dense>

namespace artery
{

class KalmanFilter {
private:
    double dt;
    Eigen::Vector2d u;
    Eigen::Vector4d x;
    Eigen::Matrix4d A;
    Eigen::Matrix<double, 4, 2> B;
    Eigen::Matrix<double, 2, 4> H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Matrix4d P;

public:
    // Default constructor
    KalmanFilter() : dt(1.0), u(0.0, 0.0), x(Eigen::Vector4d::Zero()), A(Eigen::Matrix4d::Identity()),
                     B(Eigen::Matrix<double, 4, 2>::Zero()), H(Eigen::Matrix<double, 2, 4>::Zero()),
                     Q(Eigen::Matrix4d::Zero()), R(Eigen::Matrix2d::Zero()), P(Eigen::Matrix4d::Identity() * 500) {}

    // Parameterized constructor
    KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas);

    Eigen::Vector2d predict();
    void update(const Eigen::Vector2d& z);
    Eigen::Vector2d getPosition();
};

}

#endif // KALMANFILTER_H