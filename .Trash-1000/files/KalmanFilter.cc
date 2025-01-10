// #include "KalmanFilter.h"
// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <Eigen/Dense>

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
//     KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas) 
//         : dt(dt), u(u_x, u_y), x(Eigen::Vector4d::Zero()) {
        
//         A << 1, 0, dt, 0,
//              0, 1, 0, dt,
//              0, 0, 1, 0,
//              0, 0, 0, 1;

//         B << std::pow(dt, 2) / 2, 0,
//              0, std::pow(dt, 2) / 2,
//              dt, 0,
//              0, dt;

//         H << 1, 0, 0, 0,
//              0, 1, 0, 0;

//         Q << std::pow(dt, 4) / 4, 0, std::pow(dt, 3) / 2, 0,
//              0, std::pow(dt, 4) / 4, 0, std::pow(dt, 3) / 2,
//              std::pow(dt, 3) / 2, 0, std::pow(dt, 2), 0,
//              0, std::pow(dt, 3) / 2, 0, std::pow(dt, 2);
//         Q *= std::pow(std_acc, 2);

//         R << std::pow(x_std_meas, 2), 0,
//              0, std::pow(y_std_meas, 2);

//         P = Eigen::Matrix4d::Identity() * 500;
//     }

//     // void update_measurement_noise(double confidence) {
//     //     R << std::pow(confidence, 2), 0,
//     //          0, std::pow(confidence, 2);
//     // }

//     Eigen::Vector2d KalmanFilter::predict() {
//         x = A * x + B * u;
//         P = A * P * A.transpose() + Q;
//         return x.head<2>();
//     }

//     void KalmanFilter::update(const Eigen::Vector2d& z) {
//         Eigen::Matrix2d S = H * P * H.transpose() + R;
//         Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
//         x = x + K * (z - H * x);
//         Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
//         P = (I - K * H) * P;
//     }
// };

#include "KalmanFilter.h"
#include <cmath>

namespace artery
{

KalmanFilter::KalmanFilter(double dt, double u_x, double u_y, double std_acc, double x_std_meas, double y_std_meas)
    : dt(dt), u(u_x, u_y), x(Eigen::Vector4d::Zero()) {

    A << 1, 0, dt, 0,
         0, 1, 0, dt,
         0, 0, 1, 0,
         0, 0, 0, 1;

    B << std::pow(dt, 2) / 2, 0,
         0, std::pow(dt, 2) / 2,
         dt, 0,
         0, dt;

    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Q << std::pow(dt, 4) / 4, 0, std::pow(dt, 3) / 2, 0,
         0, std::pow(dt, 4) / 4, 0, std::pow(dt, 3) / 2,
         std::pow(dt, 3) / 2, 0, std::pow(dt, 2), 0,
         0, std::pow(dt, 3) / 2, 0, std::pow(dt, 2);
    Q *= std::pow(std_acc, 2);

    R << std::pow(x_std_meas, 2), 0,
         0, std::pow(y_std_meas, 2);

    P = Eigen::Matrix4d::Identity() * 500;
}

Eigen::Vector2d KalmanFilter::predict() {
    x = A * x + B * u;
    P = A * P * A.transpose() + Q;
    return x.head<2>();
}

void KalmanFilter::update(const Eigen::Vector2d& z) {
    Eigen::Matrix2d S = H * P * H.transpose() + R;
    Eigen::Matrix<double, 4, 2> K = P * H.transpose() * S.inverse();
    x = x + K * (z - H * x);
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    P = (I - K * H) * P;
}

}
