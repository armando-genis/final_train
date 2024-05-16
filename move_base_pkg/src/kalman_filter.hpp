#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <iostream>

class KalmanFilter {
public:
    KalmanFilter(double process_noise, double measurement_noise_aruco, double measurement_noise_motor);

    void initialize(double initial_state, double initial_covariance);
    double update(double motor_measurement, double aruco_measurement, bool aruco_detected);

private:
    // Matrices for the Kalman filter
    Eigen::Matrix<double, 2, 2> A; // State transition matrix
    Eigen::Matrix<double, 2, 2> P; // Error covariance matrix
    Eigen::Matrix<double, 2, 2> Q; // Process noise covariance matrix
    Eigen::Matrix<double, 1, 2> H_motor; // Observation matrix for motor measurement
    Eigen::Matrix<double, 1, 2> H_aruco; // Observation matrix for aruco measurement
    Eigen::Matrix<double, 1, 1> R_motor; // Measurement noise covariance matrix for motor
    Eigen::Matrix<double, 1, 1> R_aruco; // Measurement noise covariance matrix for aruco
    Eigen::Matrix<double, 2, 1> x; // State estimate
    Eigen::Matrix<double, 2, 1> K; // Kalman gain
    Eigen::Matrix<double, 1, 1> y; // Measurement residual
};

KalmanFilter::KalmanFilter(double process_noise, double measurement_noise_aruco, double measurement_noise_motor) {
    // State transition matrix
    A << 1, 0,
         0, 1;

    // Process noise covariance matrix
    Q << process_noise, 0,
         0, process_noise;

    // Observation matrix for motor measurement
    H_motor << 1, 0;

    // Observation matrix for aruco measurement
    H_aruco << 0, 1;

    // Measurement noise covariance matrix for motor
    R_motor << measurement_noise_motor;

    // Measurement noise covariance matrix for aruco
    R_aruco << measurement_noise_aruco;
}

void KalmanFilter::initialize(double initial_state, double initial_covariance) {
    // Initial state estimate
    x << initial_state, initial_state;

    // Initial error covariance matrix
    P << initial_covariance, 0,
         0, initial_covariance;
}

double KalmanFilter::update(double motor_measurement, double aruco_measurement, bool aruco_detected) {
    // Prediction step
    x = A * x;
    P = A * P * A.transpose() + Q;

    // Measurement update for motor
    y(0) = motor_measurement - H_motor * x;
    K = P * H_motor.transpose() * (H_motor * P * H_motor.transpose() + R_motor).inverse();
    x = x + K * y;
    P = (Eigen::Matrix2d::Identity() - K * H_motor) * P;

    // If an aruco measurement is available, update with aruco measurement
    if (aruco_detected) {
        y(0) = aruco_measurement - H_aruco * x;
        K = P * H_aruco.transpose() * (H_aruco * P * H_aruco.transpose() + R_aruco).inverse();
        x = x + K * y;
        P = (Eigen::Matrix2d::Identity() - K * H_aruco) * P;
    }

    return x(0);
}

#endif // KALMAN_FILTER_HPP
