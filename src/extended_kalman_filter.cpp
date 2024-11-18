#include "extended_kalman_filter.h"
#include <cmath>

/*
    1. 预测阶段
        1.1 状态转移函数：
            x[k] = x[k-1] + u[k] + 0.1 * x[k-1]^2
        1.2 先验估计协方差
            p[k] = p[k-1] + Q;
    2. 更新阶段
        2.1 计算卡尔曼增益
            2.1.1 计算状态转移函数雅可比矩阵
                df/dx = 1 + 0.2 * x;
            2.1.2 计算观测函数雅可比矩阵
                df/dx = 1 + 0.4 * x;
        2.2 更新最优估计
            2.2.1 计算观测值
                z[k] = x[k] + 0.2 * x[k]^2
        2.3 更新协方差矩阵
*/

// 构造函数，初始化状态和协方差
ExtendedKalmanFilter::ExtendedKalmanFilter(double initial_x, double initial_P)
    : x_(initial_x), P_(initial_P), true_x_(initial_x), true_P_(initial_P), 
      generator_(std::random_device{}()), gaussian_noise_(0.0, 0.1) {}

// 状态转移函数
// x[k] = x[k-1] + u[k] + 0.1 * x[k-1]^2
double ExtendedKalmanFilter::stateTransition(double control) {
    return x_ + control + 0.1 * x_ * x_;
}

// 观测函数
// z[k] = x[k] + 0.2 * x[k]^2
double ExtendedKalmanFilter::measurementFunc(double x) const {
    return x + 0.2 * x * x;
}

// 状态转移函数的雅可比矩阵
// df/dx = 1 + 0.2 * x
double ExtendedKalmanFilter::jacobianStateTransition() const {
    return 1.0 + 0.2 * x_;
}

// 观测函数的雅可比矩阵
// dh/dx = 1 + 0.4 * x
double ExtendedKalmanFilter::jacobianMeasurement() const {
    return 1.0 + 0.4 * x_;
}

// 扩展卡尔曼滤波更新函数
void ExtendedKalmanFilter::update(double measurement, double control) {
    // 1. 预测阶段
    double predicted_x = stateTransition(control);
    double J_f = jacobianStateTransition();
    double predicted_P = P_ + 0.1;  // 假设过程噪声 Q 为 0.1

    // 2. 计算卡尔曼增益
    double J_h = jacobianMeasurement();
    double K = predicted_P * J_h / (J_h * predicted_P * J_h + 0.1);  // 假设观测噪声 R 为 0.1

    // 3. 更新阶段
    x_ = predicted_x + K * (measurement - measurementFunc(predicted_x));
    P_ = (1 - K * J_h) * predicted_P;
}

// 获取当前的状态估计值
double ExtendedKalmanFilter::getState() const {
    return x_;
}

// 获取当前的协方差估计值
double ExtendedKalmanFilter::getCovariance() const {
    return P_;
}

// 模拟观测值，加入高斯噪声
double ExtendedKalmanFilter::simulateMeasurement() {
    // 使用真实状态 true_x_ 计算观测值，并加入高斯噪声
    double true_measurement = true_x_ + 0.2 * true_x_ * true_x_;
    double noise = gaussian_noise_(generator_);
    return true_measurement + noise;
}

// 更新真实状态
void ExtendedKalmanFilter::updateTrueState(double control) {
    true_x_ = true_x_ + control + 0.05 * true_x_ * true_x_;
    true_P_ += 0.05; // 假设真实过程噪声为 0.05
}
