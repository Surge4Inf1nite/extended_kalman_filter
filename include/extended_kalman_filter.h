#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

#include <random>

// 扩展卡尔曼滤波器类定义
class ExtendedKalmanFilter {
public:
    // 构造函数，初始化状态和协方差
    ExtendedKalmanFilter(double initial_x, double initial_P);

    // 执行扩展卡尔曼滤波更新
    void update(double measurement, double control);

    // 获取当前的状态估计值
    double getState() const;

    // 获取当前的协方差估计值
    double getCovariance() const;
    
    // 模拟观测值，带高斯噪声
    double simulateMeasurement();

    // 更新真实状态
    void updateTrueState(double control);

private:
    // 滤波器状态变量
    double x_; // 状态估计
    double P_; // 协方差估计

    // 真实系统状态
    double true_x_; // 真实状态
    double true_P_; // 真实协方差

    // 随机数生成器和高斯噪声分布
    std::default_random_engine generator_;
    std::normal_distribution<double> gaussian_noise_;

    // 状态转移函数
    double stateTransition(double control);

    // 观测函数
    double measurementFunc(double x) const;

    // 雅可比矩阵计算（状态转移函数）
    double jacobianStateTransition() const;

    // 雅可比矩阵计算（观测函数）
    double jacobianMeasurement() const;
};

#endif // EXTENDED_KALMAN_FILTER_H


