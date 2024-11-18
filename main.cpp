#include <iostream>
#include <cstdlib>
#include "extended_kalman_filter.h"

int main() {
    // 初始化扩展卡尔曼滤波器
    ExtendedKalmanFilter ekf(1.0, 1.0);

    // 控制输入和观测值
    double control;
    double measurement;

    // 扩展卡尔曼滤波迭代过程
    for (int i = 0; i < 10; i++) {
        // 模拟控制输入
        control = 0.5 + 0.1 * i;

        // 更新真实状态
        ekf.updateTrueState(control);

        // 模拟观测值
        measurement = ekf.simulateMeasurement();

        // 更新滤波器
        ekf.update(measurement, control);

        // 输出状态估计值和真实值
        std::cout << "Iteration " << i << ": Estimated state = " << ekf.getState()
                  << ", Covariance = " << ekf.getCovariance() << std::endl;
    }

    return 0;
}
