#pragma once
#include <Eigen/Core>

class Rigging {
public:
    void initialize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
    Eigen::MatrixXd getWeights() const { return weights; }
    
private:
    Eigen::MatrixXd weights;
    void computeWeights(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);
};
