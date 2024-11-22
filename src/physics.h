#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

class Physics {
public:
    void initialize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, 
                   const Eigen::MatrixXd& weights);
    void update();
    Eigen::MatrixXd getVertices() const { return current_vertices; }
    
private:
    Eigen::MatrixXd rest_vertices;
    Eigen::MatrixXd current_vertices;
    Eigen::SparseMatrix<double> mass_matrix;
    Eigen::MatrixXd weights;
    
    void computeMassMatrix(const Eigen::MatrixXi& F);
    void enforceConstraints();
};
