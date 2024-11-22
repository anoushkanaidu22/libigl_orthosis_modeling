#include "physics.h"
#include <igl/massmatrix.h>
#include <Eigen/Sparse>

//NEEDS LOTS OF WORK

void Physics::initialize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F,
                        const Eigen::MatrixXd& W)
{
    rest_vertices = V;
    current_vertices = V;
    weights = W;
    computeMassMatrix(F);
}

void Physics::computeMassMatrix(const Eigen::MatrixXi& F)
{
    igl::massmatrix(rest_vertices, F, igl::MASSMATRIX_TYPE_BARYCENTRIC,
                    mass_matrix);
}

void Physics::update()
{
    //simiplified physics update (placeholder)
    //add gravity
    Eigen::Vector3d gravity(0,-9.81,0);
    current_vertices.rowwise() += gravity.transpose() * 0.001;
    
    // Enforce constraints
    enforceConstraints();
}

void Physics::enforceConstraints()
{
    //simplified joint angle constraints
    double max_angle = 45.0 * M_PI / 180.0;  // 45 degrees
    
    //apply joint limits using weights
    for(int i = 0; i < current_vertices.rows(); i++)
    {
        //simplified constraint enforcement
        Eigen::Vector3d pos = current_vertices.row(i);
        if(weights(i,0) > 0.5)  // Base joint influence
        {
            current_vertices.row(i) = rest_vertices.row(i);
        }
    }
}
