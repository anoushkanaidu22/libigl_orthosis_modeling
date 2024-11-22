#include "rigging.h"
#include <igl/bbw.h>
#include <igl/boundary_conditions.h>

//NEEDS LOTS OF WORK

void Rigging::initialize(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F)
{
    //define joint positions (3 joints for finger)
    Eigen::MatrixXd C(3,3);
    C << V.row(0),                // Base joint
         V.row(V.rows()/2),       // Middle joint
         V.row(V.rows()-1);       // Tip joint
    
    //compute weights
    Eigen::MatrixXd W;
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 10;
    
    //compute boundary conditions
    Eigen::VectorXi b(3);
    b << 0, V.rows()/2, V.rows()-1;
    Eigen::MatrixXd bc(3,3);
    bc << 1,0,0,
          0,1,0,
          0,0,1;
    
    igl::bbw(V,F,b,bc,bbw_data,W);
    weights = W;
}
